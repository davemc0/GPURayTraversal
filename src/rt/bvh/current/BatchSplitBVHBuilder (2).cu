#define FW_ENABLE_ASSERT

#include "bvh/BVHNode.hpp"
#include "bvh/BatchSplitBVHBuilder.hpp"
#include "base/Array.hpp"
#include "base/Timer.hpp"

#include <thrust/execution_policy.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/discard_iterator.h>
#include <thrust/iterator/zip_iterator.h>

#include <thrust/partition.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>
#include <thrust/transform_scan.h>

#define BHD __device__

// Copied from Platform.hpp since I don't want to pull that into the device.
// batch processing (how many ops at the price of one)
#define n_triBatchSize    1
#define n_nodeBatchSize   1
#define n_SAHTriangleCost 1.0f
#define n_SAHNodeCost     1.0f

using FW::S32;
using FW::F32;
using FW::AABB;

S32   BHD roundToTriangleBatchSize(S32 n) { return ((n + n_triBatchSize - 1) / n_triBatchSize)*n_triBatchSize; }
S32   BHD roundToNodeBatchSize(S32 n) { return ((n + n_nodeBatchSize - 1) / n_nodeBatchSize)*n_nodeBatchSize; }

float BHD getTriangleCost(S32 n) { return roundToTriangleBatchSize(n) * n_SAHTriangleCost; }
float BHD getNodeCost(S32 n) { return roundToNodeBatchSize(n) * n_SAHNodeCost; }
float BHD getCost(int numChildNodes, int numTris) { return getNodeCost(numChildNodes) + getTriangleCost(numTris); }

typedef thrust::tuple<AABB, S32, AABB, S32> BIBITuple;
typedef thrust::tuple<AABB*, S32*, AABB*, S32*> BIBIItTuple;
typedef thrust::zip_iterator<BIBIItTuple> BIBIZipIt;

typedef thrust::tuple<float, S32, S32> FIITuple;

struct BoundsToCost
{
    BHD FIITuple operator()(BIBITuple x) // rightBounds, rightIdx, leftBounds, leftIdx
    {
        float rightA = thrust::get<0>(x).area();
        float leftA = thrust::get<2>(x).area();
        S32 rightN = thrust::get<1>(x);
        S32 leftN = thrust::get<3>(x);
        // This is just SAH of the two children. Need to add nodeSAH for it to be a full SAH. Also it's not scaled by root bounds.
        F32 childSAH = leftA * getTriangleCost(leftN) + rightA * getTriangleCost(rightN);
        // Add nodeSAH - OPT: Instead, subtract nodeSAH from leafSAH to avoid computing nodeBounds here
        AABB nodeBounds = thrust::get<0>(x) + thrust::get<2>(x);
        F32 nodeSAH = nodeBounds.area() * getNodeCost(2);
        F32 sah = childSAH + nodeSAH;
        // F32 sah = childSAH;
        return thrust::make_tuple(sah, leftN, rightN);
    }
};

void FW::BatchSplitBVHBuilder::initBBArrays(S32 maxN, FW::Scene* scene, FW::BVH& bvh)
{
    m_intArray.setManaged(true);
    m_intArray.resize(maxN * 10);
    m_boundsArray.setManaged(true);
    m_boundsArray.resize(maxN * 3);
    m_keysArray.setManaged(true);
    m_keysArray.resize(maxN * 2);

    cudaMemset(m_intArray.getPtr(), 0, sizeof(S32)  * m_intArray.getSize());
    cudaMemset(m_boundsArray.getPtr(), 0, sizeof(AABB) * m_boundsArray.getSize());
    cudaMemset(m_keysArray.getPtr(), 0, sizeof(U64)  * m_keysArray.getSize());
    
    m_refRightIdx = m_intArray.getPtr();
    m_refLeftIdx = m_refRightIdx + maxN;
    m_refGamma = m_refLeftIdx + maxN;
    m_refSegIdx = m_refGamma + maxN;
    m_segIdxBest = m_refSegIdx + maxN; // Per-reference index into segment vector
    m_segIdxNew = m_segIdxBest + maxN;
    m_segCostBest = (F32*)(m_segIdxNew + maxN);
    m_segCostNew = m_segCostBest + maxN;
    m_segStratRefIdx = (S32*)(m_segCostNew + maxN);
    m_refBounds = m_boundsArray.getPtr();
    m_refRightBounds = m_refBounds + maxN;
    m_refLeftBounds = m_refRightBounds + maxN;
    m_refKeys = m_keysArray.getPtr();
    m_segKeys = m_refKeys + maxN;

    bvh.getTriIndices().setManaged(true);
    bvh.getTriIndices().resize(maxN);
    m_refTriIdx = bvh.getTriIndices().getPtr();

    m_tris = (const Vec3i*)scene->getTriVtxIndexBuffer().getCudaPtr();
    m_verts = (const Vec3f*)scene->getVtxPosBuffer().getCudaPtr();

    cuMemsetD32((CUdeviceptr)m_refGamma, FW_S32_MIN, maxN); // Set gamma to FW_S32_MIN
}

void FW::BatchSplitBVHBuilder::freeArrays()
{
    // printf("freeBBArrays()\n");
    m_intArray.reset(0);
    m_boundsArray.reset(0);
    m_keysArray.reset(0);
}

void FW::BatchSplitBVHBuilder::doGeneration(S32& N, S32& nSegments, S32 level)
{
    S32*  refTriIdx = m_refTriIdx;
    S32*  refRightIdx = m_refRightIdx;
    S32*  refLeftIdx = m_refLeftIdx;
    S32*  refGamma = m_refGamma;
    S32*  refSegIdx = m_refSegIdx;
    S32*  segIdxBest = m_segIdxBest;
    S32*  segIdxNew = m_segIdxNew;
    F32*  segCostBest = m_segCostBest;
    F32*  segCostNew = m_segCostNew;
    S32*  segStratRefIdx = m_segStratRefIdx;
    AABB* refBounds = m_refBounds;
    AABB* refRightBounds = m_refRightBounds;
    AABB* refLeftBounds = m_refLeftBounds;
    U64*  refKeys = m_refKeys;
    U64*  segKeys = m_segKeys;

    typedef thrust::tuple<S32, S32, AABB, U64>    TGBKTuple;
    typedef thrust::tuple<S32*, S32*, AABB*, U64*> TGBKItTuple;
    typedef thrust::zip_iterator<TGBKItTuple> TGBKZipIt;
    TGBKZipIt refsTGBK(thrust::make_tuple(refTriIdx, refGamma, refBounds, refKeys));

    auto OneIt = thrust::make_constant_iterator((S32)1);

    // Remove degenerates.
    // OPT: For Sweep builder move this out of the loop. If so, for speed, change it to not be a stable_partition. Split builder makes new refs.
    // OPT: Could make this part of sort predicate and use custom iterator to count how many are rejected
    auto mid = thrust::stable_partition(thrust::device, refsTGBK, refsTGBK + N, [] BHD(const TGBKTuple r) {
        Vec3f size = thrust::get<2>(r).max() - thrust::get<2>(r).min();
        return !(min(size) < 0.0f || sum(size) == max(size));
    });

    S32 newN = thrust::get<0>(mid.get_iterator_tuple()) - refTriIdx;
    if (newN != N) printf("%d => %d\n", N, newN);
    N = newN;

    // Try object split in each dimension
    for (int dim = 0; dim < 3; dim++) {

        // Sort in given dimension
        thrust::sort(thrust::device,
            refsTGBK, refsTGBK + N,
            [dim] BHD(TGBKTuple a, TGBKTuple b) {
            F32 ca = thrust::get<2>(a).min()[dim] + thrust::get<2>(a).max()[dim];
            F32 cb = thrust::get<2>(b).min()[dim] + thrust::get<2>(b).max()[dim];
            U64 ka = thrust::get<3>(a);
            U64 kb = thrust::get<3>(b);
            return (ka < kb) || (ka == kb && (ca < cb || (ca == cb && thrust::get<0>(a) < thrust::get<0>(b))));
        });

        // Sweep right to left and determine bounds; refRightIdx is offset from right edge of segment
        // refLeftBounds[i] and refRightBounds[i] contain the two AABBs for splitting at i.

        typedef thrust::tuple<AABB, S32> BITuple;
        typedef thrust::tuple<AABB*, S32*> BIItTuple;
        typedef thrust::zip_iterator<BIItTuple> BIZipIt;

        auto BIRevIt(thrust::make_zip_iterator(thrust::make_tuple(thrust::make_reverse_iterator(refBounds + N), OneIt)));
        auto refRightOut(thrust::make_zip_iterator(thrust::make_tuple(thrust::make_reverse_iterator(refRightBounds + N), thrust::make_reverse_iterator(refRightIdx + N))));

        // OPT: What if I combined the two scans and pulled from both ends of segment at same time?
        thrust::inclusive_scan_by_key(thrust::device,
            thrust::make_reverse_iterator(refKeys + N), thrust::make_reverse_iterator(refKeys),
            BIRevIt,
            refRightOut,
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(BITuple a, BITuple b) { return thrust::make_tuple(thrust::get<0>(a) + thrust::get<0>(b), thrust::get<1>(a) + thrust::get<1>(b)); });

        // Sweep left to right and determine bounds; refLeftIdx is offset from left edge of segment
        BIZipIt refLeftOut(thrust::make_tuple(refLeftBounds, refLeftIdx));

        // OPT: Don't need to write leftIdx and rightIdx all three times.
        thrust::exclusive_scan_by_key(thrust::device,
            refKeys, refKeys + N,
            thrust::make_zip_iterator(thrust::make_tuple(refBounds, OneIt)),
            refLeftOut,
            thrust::make_tuple(AABB(), (S32)0),
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(BITuple a, BITuple b) { return thrust::make_tuple(thrust::get<0>(a) + thrust::get<0>(b), thrust::get<1>(a) + thrust::get<1>(b)); });

        // OPT: Store a segment's full AABB into its final BVHNode, since we know its location now

        // Select lowest SAH.
        BIBIZipIt BoundsIt(thrust::make_tuple(refRightBounds, refRightIdx, refLeftBounds, refLeftIdx));
        typedef thrust::discard_iterator<S32> IDisIt;
        IDisIt Dis;

        // OPT: Only need to write the keys out once. Could use discard_iterator on the other two dimensions.
        // OPT: refSegIdx is unneeded; should use a discard_iterator to get rid of refRightIdx output, but was getting errors.
        auto segValues = thrust::make_zip_iterator(thrust::make_tuple(dim == 0 ? segCostBest : segCostNew, dim == 0 ? segIdxBest : segIdxNew, refSegIdx));

        auto outEnd = thrust::reduce_by_key(thrust::device,
            refKeys, refKeys + N, thrust::make_transform_iterator(BoundsIt, BoundsToCost()),
            segKeys, segValues, // OPT: I don't use segKeys. Should use discard_iterator, but need nSegments.
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(FIITuple a, FIITuple b) { return thrust::get<0>(a) < thrust::get<0>(b) ? a :
                (thrust::get<0>(a) > thrust::get<0>(b) ? b :
                (abs(thrust::get<1>(a) - thrust::get<2>(a)) < abs(thrust::get<1>(b) - thrust::get<2>(b)) ? a : b)); });

        nSegments = outEnd.first - segKeys;
        U64 demoK = segKeys[0];
        S32 thisStrategy = stratObjectSplit | (dim << stratBitOffset);

        if (dim == 0) {
            // Compute the count
            auto IIZipIt = thrust::make_zip_iterator(thrust::make_tuple(segIdxBest, refSegIdx)); // These currently contain left and right counts per segment

            thrust::transform_exclusive_scan(thrust::device,
                IIZipIt, IIZipIt + nSegments,
                segStratRefIdx, [] BHD(auto v) { return thrust::get<0>(v) + thrust::get<1>(v); },
                (S32)thisStrategy, [thisStrategy] BHD(S32 a, S32 b) { return thisStrategy | (stratNumMask & (a + b)); });

            // Compute the SAH of making each segment a leaf
            S32 maxLeafSize = m_platform.getMaxLeafSize(), minLeafSize = m_platform.getMinLeafSize();
            thrust::for_each_n(thrust::device, thrust::counting_iterator<S32>((S32)0), nSegments, [=] BHD(S32 i) {
                S32 ind = stratNumMask & segStratRefIdx[i];
                S32 leafN = segIdxBest[i] + refSegIdx[i];
                F32 leafSAH = FW_F32_MAX;
                if (leafN <= minLeafSize)
                    leafSAH = FW_F32_MIN;
                else if (leafN <= maxLeafSize) {
                    AABB bounds = refRightBounds[ind];
                    leafSAH = bounds.area() * getTriangleCost(leafN);
                }

                if (leafSAH < segCostBest[i]) {
                    segCostBest[i] = leafSAH;
                    segIdxBest[i] = FW_S32_MAX;
                    segStratRefIdx[i] = stratLeaf | ind;
                }
            });
        }
        else {
            // OPT: Would rather do this as a conditional_iterator as part of reduce_by_key.
            thrust::for_each_n(thrust::device, thrust::counting_iterator<S32>((S32)0), nSegments, [=] BHD(S32 i) {
                if (segCostNew[i] < segCostBest[i]) {
                    segCostBest[i] = segCostNew[i];
                    segIdxBest[i] = segIdxNew[i];
                    segStratRefIdx[i] = thisStrategy | (stratNumMask & segStratRefIdx[i]);
                }
            });
        }

        printf("level=%d dim=%d nSegments=%d keys=%016llx\n", level, dim, nSegments, demoK);
        if (level > -30) {
            cudaDeviceSynchronize(); // XXX
            for (int i = 0; i < nSegments; i++) {
                printf("%d 0x%x %d %d %f %016llx\n", i, (U32)segStratRefIdx[i] >> stratBitOffset, stratNumMask & segStratRefIdx[i], segIdxBest[i], segCostBest[i], segKeys[i]);
            }
        }
    }

    // Count how many refs want each kind of strategy to give me indices to them after they're sorted
    // thrust::inclusive_scan with an output tuple with a value per strategy. Could fold it into the for_each_n and use atomic counters?

    // Make refSegIdx be the per-reference index into out*
    thrust::transform_inclusive_scan(thrust::device,
        thrust::make_counting_iterator((S32)0), thrust::make_counting_iterator((S32)N), refSegIdx,
        [refKeys] BHD(S32 i) { return (i == 0 || refKeys[i] == refKeys[i - 1]) ? 0 : 1; },
        [] BHD(S32 a, S32 b) { return a + b; });

    cudaDeviceSynchronize(); // XXX
    for (int i = 0; i < N; i++)
        printf("i=%d refSegIdx[i]=%d refLeftIdx[i]=%d refKeys[i]=%016llx\n", i, refSegIdx[i], refLeftIdx[i], refKeys[i]);

    // Sort each segment by its best dimension
    typedef thrust::tuple<S32, S32, AABB, U64, S32>    TGBKITuple;
    typedef thrust::tuple<S32*, S32*, AABB*, U64*, S32*> TGBKIItTuple;
    typedef thrust::zip_iterator<TGBKIItTuple> TGBKIZipIt;
    TGBKIZipIt refsTGBKI(thrust::make_tuple(refTriIdx, refGamma, refBounds, refKeys, refSegIdx));
    // XXX Need to make sure that for multi-reference leaves the right ref sorts to the end to make gamma work.

    // OPT: Sort by strat to give good spans for doing separate algorithms in next pass; maybe lets us keep a sorted array per dim
    thrust::sort(thrust::device,
        refsTGBKI, refsTGBKI + N, [segStratRefIdx] BHD(TGBKITuple a, TGBKITuple b) {
        S32 sa = thrust::get<4>(a); // Segment index of each reference
        S32 sb = thrust::get<4>(b);

        int dim = (stratDimMask & segStratRefIdx[sa]) >> stratBitOffset;
        S32 la = segStratRefIdx[sa]; // sort by strategy and segment index in reference arrays (only strategy is relevant so far)
        S32 lb = segStratRefIdx[sb];

        F32 ca = thrust::get<2>(a).min()[dim] + thrust::get<2>(a).max()[dim]; // centroid in dim
        F32 cb = thrust::get<2>(b).min()[dim] + thrust::get<2>(b).max()[dim];

        return (la < lb) || (la == lb && (ca < cb || (ca == cb && thrust::get<0>(a) < thrust::get<0>(b))));
    });

    // Update Nactive here so only the active ones get their keys updated

    // Try to get rid of keys and just use segIdx. Have to be able to put them back in order to make gamma work.
    // XXX Will splits screw up gamma by inserting nodes between index and what it points to?

    // Update keys to partition each segment at the best location
    thrust::for_each(thrust::device, thrust::make_counting_iterator((S32)0), thrust::make_counting_iterator((S32)N),
        [=] BHD(S32 i) {
        S32 s = refSegIdx[i]; // Segment index in output arrays

        if (refLeftIdx[i] >= segIdxBest[s])
            // My offset within segment is to the right of the split index
            refKeys[i] = refKeys[i] | 1ull << (U64)(63 - level);

        // If I'm at the start or end of the segment and the gamma slot hasn't been claimed yet so I record the relative split location.
        if (refLeftIdx[i] == 0 && refGamma[i] == FW_S32_MIN)
            refGamma[i] = segIdxBest[s];
        if (refRightIdx[i] == 1 && refGamma[i] == FW_S32_MIN)
            refGamma[i] = segIdxBest[s] - refLeftIdx[i]; // The (negative) offset from i to the relative split location.
    });

    // printf("Done with generation %d.\n", level);
}

FW::BVHNode* FW::BatchSplitBVHBuilder::makeNodes(S32 N)
{
    printf("makeNodes\n");
    S32*  gamma = m_refGamma;
    AABB* refBounds = m_refBounds;

    // In parallel, make all the leaves
    cudaDeviceSynchronize(); // Needed to allocate managed with ArrayAllocator.
    LeafNode* leaves = new LeafNode[N];
    InnerNode* inner = new InnerNode[N];

    // OPT: Store refBounds directly in BVHNodes. Or replace BVHNodes with SOA.

    thrust::for_each(thrust::device, thrust::make_counting_iterator((S32)0), thrust::make_counting_iterator((S32)N),
        [=] BHD(S32 i) {
        // Fill leaf node i
        leaves[i].m_bounds = refBounds[i];
        leaves[i].m_lo = i;
        leaves[i].m_hi = i + 1;
    });

    // Object splits:
    // Split location [i] means there are i refs to the left and N - i refs to the right. (=> [0] is unused.)
    // It means a split between [i-1] and [i]. (Different than Karras 2012.)

    // gamma[i] is the offset from i to the split of the segment that either starts or ends at i.
    // If that index is a segment of length 1 then the child is in Leaves; otherwise it's in Inner.
    // gamma[i] > 0 if i's segment is to the right; <= 0 if to the left.

    thrust::for_each(thrust::device, thrust::make_counting_iterator((S32)0), thrust::make_counting_iterator((S32)N - 1),
        [=] BHD(S32 i) {
        // Fill inner node i
        bool leftIsLeaf, rightIsLeaf;
        S32 dj = gamma[i];
        S32 j = i + dj;
        if (dj <= 0) { // am a left child
            leftIsLeaf = gamma[j - 1] > 0; // true if the span starting at j-1 is a child of i's span, not an ancestor
            rightIsLeaf = dj == 0;
        }
        else { // am a right child
            leftIsLeaf = dj == 1;
            rightIsLeaf = gamma[j] <= 0;
        }

        BVHNode* left = leftIsLeaf ? (BVHNode*)(&leaves[j - 1]) : (BVHNode*)(&inner[j - 1]);
        BVHNode* right = rightIsLeaf ? (BVHNode*)(&leaves[j]) : (BVHNode*)(&inner[j]);
        left->m_parent = inner + i;
        right->m_parent = inner + i;
        inner[i].m_children[0] = left;
        inner[i].m_children[1] = right;
    });

    cudaError_t err = cudaGetLastError();
    FW_ASSERT(err == cudaSuccess);

    return inner; // inner[0] is the root node.
}

FW::BVHNode* FW::BatchSplitBVHBuilder::batchRun(BatchSplitBVHBuilder& BS, AABB& rootBounds)
{
    S32 N = BS.m_bvh.getScene()->getNumTriangles();

    S32 maxN = (S32)(BS.m_params.maxDuplication * (float)N);

    FW_ASSERT(BS.m_platform.getTriangleBatchSize() == n_triBatchSize);
    FW_ASSERT(BS.m_platform.getNodeBatchSize() == n_nodeBatchSize);
    FW_ASSERT(BS.m_platform.getSAHTriangleCost() == n_SAHTriangleCost);
    FW_ASSERT(BS.m_platform.getSAHNodeCost() == n_SAHNodeCost);

    FW::Scene* scene = BS.m_bvh.getScene();
    initBBArrays(maxN, scene, BS.m_bvh);

    // Do this in every function that uses these so they can be used in device lambdas
    S32*  refTriIdx = m_refTriIdx;
    AABB* refBounds = m_refBounds;

    const Vec3i* tris = m_tris;
    const Vec3f* verts = m_verts;

    // Determine triangle and root bounds
    rootBounds = thrust::transform_reduce(thrust::device, thrust::make_counting_iterator(0),
        thrust::make_counting_iterator(N), [refTriIdx, refBounds, tris, verts] BHD(S32 i) {
        refTriIdx[i] = i;
        refBounds[i] = AABB();
        for (int j = 0; j < 3; j++)
            refBounds[i].grow(verts[tris[i][j]]);
        return refBounds[i];
    }, AABB(), [] BHD(AABB a, AABB b) { return a + b; });

    // Initialize rest of the members
    m_minOverlap = rootBounds.area() * BS.m_params.splitAlpha;
    //printf("rootBounds: ");
    //rootBounds.print();

    // Build by generation
    S32 nSegments = -1; // Number of nodes (segments), which is the number of unique keys
    for (S32 level = 0; level < 64; level++) {
        doGeneration(N, nSegments, level); // Modifies N
        if (nSegments == N)
            break;
    }

    BVHNode* root = makeNodes(N);

    // OPT: BS.m_bvh.getTriIndices().compact(); Can't do this yet because we can't realloc managed.

    freeArrays();

    return root;
}

FW::BatchSplitBVHBuilder::BatchSplitBVHBuilder(FW::BVH& bvh, const FW::BVH::BuildParams& params)
    : m_bvh(bvh), m_platform(bvh.getPlatform()), m_params(params)
{
}

FW::BatchSplitBVHBuilder::~BatchSplitBVHBuilder(void)
{
}

FW::BVHNode* FW::BatchSplitBVHBuilder::run(void)
{
    printf("BatchSBVH alpha=%g minLeafSize=%d maxLeafSize=%d\n",
        m_params.splitAlpha, m_platform.getMinLeafSize(), m_platform.getMaxLeafSize());

    Timer progressTimer;
    progressTimer.start();

    AABB rootBounds;
    BVHNode* root = batchRun(*this, rootBounds);

    printf("BatchSplitBVHBuilder: t=%f duplicates %.0f%%\n", progressTimer.end(),
        100.0f, (F32)m_numDuplicates / (F32)m_bvh.getScene()->getNumTriangles() * 100.0f);

    // Fix everything up on CPU for now.
    cudaDeviceSynchronize();
    root->computeSubtreeValues(m_platform, rootBounds.area(), true, true);

    return root;
}
