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
        return thrust::make_tuple(childSAH, leftN, rightN);
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

    m_rightIdx = m_intArray.getPtr();
    m_leftIdx = m_rightIdx + maxN;
    m_gamma = m_leftIdx + maxN;
    m_segIdx = m_gamma + maxN;
    m_outIdx[0] = m_segIdx + maxN;
    m_outIdx[1] = m_outIdx[0] + maxN;
    m_outIdx[2] = m_outIdx[1] + maxN;
    m_outCost[0] = (F32*)(m_outIdx[2] + maxN);
    m_outCost[1] = m_outCost[0] + maxN;
    m_outCost[2] = m_outCost[1] + maxN;
    m_refBounds = m_boundsArray.getPtr();
    m_rightBounds = m_refBounds + maxN;
    m_leftBounds = m_rightBounds + maxN;
    m_keys = m_keysArray.getPtr();
    m_outKeys = m_keys + maxN;

    bvh.getTriIndices().setManaged(true);
    bvh.getTriIndices().resize(maxN);
    m_refTriIdx = bvh.getTriIndices().getPtr();

    m_tris = (const Vec3i*)scene->getTriVtxIndexBuffer().getCudaPtr();
    m_verts = (const Vec3f*)scene->getVtxPosBuffer().getCudaPtr();

    cuMemsetD32((CUdeviceptr)m_gamma, FW_S32_MIN, maxN); // Set gamma to FW_S32_MIN
}

void FW::BatchSplitBVHBuilder::freeArrays()
{
    printf("freeBBArrays()\n");
    m_intArray.reset(0);
    m_boundsArray.reset(0);
    m_keysArray.reset(0);
}

void FW::BatchSplitBVHBuilder::doGeneration(S32& N, S32& nSegments, S32 level)
{
    S32*  refTriIdx = m_refTriIdx;
    S32*  rightIdx = m_rightIdx;
    S32*  leftIdx = m_leftIdx;
    S32*  gamma = m_gamma;
    S32*  segIdx = m_segIdx;
    S32*  outIdx[3]; outIdx[0] = m_outIdx[0]; outIdx[1] = m_outIdx[1]; outIdx[2] = m_outIdx[2];
    F32*  outCost[3]; outCost[0] = m_outCost[0]; outCost[1] = m_outCost[1]; outCost[2] = m_outCost[2];
    AABB* refBounds = m_refBounds;
    AABB* rightBounds = m_rightBounds;
    AABB* leftBounds = m_leftBounds;
    U64*  keys = m_keys;
    U64*  outKeys = m_outKeys;

    typedef thrust::tuple<S32, AABB, U64>    TBKTuple;
    typedef thrust::tuple<S32*, AABB*, U64*> TBKItTuple;
    typedef thrust::zip_iterator<TBKItTuple> TBKZipIt;
    TBKZipIt refsTBK(thrust::make_tuple(refTriIdx, refBounds, keys));

    auto OneIt = thrust::make_constant_iterator((S32)1);

    // Remove degenerates.
    // OPT: For Sweep builder move this out of the loop. If so, for speed, change it to not be a stable_partition. Split builder makes new refs.
    auto mid = thrust::stable_partition(thrust::device, refsTBK, refsTBK + N, [] BHD(const TBKTuple r) {
        Vec3f size = thrust::get<1>(r).max() - thrust::get<1>(r).min();
        return !(min(size) < 0.0f || sum(size) == max(size));
    });

    S32 newN = thrust::get<0>(mid.get_iterator_tuple()) - refTriIdx;
    if (newN != N) printf("%d => %d\n", N, newN);
    N = newN;

    // Try object split in each dimension
    for (int dim = 0; dim < 3; dim++) {

        // Sort in given dimension
        thrust::sort(thrust::device,
            refsTBK, refsTBK + N,
            [dim] BHD(TBKTuple a, TBKTuple b) {
            F32 ca = thrust::get<1>(a).min()[dim] + thrust::get<1>(a).max()[dim];
            F32 cb = thrust::get<1>(b).min()[dim] + thrust::get<1>(b).max()[dim];
            U64 ka = thrust::get<2>(a);
            U64 kb = thrust::get<2>(b);
            return (ka < kb) || (ka == kb && (ca < cb || (ca == cb && thrust::get<0>(a) < thrust::get<0>(b))));
        });

        // Sweep right to left and determine bounds; rightIdx is offset from right edge of segment
        // leftBounds[i] and rightBounds[i] contain the two AABBs for splitting at i.

        typedef thrust::tuple<AABB, S32> BITuple;
        typedef thrust::tuple<AABB*, S32*> BIItTuple;
        typedef thrust::zip_iterator<BIItTuple> BIZipIt;

        auto BIRevIt(thrust::make_zip_iterator(thrust::make_tuple(thrust::make_reverse_iterator(refBounds + N), OneIt)));
        auto OutBIRevIt(thrust::make_zip_iterator(thrust::make_tuple(thrust::make_reverse_iterator(rightBounds + N), thrust::make_reverse_iterator(rightIdx + N))));

        thrust::inclusive_scan_by_key(thrust::device,
            thrust::make_reverse_iterator(keys + N),
            thrust::make_reverse_iterator(keys),
            BIRevIt, OutBIRevIt,
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(BITuple a, BITuple b) { return thrust::make_tuple(thrust::get<0>(a) + thrust::get<0>(b), thrust::get<1>(a) + thrust::get<1>(b)); });

        // Sweep left to right and determine bounds; leftIdx is offset from left edge of segment
        BIZipIt OutBIIt(thrust::make_tuple(leftBounds, leftIdx));

        // OPT: Don't need to write leftIdx and rightIdx all three times.
        thrust::exclusive_scan_by_key(thrust::device,
            keys, keys + N,
            thrust::make_zip_iterator(thrust::make_tuple(refBounds, OneIt)),
            OutBIIt,
            thrust::make_tuple(AABB(), (S32)0),
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(BITuple a, BITuple b) { return thrust::make_tuple(thrust::get<0>(a) + thrust::get<0>(b), thrust::get<1>(a) + thrust::get<1>(b)); });

        // OPT: Store a segment's full AABB into its final BVHNode, since we know its location now

        // Select lowest SAH.
        BIBIZipIt bounds(thrust::make_tuple(rightBounds, rightIdx, leftBounds, leftIdx));
        typedef thrust::discard_iterator<S32> IDisIt;
        IDisIt Dis;

        // OPT: Only need to write the keys out once. Could use discard_iterator on the other two dimensions.
        // OPT: segIdx is unneeded; should use a discard_iterator, but was getting errors.
        auto outEnd = thrust::reduce_by_key(thrust::device,
            keys, keys + N, thrust::make_transform_iterator(bounds, BoundsToCost()),
            outKeys, thrust::make_zip_iterator(thrust::make_tuple(outCost[dim], outIdx[dim], segIdx)),
            [] BHD(U64 ka, U64 kb) { return ka == kb; },
            [] BHD(FIITuple a, FIITuple b) { return thrust::get<0>(a) < thrust::get<0>(b) ? a :
                (thrust::get<0>(a) > thrust::get<0>(b) ? b : (abs(thrust::get<1>(a) - thrust::get<2>(a)) <
                                                              abs(thrust::get<1>(b) - thrust::get<2>(b)) ? a : b)); });

        // OPT: Can merge out* here so we don't need 3 copies and so we don't have to choose the best strategy for the segment repeatedly
        nSegments = outEnd.first - outKeys;
        printf("dim=%d nSegments=%d keys=%016llx\n", dim, nSegments, outKeys[0]);
    }

    // Make segIdx be the index into outKeys, outCost, outIdx
    thrust::transform_inclusive_scan(thrust::device,
        thrust::make_counting_iterator((S32)0), thrust::make_counting_iterator((S32)N), segIdx,
        [keys] BHD(S32 i) { return (i == 0 || keys[i] == keys[i - 1]) ? 0 : 1; },
        [] BHD(S32 a, S32 b) { return a + b; });

    // Sort each segment by its best dimension
    typedef thrust::tuple<S32, AABB, S32>    TBITuple;
    typedef thrust::tuple<S32*, AABB*, S32*> TBIItTuple;
    typedef thrust::zip_iterator<TBIItTuple> TBIZipIt;
    TBIZipIt refsTBI(thrust::make_tuple(refTriIdx, refBounds, segIdx));

    F32* outCost0 = outCost[0];
    F32* outCost1 = outCost[1];
    F32* outCost2 = outCost[2];

    thrust::sort(thrust::device,
        refsTBI, refsTBI + N, [outCost0, outCost1, outCost2] BHD(TBITuple a, TBITuple b) {
        // Which dim to sort this segment by?
        S32 sa = thrust::get<2>(a); // Segment index in output arrays
        S32 sb = thrust::get<2>(b);
        int dim = 0;
        F32 bestCost = min(outCost0[sa], outCost1[sa], outCost2[sa]);
        if (bestCost == outCost1[sa]) dim = 1;
        if (bestCost == outCost2[sa]) dim = 2;

        F32 ca = thrust::get<1>(a).min()[dim] + thrust::get<1>(a).max()[dim];
        F32 cb = thrust::get<1>(b).min()[dim] + thrust::get<1>(b).max()[dim];
        return (sa < sb) || (sa == sb && (ca < cb || (ca == cb && thrust::get<0>(a) < thrust::get<0>(b))));
    });

    // Update keys to partition each segment at the best location
    S32* outIdx0 = outIdx[0];
    S32* outIdx1 = outIdx[1];
    S32* outIdx2 = outIdx[2];

    thrust::for_each(thrust::device, thrust::make_counting_iterator((S32)0), thrust::make_counting_iterator((S32)N),
        [=] BHD(S32 i) {
        // Which dim did it sort this segment by?
        S32 s = segIdx[i]; // Segment index in output arrays
        S32* dimArr = outIdx0;
        F32 bestCost = min(outCost0[s], outCost1[s], outCost2[s]);
        if (bestCost == outCost1[s]) dimArr = outIdx1;
        if (bestCost == outCost2[s]) dimArr = outIdx2;

        if (leftIdx[i] >= dimArr[s])
            // My offset within segment is to the right of the split index
            keys[i] = keys[i] | 1ull << (U64)(63 - level);

        // If I'm at the start or end of the range and the slot hasn't been claimed yet I record the relative split location.
        if (leftIdx[i] == 0 && gamma[i] == FW_S32_MIN)
            gamma[i] = dimArr[s];
        if (rightIdx[i] == 1 && gamma[i] == FW_S32_MIN)
            gamma[i] = dimArr[s] - leftIdx[i]; // The (negative) offset from i to the relative split location.
    });
}

FW::BVHNode* FW::BatchSplitBVHBuilder::makeNodes(S32 N)
{
    S32*  gamma = m_gamma;
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
    // It means a split between [i-1] and [i]. (Different than Karras 2012.)a

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
    printf("rootBounds: ");
    rootBounds.print();

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
