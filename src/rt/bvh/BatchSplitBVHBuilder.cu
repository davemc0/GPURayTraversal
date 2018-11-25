#define FW_ENABLE_ASSERT

#define THRUST_DEBUG_SYNC
#define DEBUG

#include "bvh/BVHNode.hpp"
#include "bvh/BatchSplitBVHBuilder.hpp"
#include "base/Array.hpp"
#include "base/Timer.hpp"

#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/constant_iterator.h>

#include "thrust/partition.h"
#include <thrust/sort.h>
#include <thrust/transform.h>
#include <thrust/reduce.h>
#include <thrust/transform_reduce.h>

//#define BHD __host__ __device__
#define BHD __device__

namespace BB
{
    // Use a namespace, rather than a class, to encapsulate names but still allow easy device-side access,
    // especially for lambdas. This means we can only make one at a time, if we use globals.

    using namespace FW;

    struct Reference
    {
        FW::AABB bounds;
        S32 triIdx;
        U32 aux;
        U64 key;
    };

    // Data structures owned by namespace
     Array<Reference> n_refArray;
     Array<AABB>      n_rightBoundsArray;
     Array<S64>       n_keyArray;
     Array<S32>       n_auxArray;
     F32              n_minOverlap;
     S32              n_numDuplicates;

    // Raw pointers for GPU arrays; initialized in run(); all allocated using cudaMallocManaged
    const  Vec3i* n_tris;
    const  Vec3f* n_verts;
    Reference*    n_refs;
    AABB*         n_rightBounds;
    S64*          n_keys;
    S32*          n_aux;

    // Want to use a lambda instead of this, but it doesn't work. :(
    struct getAABBF : public thrust::unary_function<const Reference&, const AABB&>
    {
        __device__ const AABB& operator()(const Reference& x) const { return x.bounds; }
    };

    void doGeneration(S32& N)
    {
        /*
        Make references from tris => refs
        Find split planes
            for each dim
                segmented (by m_keys) sort by centroid in dim => refs
                segmented (by m_keys) reverse inclusive scan of bounds => m_rightBounds
                segmented (by m_keys) inclusive scan of bounds => m_leftBounds
                segmented (by m_keys) inclusive scan of SAH lambda => m_keys
                    (
                store split record for every node -

                Have an array of nodes. Use the scan to give each ref its index into the node array.
                reduce_by_key makes list with one entry per node / span.

        Perform splits / Make leaves
        Do one prim per leaf

        */

        //const Vec3i* tris = n_tris;
        //const Vec3f* verts = n_verts;
        Reference*   refs = n_refs;
        AABB*        rightBounds = n_rightBounds;

        // Remove degenerates.
        // XXX Can probably move this out of the loop. If so, for speed, change it to not be a stable_partition.
        auto mid = thrust::stable_partition(thrust::device, refs, refs + N, [] BHD (Reference r) {
            Vec3f size = r.bounds.max() - r.bounds.min();
            return !(min(size) < 0.0f || sum(size) == max(size));
        });

        S32 newN = mid - refs;
        if (newN != N) printf("%d => %d\n", N, newN);
        N = newN;

        // Try object split in each dimension
        for (int dim = 0; dim < 3; dim++) {
            thrust::sort(thrust::device, refs, refs + N, [dim] BHD(const Reference ra, const Reference rb) {
                F32 ca = ra.bounds.min()[dim] + ra.bounds.max()[dim];
                F32 cb = rb.bounds.min()[dim] + rb.bounds.max()[dim];
                return (ra.key < rb.key) || (ra.key == rb.key && (ca < cb || (ca == cb && ra.triIdx < rb.triIdx)));
            });

            cudaDeviceSynchronize(); // XXX
            for (int i = N / 2; i < N; i++)
                refs[i].key = 1;
            printf("%d %d\n", N, N / 2);

            // Sweep right to left and determine bounds.
            // XXX Change to by key.
            thrust::reverse_iterator<Reference*> revIt(refs + N);
            thrust::inclusive_scan_by_key(thrust::device,
                thrust::make_transform_iterator(revIt, getAABBF()),
                thrust::make_transform_iterator(revIt + N, getAABBF()),
                rightBounds,
                [] BHD(AABB a, AABB b) { return a + b; });

            // Sweep left to right and select lowest SAH.
            // PAR: Is an inclusive scan
        }
    }

    BVHNode* makeLeaves()
    {
        return nullptr;
    }

    BVHNode* batchRun(BatchSplitBVHBuilder& BS)
    {
        S32 N = BS.m_bvh.getScene()->getNumTriangles();
        S32 maxN = (S32)(BS.m_params.maxDuplication * (float)N);

        n_refArray.setManaged(true);
        n_refArray.resize(maxN);
        n_keyArray.setManaged(true);
        n_keyArray.resize(maxN);
        n_auxArray.setManaged(true);
        n_auxArray.resize(maxN);
        n_rightBoundsArray.setManaged(true);
        n_rightBoundsArray.resize(maxN);

        n_tris = (const Vec3i*)BS.m_bvh.getScene()->getTriVtxIndexBuffer().getCudaPtr();
        n_verts = (const Vec3f*)BS.m_bvh.getScene()->getVtxPosBuffer().getCudaPtr();
        n_refs = n_refArray.getPtr();
        n_rightBounds = n_rightBoundsArray.getPtr();
        n_keys = n_keyArray.getPtr();
        n_aux  = n_auxArray.getPtr();

        // Do this in every function that uses a device lambda:
        const Vec3i* tris  = n_tris;
        const Vec3f* verts = n_verts;
        Reference*   refs  = n_refs;
        S64*         keys  = n_keys;
        S32*         aux   = n_aux;

        // Initialize reference array and determine root bounds.

        AABB rootBounds = thrust::transform_reduce(thrust::device, thrust::make_counting_iterator(0),
            thrust::make_counting_iterator(N), [refs, tris, verts] BHD(S32 i) {
            refs[i].triIdx = i;
            refs[i].key = 0;
            refs[i].aux = 0;
            refs[i].bounds = AABB();
            for (int j = 0; j < 3; j++)
                refs[i].bounds.grow(verts[tris[i][j]]);
            return refs[i].bounds;
        }, AABB(), [] BHD (AABB a, AABB b) { return a + b; });

        cudaMemset(keys, 0, sizeof(S64) * N);
        cudaMemset(aux,  0, sizeof(S32) * N);
        cudaDeviceSynchronize();

        // Initialize rest of the members.

        n_minOverlap = rootBounds.area() * BS.m_params.splitAlpha;

        // Build by generation

        while (1) {
            doGeneration(N); // Modifies N

            // Adjust N
            break;
        }

        BVHNode* root = makeLeaves();
        BS.m_bvh.getTriIndices().compact();

        return root;
    }

};

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

    BVHNode* root = BB::batchRun(*this);

    printf("BatchSplitBVHBuilder: t=%f duplicates %.0f%%\n", progressTimer.end(),
        100.0f, (F32)BB::n_numDuplicates / (F32)m_bvh.getScene()->getNumTriangles() * 100.0f);

    exit(0); // XXX
    return root;
}
