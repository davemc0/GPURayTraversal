#if 0

float testThrust(size_t N)
{
    return float(N);
}

#endif

#if 1

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
#include <thrust/iterator/zip_iterator.h>

#include <thrust/scan.h>
#include <thrust/partition.h>
#include <thrust/sort.h>
#include <thrust/transform.h>
#include <thrust/reduce.h>
#include <thrust/transform_reduce.h>

#define BHD __host__ __device__
//#define BHD __device__

struct AABB
{
    float v[6];

    __host__ __device__ AABB() { v[0] = 5.5f; }

    bool __host__ __device__ valid() const {
        return v[0] < v[1];
    }
};

float testThrust(size_t N)
{
    int* refTriIdx;
    AABB* refBounds;
    uint64_t* keys;

    typedef thrust::tuple<int, AABB, uint64_t>    TBKTuple;
    typedef thrust::tuple<int*, AABB*, uint64_t*> TBKItTuple;
    typedef thrust::zip_iterator<TBKItTuple> TBKZipIt;
    TBKZipIt refsTBK(thrust::make_tuple(refTriIdx, refBounds, keys));
    TBKZipIt refsTBKend(thrust::make_tuple(refTriIdx + N, refBounds + N, keys + N));

    cudaDeviceSynchronize(); // XXX
    auto mid = thrust::stable_partition(thrust::device, refsTBK, refsTBKend, [] BHD(const TBKTuple r) {
        return thrust::get<1>(r).valid();
    });

    return 0;
}

#if 0
int main()
{
    size_t N = 1024;

    testThrust(N);

    return 0;
}

#endif

#endif

#if 0
// Want to use a lambda instead of this, but it doesn't work. :(
struct getAABBF : public thrust::unary_function<const Reference&, const AABB&>
{
    __device__ const AABB& operator()(const Reference& x) const { return x.bounds; }
};

float testThrust(size_t N)
{
#if 0
    thrust::reverse_iterator<Reference*> revIt(refs + N);
    auto getAABB = [] BHD(const Reference& r) { return r.bounds; };
    thrust::inclusive_scan(thrust::device,
        thrust::make_transform_iterator(revIt, getAABB),
        thrust::make_transform_iterator(revIt + N, getAABB),
        rightBounds,
        [] __device__(AABB a, AABB b) { return a + b; });
#else
    //auto getAABB = [] BHD(const Reference& r) { return r.bounds; };
    thrust::inclusive_scan(thrust::device,
        thrust::make_transform_iterator(refs, getAABBF()),
        thrust::make_transform_iterator(refs + N, getAABBF()),
        rightBounds,
        [] BHD(AABB a, AABB b) { return a + b; });
#endif
}
#endif

#if 0
#include <thrust/reduce.h>
#include <thrust/iterator/counting_iterator.h>

int main()
{
    int N = 1024;
    thrust::counting_iterator<int> ci(0);
    int result = thrust::reduce(thrust::host, ci, ci + N, (int)0, [](int a, int b) { return a + b; });
    return 0;
}

#endif

#if 0
#include <thrust/reduce.h>
#include <thrust/iterator/counting_iterator.h>

int main()
{
    // create iterators
    thrust::counting_iterator<int> first(10);
    thrust::counting_iterator<int> last = first + 3;

    // first[0]   // returns 10
    // first[1]   // returns 11
    // first[100] // returns 110

    // sum of [first, last)
    thrust::reduce(first, last);   // returns 33 (i.e. 10 + 11 + 12)

    // initialize vector to [0,1,2,..]
    thrust::counting_iterator<int> iter(0);
    thrust::device_vector<int> vec(500);
    thrust::copy(iter, iter + vec.size(), vec.begin());
    int res = thrust::reduce(vec.begin(), vec.end());

    printf("%d\n", res);
    return 0;
}

#endif
#if 0
#include <thrust/iterator/counting_iterator.h>
#include <thrust/copy.h>
#include <thrust/functional.h>
#include <thrust/device_vector.h>

int main(void)
{
    // this example computes indices for all the nonzero values in a sequence

    // sequence of zero and nonzero values
    thrust::device_vector<int> stencil(8);
    stencil[0] = 0;
    stencil[1] = 1;
    stencil[2] = 1;
    stencil[3] = 0;
    stencil[4] = 0;
    stencil[5] = 1;
    stencil[6] = 0;
    stencil[7] = 1;

    // storage for the nonzero indices
    thrust::device_vector<int> indices(8);

    // compute indices of nonzero elements
    typedef thrust::device_vector<int>::iterator IndexIterator;

    // use make_counting_iterator to define the sequence [0, 8)
    IndexIterator indices_end = thrust::copy_if(thrust::make_counting_iterator(0),
        thrust::make_counting_iterator(8),
        stencil.begin(),
        indices.begin(),
        thrust::identity<int>());
    // indices now contains [1,2,5,7]

    return 0;
}
#endif

#if 0
#include <thrust/reduce.h>
#include <thrust/iterator/counting_iterator.h>

float testThrust(size_t N)
{
    N = 1024;
    printf("Here\n");
    thrust::counting_iterator<int> ci(0);
    size_t result = thrust::reduce(thrust::host, ci, ci + N, (size_t)0);
    printf("%lld Out\n", result);
    return float(N);
}

#endif

#if 0
#include "Util.hpp"

#include <thrust/sort.h>
#include <thrust/swap.h>
#include <thrust/transform.h>
#include <thrust/sequence.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/device_vector.h>

struct AABB
{
    float dims[6];

    __host__ __device__ AABB() { dims[0] = 5.5f; }
};

struct Reference
{
    int  triIdx;
    AABB bounds;
};

void __host__ __device__ swap(Reference& a, Reference& b)
{
    printf("swap 0x%016llx 0x%016llx\n", &a, &b);
    thrust::swap(a, b);
}

float testThrust(size_t N)
{
    N = 80000;
    printf("Here\n");
    thrust::device_vector<Reference> vec(N);

    thrust::transform(
        thrust::device,
        thrust::make_counting_iterator((int)0),
        thrust::make_counting_iterator((int)N),
        vec.begin(),
        [] __device__ (int i) {
            Reference r;
            r.bounds.dims[0] = 1000-i;

            return r;
        });

    printf("span 0x%016llx 0x%016llx\n", (unsigned long long)vec.data().get(), (unsigned long long)vec.data().get() + N);

    thrust::sort(vec.begin(), vec.end(), [] __device__ (const Reference& ra, const Reference& rb) {
        //printf("cmpr 0x%016llx 0x%016llx %f %f\n", &ra, &rb, ra.bounds.dims[0], rb.bounds.dims[0]);
        return ra.bounds.dims[0] < rb.bounds.dims[0];
    });

    size_t result = N;
    printf("%lld Out\n", result);
    return float(N);
}

#endif

#if 0
// Works in Debug, fails in Release. Spews lots of warnings.
#define THRUST_DEBUG_SYNC
#define DEBUG

#include "base/Math.hpp"
#include "Util.hpp"

#include <thrust/transform_reduce.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/reduce.h>
#include <thrust/iterator/constant_iterator.h>

#define HD __host__ __device__

float testThrust(size_t N)
{
    FW::AABB result = thrust::transform_reduce(thrust::device,
        thrust::make_counting_iterator(0), thrust::make_counting_iterator((int)N), [] HD(int i) {
        FW::AABB x;
        return x;
    }, FW::AABB(), [] HD(FW::AABB a, FW::AABB b) { return a; });
    cudaDeviceSynchronize();

    printf("Out\n");

    return float(N);
}

#if 0
int main()
{
    size_t N = 1024;

    testThrust(N);

    return 0;
}

#endif

#endif

#if 0
// Works in Debug, fails in Release. Spews lots of warnings.
#define THRUST_DEBUG_SYNC
#define DEBUG

#include <thrust/reduce.h>
#include <thrust/iterator/constant_iterator.h>

class My
{
public:
    My() { m_val = 8; }

    void doWork(size_t N)
    {
        m_val = 2;
        printf("Here\n");
        int result = thrust::reduce(thrust::device,
            thrust::make_counting_iterator((int)0), thrust::make_counting_iterator((int)N),
            0, [m_val = m_val] __device__ (int a, int b) { return a * m_val + b * m_val; });
    }

    int m_val;
    thrust::host_vector<int> m_jill;
};

float testThrust(size_t N)
{
    My Bob;
    Bob.doWork(N);

    printf("Out\n");

    return float(N);
}

#if 0
int main()
{
    size_t N = 1024;

    testThrust(N);

    return 0;
}

#endif

#endif

#if 0

#define THRUST_DEBUG_SYNC
#define DEBUG

#include "base/Math.hpp"
#include "Util.hpp"

#include <thrust/transform_reduce.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/reduce.h>
#include <thrust/iterator/constant_iterator.h>

#define HD __host__ __device__

namespace {

    struct Reference
    {
        FW::S32                 triIdx;
        FW::AABB                bounds;

        Reference(void) : triIdx(-1) {}
    };

    void duh(size_t N)
    {
        FW::Vec3i* tris;
        FW::Vec3f* verts;
        Reference* refStack;
        cuMemAllocManaged((CUdeviceptr*)&tris, sizeof(FW::Vec3i) * N, ::CU_MEM_ATTACH_GLOBAL);
        cuMemAllocManaged((CUdeviceptr*)&verts, sizeof(FW::Vec3f) * N, ::CU_MEM_ATTACH_GLOBAL);
        cuMemAllocManaged((CUdeviceptr*)&refStack, sizeof(Reference) * N, ::CU_MEM_ATTACH_GLOBAL);

        FW::AABB result;
        printf("0x%016llx 0x%016llx 0x%016llx 0x%016llx\n", (FW::U64)tris, (FW::U64)verts, (FW::U64)refStack, (FW::U64)&result);

        thrust::counting_iterator<int> ci(0);
        result = thrust::transform_reduce(thrust::device, ci, ci + N, [refStack, verts, tris] HD(FW::S32 i) {
            refStack[i].triIdx = i;
            refStack[i].bounds = FW::AABB();
            for (int j = 0; j < 3; j++)
                refStack[i].bounds.grow(verts[tris[i][j]]);
            if (i == 1) refStack[i].bounds.min().print();
            return refStack[i].bounds;
        }, FW::AABB(), [] HD(FW::AABB a, FW::AABB b) { return a + b; });
        cudaDeviceSynchronize();

        printf("Out\n");
        //rootSpec.bounds.min().print();
        //rootSpec.bounds.max().print();

    }
};

float testThrust(size_t N)
{
    duh(N);

    return float(N);
}

#endif
