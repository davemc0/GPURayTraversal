//#include "base/DeviceDefs.hpp"
//#include "base/Math.hpp"
#include "base/ManagedAllocator.hpp"

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

#include <algorithm>

#if 0

struct multip
{
    __host__ __device__
        int operator()(int x) { return x * 10; }
};

void hostSort(size_t N)
{
    thrust::host_vector<float> a(N);
    
    //thrust::generate(a.begin(), a.end(), []() { return (float)rand(); });
    thrust::generate(a.begin(), a.end(), rand);

    thrust::sort(a.begin(), a.end());

    //for each (auto var in a) {
    //    std::cout << var << ' ';
    //}
    //std::cout << '\n';
}

float testReduce(size_t N)
{
    thrust::host_vector<float> a(N);
    thrust::device_vector<float> b(N);
    thrust::device_vector<float> c(N);

    //thrust::generate(a.begin(), a.end(), []() { return (float)rand(); });
    //thrust::generate(b.begin(), b.end(), rand);

    // Copy to device
    // b = a;

    //thrust::transform(a.begin(), a.end(), b.begin(), );
    //thrust::transform(b.begin(), b.end(), c.begin(), multip());
    thrust::transform(b.begin(), b.end(), c.begin(), [] __host__ __device__(float v) { return 10.f * v; });

    thrust::sort(c.begin(), c.end());

    // Copy to host
    //a = c;

    float res = thrust::reduce(c.begin(), c.end());

    // for each (auto var in a) {
    //     std::cout << var << ' ';
    // }
    // std::cout << '\n';

    return res;
}

float testUVM(size_t N)
{
    printf("Testing UVM\n");

    thrust::host_vector<float> a(N);
    thrust::device_vector<float> b(N);
    thrust::device_vector<float> c(N);
    thrust::host_vector<float> d(N);

    thrust::generate(a.begin(), a.end(), rand);

    float* aptr;
    cudaError_t er = cudaMallocManaged(&aptr, N * sizeof(float));// , cudaStreamDefault);

    assert(er == cudaSuccess);
    printf("aptr=0x%08llx er=%d\n", (unsigned long long)aptr, er);

    for (int i = 0; i < N; i++)
        aptr[i] = 3.14159f; // (float)rand();

    thrust::transform(b.begin(), b.end(), c.begin(), [=]  __device__(float v) { return *aptr; });

    d = c;

    for(int i=0; i<16; i++)
        std::cout << d[i] << ' ';
    std::cout << '\n';

    return 0.f;
}


// Kernel definition
__global__ void VecAdd(float* A, float* B, float* C)
{
    int i = threadIdx.x;

    C[i] = A[i] + B[i];
}

void testKernel(size_t N)
{
    thrust::device_vector<float> A(N);
    thrust::device_vector<float> B(N);
    thrust::device_vector<float> C(N);

    int numBlocks = 1;
    dim3 threadsPerBlock(32);
    VecAdd<<<numBlocks, threadsPerBlock >>>(A.data().get(), (float*)B.data().get(), (float*)C.data().get());
}

// create a nickname for vectors which use a managed_allocator
template<class T>
using managed_vector = std::vector<T, managed_allocator<T>>;

void testManaged(size_t N)
{
    managed_vector<int> vec(N);

    // we can use the vector from the host
    std::iota(vec.begin(), vec.end(), 0);

    std::vector<int> ref(N);
    std::iota(ref.begin(), ref.end(), 0);
    assert(std::equal(ref.begin(), ref.end(), vec.begin()));

    // we can also use it in a CUDA kernel
    size_t block_size = 256;
    size_t num_blocks = (N + (block_size - 1)) / block_size;

    increment_kernel << <num_blocks, block_size >> > (vec.data(), vec.size());

    cudaDeviceSynchronize();

    std::for_each(ref.begin(), ref.end(), [](int& x)
    {
        x += 1;
    });

    assert(std::equal(ref.begin(), ref.end(), vec.begin()));

    // we can also use it with Thrust algorithms

    // by default, the Thrust algorithm will execute on the host with the managed_vector
    thrust::fill(vec.begin(), vec.end(), 7);
    assert(std::all_of(vec.begin(), vec.end(), [](int x)
    {
        return x == 7;
    }));

    // to execute on the device, use the thrust::device execution policy
    thrust::fill(thrust::device, vec.begin(), vec.end(), 13);

    // we need to synchronize before attempting to use the vector on the host
    cudaDeviceSynchronize();

    // to execute on the host, use the thrust::host execution policy
    assert(thrust::all_of(thrust::host, vec.begin(), vec.end(), [](int x)
    {
        return x == 13;
    }));

    std::cout << "OK" << std::endl;

    return 0;
}

#endif

float testThrust(size_t N)
{

    //testKernel(N);
    // testUVM();
    //hostSort(N);
    //testManaged(N);

    return float(N);
}
