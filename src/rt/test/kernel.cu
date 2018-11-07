#include <thrust/host_vector.h>
#include <thrust/sort.h>

struct multip
{
    __host__ __device__
        int operator()(int x) { return x * 10; }
};

void hostSort()
{
    int N = 1 << 24;

    thrust::host_vector<float> a(N);
    
    //thrust::generate(a.begin(), a.end(), []() { return (float)rand(); });
    thrust::generate(a.begin(), a.end(), rand);

    thrust::sort(a.begin(), a.end());

    //for each (auto var in a) {
    //    std::cout << var << ' ';
    //}
    //std::cout << '\n';
}

float testReduce()
{
    int N = 1 << 29;

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

float testThrust()
{
    hostSort();

    return 123.f;
}
