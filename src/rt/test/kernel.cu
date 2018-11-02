#include <thrust/host_vector.h>
#include <thrust/sort.h>

struct multip
{
    __host__ __device__
        int operator()(int x) { return x * 10; }
};

void testThrust()
{
    thrust::host_vector<int> a(1 << 10);
    thrust::device_vector<int> b(1 << 10);
    thrust::device_vector<int> c(1 << 10);

    thrust::generate(a.begin(), a.end(), rand);
    b = a;

    //thrust::transform(a.begin(), a.end(), b.begin(), );
    //thrust::transform(b.begin(), b.end(), c.begin(), multip());
    thrust::transform(b.begin(), b.end(), c.begin(), [] __host__ __device__(int v) { return 100 * v; });

    // Copy to host
    thrust::sort(c.begin(), c.end());

    a = c;

    for each (auto var in a) {
        std::cout << var << ' ';
    }
    std::cout << '\n';
}
