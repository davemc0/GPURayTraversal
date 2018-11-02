#include "bvh/LBVHBuilder.hpp"

#include <thrust/version.h>
#include <thrust/host_vector.h>
#include <iostream>

void testThrust(void)
{
    int major = THRUST_MAJOR_VERSION;
    int minor = THRUST_MINOR_VERSION;

    std::cout << "Thrust v" << major << "." << minor << std::endl;

    const size_t N = 100000000;

    thrust::host_vector<int> tvec;
    tvec.resize(N);
}
