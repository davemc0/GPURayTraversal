#include "testCPP.h"

#include <vector>
#include <execution>
#include <algorithm>
#include <iostream>

using namespace std::execution;

// Note: I turned on exception handling for this, but without the STL code I could disable it.

void testHostSort(size_t N)
{
    std::vector<float> a(N);
    std::generate(a.begin(), a.end(), []() {return (float)rand(); }); // 2.22 sec at N=2^27

    std::sort(par, a.begin(), a.end()); // serial is 10.7 seconds including generate
    
    printf("%f\n", a[0]);
}
