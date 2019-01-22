/*
 *  Copyright 2008-2016 NVIDIA Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "conditional_output_iterator.h"
#include <thrust/iterator/iterator_adaptor.h>

namespace thrust
{

template <typename OutputIterator, typename BinaryPredicate>
  class conditional_output_iterator;

namespace detail 
{

// Proxy reference that uses Unary Functiont o conditional the rhs of assigment
// operator before writing the result to OutputIterator
template <typename BinaryPredicate, typename OutputIterator>
  class conditional_output_iterator_proxy
{
  public:
    __host__ __device__
    conditional_output_iterator_proxy(const OutputIterator& out, BinaryPredicate fun) : out(out), fun(fun)
    {
    }

    template <typename T>
    __host__ __device__
    conditional_output_iterator_proxy operator=(const T& x)
    {
      if(fun(x, *out))
          *out = x;
      return *this;
    }

  private:
    OutputIterator out;
    BinaryPredicate fun;
};

// Compute the iterator_adaptor instantiation to be used for conditional_output_iterator
template <typename BinaryPredicate, typename OutputIterator>
struct conditional_output_iterator_base
{
    typedef thrust::iterator_adaptor
    <
        conditional_output_iterator<BinaryPredicate, OutputIterator>
      , OutputIterator
      , thrust::use_default
      , thrust::use_default
      , thrust::use_default
      , conditional_output_iterator_proxy<BinaryPredicate, OutputIterator>
    > type;
};

// Register conditional_output_iterator_proxy with 'is_proxy_reference' from
// type_traits to enable its use with algorithms.
template <class OutputIterator, class BinaryPredicate>
struct is_proxy_reference<
    conditional_output_iterator_proxy<OutputIterator, BinaryPredicate> >
    : public thrust::detail::true_type {};

} // end detail
} // end thrust

