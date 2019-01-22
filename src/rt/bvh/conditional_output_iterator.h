/*! \file thrust/iterator/conditional_output_iterator.h
 *  \brief An output iterator which adapts another output iterator by applying a
 *         function to the result of its dereference before writing it.
 */

#pragma once

#include <thrust/detail/config.h>
#include "conditional_output_iterator.inl"

namespace thrust
{

/*! \addtogroup iterators
 *  \{
 */

/*! \addtogroup fancyiterator Fancy Iterators
 *  \ingroup iterators
 *  \{
 */

/*! \p conditional_output_iterator is a special kind of output iterator which
 * conditionals a value written upon dereference. This iterator is useful
 * for conditionaling an output from algorithms without explicitly storing the
 * intermediate result in the memory and applying subsequent conditionalation, 
 * thereby avoiding wasting memory capacity and bandwidth.
 * Using \p conditional_iterator facilitates kernel fusion by deferring execution
 * of conditionalation until the value is written while saving both memory
 * capacity and bandwidth.
 *
 * The following code snippet demonstrated how to create a
 * \p conditional_output_iterator which applies \c sqrtf to the assigning value.
 *
 * \code
 * #include <thrust/iterator/conditional_output_iterator.h>
 * #include <thrust/device_vector.h>
 *
 * // note: functor inherits form unary function
 *  // note: functor inherits from unary_function
 *  struct square_root : public thrust::unary_function<float,float>
 *  {
 *    __host__ __device__
 *    float operator()(float x) const
 *    {
 *      return sqrtf(x);
 *    }
 *  };
 *  
 *  int main(void)
 *  {
 *    thrust::device_vector<float> v(4);
 *
 *    typedef thrust::device_vector<float>::iterator FloatIterator;
 *    thrust::conditional_output_iterator<square_root, FloatIterator> iter(v.begin(), square_root());
 *
 *    iter[0] =  1.0f;    // stores sqrtf( 1.0f) 
 *    iter[1] =  4.0f;    // stores sqrtf( 4.0f)
 *    iter[2] =  9.0f;    // stores sqrtf( 9.0f)
 *    iter[3] = 16.0f;    // stores sqrtf(16.0f)
 *    // iter[4] is an out-of-bounds error
 *                                                                                           
 *    v[0]; // returns 1.0f;
 *    v[1]; // returns 2.0f;
 *    v[2]; // returns 3.0f;
 *    v[3]; // returns 4.0f;
 *                                                                                           
 *  }
 *  \endcode
 *
 *  \see make_conditional_output_iterator
 */

template <typename BinaryPredicate, typename OutputIterator>
  class conditional_output_iterator
    : public detail::conditional_output_iterator_base<BinaryPredicate, OutputIterator>::type
{

  /*! \cond
   */

  public:

    typedef typename
    detail::conditional_output_iterator_base<BinaryPredicate, OutputIterator>::type
    super_t;

    friend class thrust::iterator_core_access;
  /*! \endcond
   */

  /*! This constructor takes as argument an \c OutputIterator and an \c
   * BinaryPredicate and copies them to a new \p conditional_output_iterator
   *
   * \param out An \c OutputIterator pointing to the output range whereto the result of 
   *            \p conditional_output_iterator's \c BinaryPredicate will be written.
   * \param fun An \c BinaryPredicate used to conditional the objects assigned to
   *            this \p conditional_output_iterator.
   */
    __host__ __device__
    conditional_output_iterator(OutputIterator const& out, BinaryPredicate fun) : super_t(out), fun(fun)
    {
    }

    /*! \cond
     */
  private:

    __host__ __device__
    typename super_t::reference dereference() const
    {
        return detail::conditional_output_iterator_proxy<BinaryPredicate, OutputIterator>(this->base_reference(), fun);
    }

    BinaryPredicate fun;

    /*! \endcond
     */
}; // end conditional_output_iterator

/* \p make_conditional_output_iterator creates a \p conditional_output_iterator from
 * an \c OutputIterator and \c BinaryPredicate.
 *
 * \param out The \c OutputIterator pointing to the output range of the newly
 *            created \p conditional_output_iterator
 * \param fun The \c BinaryPredicate conditional the object before assigning it to
 *            \c out by the newly created \p conditional_output_iterator
 * \see conditional_output_iterator
 */

template <typename BinaryPredicate, typename OutputIterator>
conditional_output_iterator<BinaryPredicate, OutputIterator>
__host__ __device__
make_conditional_output_iterator(OutputIterator out, BinaryPredicate fun)
{
    return conditional_output_iterator<BinaryPredicate, OutputIterator>(out, fun);
} // end make_conditional_output_iterator

/*! \} // end fancyiterators
 */

/*! \} // end iterators
 */

} // end thrust
