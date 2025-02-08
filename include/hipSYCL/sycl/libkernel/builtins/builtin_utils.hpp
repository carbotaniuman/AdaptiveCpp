/*
 * This file is part of AdaptiveCpp, an implementation of SYCL and C++ standard
 * parallelism for CPUs and GPUs.
 *
 * Copyright The AdaptiveCpp Contributors
 *
 * AdaptiveCpp is released under the BSD 2-Clause "Simplified" License.
 * See file LICENSE in the project root for full license details.
 */
// SPDX-License-Identifier: BSD-2-Clause
#ifndef HIPSYCL_BUILTIN_UTILS_HPP
#define HIPSYCL_BUILTIN_UTILS_HPP

#include <cstdint>
#include <type_traits>

#include "hipSYCL/sycl/libkernel/backend.hpp"
#include "hipSYCL/sycl/libkernel/marray.hpp"
#include "hipSYCL/sycl/libkernel/vec.hpp"

#define HIPSYCL_PP_CONCATENATE_IMPL(x,y) x ## y
#define HIPSYCL_PP_CONCATENATE(x, y) HIPSYCL_PP_CONCATENATE_IMPL(x,y)

namespace hipsycl::sycl::detail {
  struct scalar_type_tag {};
  struct marray_type_tag {};
  struct vec_type_tag {};

  template <typename T>
  struct builtin_input {
    using return_type = T;
    using element_type = T;
    using intlike_type = int;
    using tag_type = scalar_type_tag;
    static constexpr bool is_nonscalar = false;
    static constexpr std::size_t num_elems = 1;
  };

  template <typename DataT, std::size_t NumElements>
  struct builtin_input<marray<DataT, NumElements>> {
    using element_type = DataT;
    using return_type = marray<DataT, NumElements>;
    using intlike_type = marray<int, NumElements>;
    using tag_type = marray_type_tag;
    static constexpr bool is_nonscalar = true;
    static constexpr std::size_t num_elems = NumElements;
  };

  template <typename T, int N, class VectorStorage>
  struct builtin_input<vec<T, N, VectorStorage>> {
    using element_type = T;
    using return_type = vec<T, N>;
    using intlike_type = vec<int32_t, N>;
    using tag_type = vec_type_tag;
    static constexpr bool is_nonscalar = true;
    static constexpr std::size_t num_elems = N;
  };

  template <typename T>
  using builtin_input_element_t = typename builtin_input<T>::element_type;

  template <typename T>
  using builtin_input_return_t = typename builtin_input<T>::return_type;

  template <typename T>
  using builtin_input_intlike_t = typename builtin_input<T>::intlike_type;

  template <typename T>
  constexpr bool builtin_input_is_nonscalar_v = builtin_input<T>::is_nonscalar;

  template <typename T>
  constexpr std::size_t builtin_input_num_elems_v = builtin_input<T>::num_elems;

  // Helper for testing the valid elems for `is_compatible_builtin_nonscalar`
  template <typename... Ts>
  struct is_in_type_set {
    template <typename T>
    struct scalar_tester : std::bool_constant<(std::is_same_v<T, Ts> || ...)> {};
  };

  // Partial specialization that lets no elements through
  template <>
  struct is_in_type_set<> {
    template <typename T>
    struct scalar_tester : std::false_type {};
  };

  // These two overloads allow us to select between scalars
  // and nonscalars with `if constexpr`.
  template <typename T, std::enable_if_t<!builtin_input_is_nonscalar_v<T>, bool> = true>
  ACPP_UNIVERSAL_TARGET HIPSYCL_FORCE_INLINE
  builtin_input_element_t<T>& data_element(T& container, std::size_t index) {
    return container;
  }

  template <typename T, std::enable_if_t<builtin_input_is_nonscalar_v<T>, bool> = true>
  ACPP_UNIVERSAL_TARGET HIPSYCL_FORCE_INLINE
  builtin_input_element_t<T>& data_element(T& container, std::size_t index) {
    return container[index];
  }

  template <typename T, typename... Ts>
  struct builtin_inputs_are_compatible : std::bool_constant<
    (std::is_same_v<builtin_input_return_t<T>, builtin_input_return_t<Ts>> && ...)
  > {};

  template <typename T>
  struct builtin_inputs_are_compatible<T> : std::true_type {};

  template <typename T, typename... Ts>
  constexpr bool builtin_inputs_are_compatible_v = builtin_inputs_are_compatible<T, Ts...>::value;
}

#endif
