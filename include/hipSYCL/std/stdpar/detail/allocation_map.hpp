/*
 * This file is part of hipSYCL, a SYCL implementation based on CUDA/HIP
 *
 * Copyright (c) 2023 Aksel Alpay
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HIPSYCL_ALLOCATION_MAP_HPP
#define HIPSYCL_ALLOCATION_MAP_HPP


#include <cstdlib>
#include <cstdint>
#include <type_traits>
#include <atomic>
#include <algorithm>

extern "C" void *__libc_malloc(size_t);
extern "C" void __libc_free(void*);

namespace hipsycl::stdpar {

struct default_allocation_map_payload {};

template<class UserPayload = default_allocation_map_payload>
class allocation_map {
public:
  static_assert(sizeof(void*) == 8, "Unsupported pointer size");
  static_assert(std::is_trivial_v<UserPayload>, "UserPayload must be trivial type");

  allocation_map()
  : _num_in_progress_operations{0} {}

  struct value_type : public UserPayload {
    std::size_t allocation_size;
  };

  // Access entry of allocation that address belongs to, or nullptr if the address
  // does not belong to a known allocation.
  value_type* get_entry(uint64_t address, uint64_t& root_address) noexcept {
    insert_or_get_entry_lock lock{_num_in_progress_operations};
    root_address = 0;
    int num_leaf_attempts = 0;
    return get_entry(_root, address, num_leaf_attempts, root_address);
  }

  // Insert new element. Element's allocation range must be
  // non-overlapping w.r.t existing entries.
  // ~0ull is unsupported, because then non-zero allocation
  // ranges cannot be expressed.
  bool insert(uint64_t address, const value_type& v) {
    insert_or_get_entry_lock lock{_num_in_progress_operations};
    return insert(_root, address, v);
  }

  bool erase(uint64_t address) {
    erase_lock lock{_num_in_progress_operations};
    return erase(_root, address);
  }

  ~allocation_map() {
    for(int i = 0; i < get_num_entries_in_level(root_level_idx); ++i) {
      auto* ptr = _root.children[i].load(std::memory_order::memory_order_acquire);
      if(ptr)
        release(*ptr);
    }
  }
    
private:
  // Useful for debugging/printing
  template<class F>
  void with_decomposed_address(uint64_t address, int current_level, F&& handler) {
    for(int i = root_level_idx; i >= current_level; --i) {
      handler(get_index_in_level(address, i));
    }
    for(int i = current_level - 1; i >= 0; --i) {
      handler(-1);
    }
  }

  template<class Ostream>
  void print(Ostream& ostr, uint64_t address, int level) {
    with_decomposed_address(address, level, [&](int x){
      if(x >= 0)
        ostr << x << ".";
      else
        ostr << "x";
    });
    ostr << "\n";
  }

  static constexpr int num_levels = 16;
  static constexpr int root_level_idx = num_levels - 1;
  static constexpr int bitsizes[num_levels] = {4, 4, 4, 4, 4, 4, 4, 4,
                                               4, 4, 4, 4, 4, 4, 4, 4};

  static constexpr int get_num_entries_in_level(int level) {
    return 1ull << bitsizes[level];
  }

  static constexpr int get_bitoffset_in_level(int level) {
    int result = 0;
    for(int i = 0; i < level; ++i) {
      result += bitsizes[i];
    }
    return result;
  }

  static constexpr int get_index_in_level(uint64_t address, int level) {
    uint64_t bitmask = get_n_low_bits_set(bitsizes[level]);
    return (address >> get_bitoffset_in_level(level)) & bitmask;
  }

  static constexpr uint64_t get_n_low_bits_set(int n) {
    if(n == 64)
      return ~0ull;
    return (1ull << n) - 1;
  }

  struct leaf_node {
    leaf_node()
    : num_entries {} {
      for(int i = 0; i < get_num_entries_in_level(0); ++i) {
        entries[i].allocation_size = 0;
      }
    }

    value_type entries [get_num_entries_in_level(0)];
    std::atomic<int> num_entries;
  };

  template<int Level>
  struct intermediate_node {
  private:
      static constexpr auto make_child() {
      if constexpr (Level > 1) return 
        intermediate_node<Level - 1>{};
      else return leaf_node{};
    }
  public:
    intermediate_node()
    : children{}, num_entries{} {}

    using child_type = decltype(make_child());

    std::atomic<child_type*> children [get_num_entries_in_level(Level)];
    std::atomic<int> num_entries;
  };

  template<class T>
  static T* alloc(int count) {
    return static_cast<T*>(__libc_malloc(sizeof(T) * count));
  }

  static void free(void* ptr) {
    __libc_free(ptr);
  }

  value_type *get_entry(leaf_node &current_node, uint64_t address,
                        int &num_leaf_attempts,
                        uint64_t &root_address) noexcept {
    int start_address = 0;

    uint64_t max_local_address =
        root_address | (get_num_entries_in_level(0) - 1);
    
    if(max_local_address <= address)
      start_address = get_num_entries_in_level(0) - 1;
    else
      start_address = get_index_in_level(address, 0);

    for (int local_address = start_address; local_address >= 0;
         --local_address) {
      
      auto& element = current_node.entries[local_address];

      std::size_t allocation_size =
          __atomic_load_n(&(element.allocation_size), __ATOMIC_ACQUIRE);
      if(allocation_size > 0) {

        uint64_t root_address_candidate =
            root_address |
            (static_cast<uint64_t>(local_address) << get_bitoffset_in_level(0));

        uint64_t allocation_end = root_address_candidate + allocation_size;
        if(address >= root_address_candidate && address < allocation_end) {
          root_address = root_address_candidate;
          return &element;
        } else {
          return nullptr;
        }
        
      }
    }
    return nullptr;
  }

  template <int Level>
  value_type *get_entry(intermediate_node<Level> &current_node,
                        uint64_t address,
                        int& num_leaf_attempts,
                        uint64_t& root_address) noexcept {
    // If the queried address is too close to the next allocation,
    // it can happen that the search converges on the next allocation.
    // Therefore, to exclude that case, if a search fails, we also
    // need to try again with the next allocation before that.
    // This variable counts how many leaves we have accessed. If it
    // reaches two, we can abort.
    if constexpr(Level == root_level_idx) {
      num_leaf_attempts = 0;
    }

    uint64_t max_local_address =
        root_address |
        get_n_low_bits_set(get_bitoffset_in_level(Level) + bitsizes[Level]);

    // We are always looking for the next allocation preceding the
    // current address. If the maximum local address in this node
    // cannot reach the search address, (e.g. if we are looking in
    // a preceding node at the same level), we need to start from 
    // the maximum address. Otherwise, we need to look at the bits
    // set in this address.
    int start_address = 0;
    if(max_local_address <= address)
      start_address = get_num_entries_in_level(Level) - 1;
    else
      start_address = get_index_in_level(address, Level);

    for (int local_address = start_address;
         local_address >= 0; --local_address) {
      
      auto *ptr = current_node.children[local_address].load(
          std::memory_order::memory_order_acquire);
      
      if(ptr) {
        uint64_t root_address_candidate =
            root_address | (static_cast<uint64_t>(local_address)
                            << get_bitoffset_in_level(Level));

        auto* ret = get_entry(*ptr, address, num_leaf_attempts,
                              root_address_candidate);
        // If we are in level 1, ret refers to a leaf node
        if constexpr(Level == 1) {
          ++num_leaf_attempts;
        }

        if(ret) {
          root_address = root_address_candidate;
          return ret;
        } else if(num_leaf_attempts >= 2) {
          // We can abort if we have looked at the first hit leaf node,
          // and the one before that.
          return nullptr;
        }
      }
    }
    return nullptr;
  }

  bool insert(leaf_node &current_node, uint64_t address, const value_type &v) {

    int local_address = get_index_in_level(address, 0);

    std::size_t *allocation_size_ptr =
        &(current_node.entries[local_address].allocation_size);

    std::size_t allocation_size = __atomic_load_n(allocation_size_ptr, __ATOMIC_ACQUIRE);
    if(allocation_size > 0) {
      // Entry is already occupied
      return false;
    }
    
    __atomic_store_n(allocation_size_ptr, v.allocation_size, __ATOMIC_RELEASE);
    current_node.entries[local_address].UserPayload::operator=(v);
    
    auto val = current_node.num_entries.fetch_add(
        1, std::memory_order::memory_order_acq_rel);

    return true;
  }

  template <int Level>
  bool insert(intermediate_node<Level> &current_node, uint64_t address,
              const value_type &v) {
    using child_t = typename intermediate_node<Level>::child_type;

    int local_address = get_index_in_level(address, Level);
    
    auto *ptr = current_node.children[local_address].load(
        std::memory_order::memory_order_acquire);
    
    if(!ptr) {
      child_t* new_child = alloc<child_t>(1);
      new (new_child) child_t{};

      if (!current_node.children[local_address].compare_exchange_strong(
              ptr /* == nullptr*/, new_child, std::memory_order_acq_rel)) {
        // Assigning new child has failed because child is no longer nullptr
        // -> free new child again
        destroy(*new_child);
        this->free(new_child);
      } else {
        current_node.num_entries.fetch_add(
            1, std::memory_order::memory_order_acq_rel);
        ptr = new_child;
      }
    }

    return insert(*ptr, address, v);
  }

  bool erase(leaf_node& current_node, uint64_t address) {
    int local_address = get_index_in_level(address, 0);

    std::size_t *allocation_size_ptr =
        &(current_node.entries[local_address].allocation_size);
    // Entry was already deleted or does not exist
    if(__atomic_load_n(allocation_size_ptr, __ATOMIC_ACQUIRE) == 0)
      return false;

    __atomic_store_n(allocation_size_ptr, 0, __ATOMIC_RELEASE);

    current_node.num_entries.fetch_sub(
        1, std::memory_order::memory_order_acq_rel);
    
    return true;
  }

  template <int Level>
  bool erase(intermediate_node<Level> &current_node, uint64_t address) {

    int local_address = get_index_in_level(address, Level);
    auto *ptr = current_node.children[local_address].load(
        std::memory_order::memory_order_acquire);
    if(!ptr)
      return false;
    
    bool result = erase(*ptr, address);
    if(result) {
      if(ptr->num_entries.load(std::memory_order::memory_order_acquire) == 0) {
        auto *current_ptr = current_node.children[local_address].exchange(
            nullptr, std::memory_order::memory_order_acq_rel);
        // TODO: We could potentially get erase() lock-free
        // by counting by how many ops each node is currently used,
        // and waiting here until the count turns to 0.
        if(current_ptr) {
          destroy(*current_ptr);
          this->free(current_ptr);
          current_node.num_entries.fetch_sub(
              1, std::memory_order::memory_order_acq_rel);
        }
      }
    }
    return result;
  }

  void release(leaf_node& current_node) {
    destroy(current_node);
  }

  template<int Level>
  void release(intermediate_node<Level>& current_node) {
    for(int i = 0; i < get_num_entries_in_level(Level); ++i){
      if (auto *ptr = current_node.children[i].load(
              std::memory_order::memory_order_acquire)) {
        release(*ptr);
        free(ptr);
      }
    }
    destroy(current_node);
  }

  void destroy(leaf_node& node) {
    node.~leaf_node();
  }

  template<int Level>
  void destroy(intermediate_node<Level>& node) {
    node.~intermediate_node<Level>();
  }

  struct erase_lock {
  public:
    erase_lock(std::atomic<int>& op_counter)
    : _op_counter{op_counter} {
      int expected = 0;
      while (!_op_counter.compare_exchange_strong(
          expected, -1, std::memory_order::memory_order_release,
          std::memory_order_relaxed)) {
        expected = 0;
      }
    }

    ~erase_lock() {
      _op_counter.store(0, std::memory_order::memory_order_release);
    }
  private:
    std::atomic<int>& _op_counter;
  };

  struct insert_or_get_entry_lock {
  public:
    insert_or_get_entry_lock(std::atomic<int>& op_counter)
    : _op_counter{op_counter} {
      int expected = std::max(0, _op_counter.load(std::memory_order_acquire));
      while (!_op_counter.compare_exchange_strong(
          expected, expected+1, std::memory_order::memory_order_release,
          std::memory_order_relaxed)) {
        if(expected < 0)
          expected = 0;
      }
    }

    ~insert_or_get_entry_lock() {
      _op_counter.fetch_sub(1, std::memory_order::memory_order_acq_rel);
    }
  private:
   std::atomic<int>& _op_counter;
  };

  intermediate_node<root_level_idx> _root;
  std::atomic<int> _num_in_progress_operations;
};

}

#endif
