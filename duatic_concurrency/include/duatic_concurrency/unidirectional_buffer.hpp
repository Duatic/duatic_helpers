#pragma once

#include <atomic>
#include <array>

#include "duatic_concurrency/memory.hpp"

namespace duatic::concurrency
{

template <typename Data>
class UnidirectionalBuffer
{
public:
  using value_type = Data;
  using aligned_data = CacheAlignedDataWrapper<Data>;

public:
  template <typename... Args>
  explicit inline UnidirectionalBuffer(Args&&... args)
    : buffer_{ aligned_data(std::forward<Args>(args)...) }, src_idx_(0), buff_idx_(1), dst_idx_(2)
  {
  }

  template <typename... Args>
  explicit inline UnidirectionalBuffer(const Args&... args)
    : buffer_{ aligned_data(args...) }, src_idx_(0), buff_idx_(1), dst_idx_(2)
  {
  }

private:
  using index_type = uint8_t;
  using atomic_index = std::atomic<index_type>;
  static_assert(std::is_integral_v<index_type> && std::is_unsigned_v<index_type>);
  static_assert(atomic_index::is_always_lock_free, "This hardware is not suitable for the herein-used lock-free atomic "
                                                   "index_type. Cannot compile this code on this machine!");

  static constexpr std::size_t buffer_size = 3;
  static constexpr index_type update_flag = 1 << (sizeof(index_type) * 8 - 1);  // MSB
  static constexpr index_type addr_mask = ~update_flag;                         // everything except the MSB
  static_assert((buffer_size - 1) & addr_mask == (buffer_size - 1));

  CACHE_ALIGNED std::array<aligned_data, buffer_size> buffer_;
  CACHE_ALIGNED index_type src_idx_;     // source
  CACHE_ALIGNED atomic_index buff_idx_;  // atomic exchange buffer
  CACHE_ALIGNED index_type dst_idx_;     // destination
};

}  // namespace duatic::concurrency
