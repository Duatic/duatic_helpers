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
  inline UnidirectionalBuffer(const Args&... args)
    : buffer_{ aligned_data(args...) }, write_idx(2), buff_idx_(1), read_idx_(0)  // initialized indizes are important
  {
  }

  // THE INPUT SIDE: WRITE

  inline value_type& write()
  {
    return buffer_[write_idx].data;
  }
  inline void write(value_type&& data)
  {
    write() = std::move(data);
  }
  inline void write(const value_type& data)
  {
    write() = data;
  }

  inline void publish_write()
  {
    // atomic swap indizes with update indication
    write_idx = buff_idx_.exchange(write_idx | update_flag, std::memory_order_release) &
                addr_mask;  // always store with update flag and remove the update flag on load
    assert((write_idx < buffer_size) && "Invariance Violation: write_idx out of bounds. "
                                        "There is something serious going wrong here!");
  }

  inline void publish_write(value_type&& data)
  {
    write(std::move(data));
    publish_write();
  }

  inline void publish_write(const value_type& data)
  {
    write(data);
    publish_write();
  }

  // THE OUTPUT SIDE: READ

  inline const value_type& update_read()
  {
    // check for an available update
    const index_type update_available =
        buff_idx_.load(std::memory_order_acquire) & update_flag;  // mask with update bit
    if (update_available) {  // update dst data copy only if there is an update available
      // atomic update dst data copy
      read_idx_ = buff_idx_.exchange(read_idx_, std::memory_order_acquire) &
                  addr_mask;  // always remove the update flag and store without
      assert((read_idx_ < buffer_size) && "Invariance Violation: read_idx_ out of bounds. "
                                          "There is something serious going wrong here!");
    }
    return read();
  }

  inline const value_type& read() const
  {
    return buffer_[read_idx_].data;
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
  static_assert(((buffer_size - 1) & addr_mask) == (buffer_size - 1));

  CACHE_ALIGNED std::array<aligned_data, buffer_size> buffer_;
  CACHE_ALIGNED index_type write_idx;    // source
  CACHE_ALIGNED atomic_index buff_idx_;  // atomic exchange buffer
  CACHE_ALIGNED index_type read_idx_;    // destination
};

}  // namespace duatic::concurrency
