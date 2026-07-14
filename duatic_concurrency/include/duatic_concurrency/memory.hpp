#pragma once

#include <memory>

#define CACHE_ALIGNED alignas(std::hardware_destructive_interference_size)

namespace duatic::concurrency
{

template <typename Data>
struct CACHE_ALIGNED CacheAlignedDataWrapper
{
  using value_type = Data;

  value_type data;

  template <typename... Args>
    requires(sizeof...(Args) > 0) && std::is_rvalue_reference_v<Args&&...>
  inline CacheAlignedDataWrapper(Args&&... args) : data(std::forward<Args>(args)...)
  {
  }

  template <typename... Args>
  inline CacheAlignedDataWrapper(const Args&... args) : data(args...)
  {
  }
};

}  // namespace duatic::concurrency
