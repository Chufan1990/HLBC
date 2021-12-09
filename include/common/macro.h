
/**
 * @file macro.h
 */

#pragma once

#include "glog/logging.h"
#include "ros/console.h"
#include "ros/this_node.h"

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

#ifndef MODULE_NAME
#define MODULE_NAME ros::this_node::getName()
#endif

#define AERROR(args) ROS_ERROR_STREAM(args)
#define AINFO(args) ROS_INFO_STREAM(args)
#define AWARN(args) ROS_WARN_STREAM(args)
#define ADEBUG(args) ROS_DEBUG_STREAM(args)

#define AERROR_EVERY(period, args) \
  ROS_INFO_STREAM_THROTTLE(period / 1000.0, args)
#define AINFO_EVERY(period, args) \
  ROS_INFO_STREAM_THROTTLE(period / 1000.0, args)
#define AWARN_EVERY(period, args) \
  ROS_WARN_STREAM_THROTTLE(period / 1000.0, args)
#define ADEBUG_EVERY(period, args) \
  ROS_DEBUG_STREAM_THROTTLE(period / 1000.0, args)

#define AERROR_IF(cond, args) ROS_ERROR_STREAM_COND(cond, args)
#define ADEBUG_IF(cond, args) ROS_DEBUG_STREAM_COND(cond, args)
#define AINFO_IF(cond, args) ROS_INFO_STREAM_COND(cond, args)
#define AWARN_IF(cond, args) ROS_WARN_STREAM_COND(cond, args)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <type_traits>
#include <utility>

#if __GNUC__ >= 3
#define cyber_likely(x) (__builtin_expect((x), 1))
#define cyber_unlikely(x) (__builtin_expect((x), 0))
#else
#define cyber_likely(x) (x)
#define cyber_unlikely(x) (x)
#endif

#define CACHELINE_SIZE 64

#define DEFINE_TYPE_TRAIT(name, func)                      \
  template <typename T>                                    \
  struct name {                                            \
    template <typename Class>                              \
    static constexpr bool Test(decltype(&Class::func) *) { \
      return true;                                         \
    }                                                      \
    template <typename>                                    \
    static constexpr bool Test(...) {                      \
      return false;                                        \
    }                                                      \
                                                           \
    static constexpr bool value = Test<T>(nullptr);        \
  };                                                       \
                                                           \
  template <typename T>                                    \
  constexpr bool name<T>::value;

inline void cpu_relax() {
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#else
  asm volatile("rep; nop" ::: "memory");
#endif
}

inline void *CheckedMalloc(size_t size) {
  void *ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

inline void *CheckedCalloc(size_t num, size_t size) {
  void *ptr = std::calloc(num, size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

DEFINE_TYPE_TRAIT(HasShutdown, Shutdown)

template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T *instance) {
  instance->Shutdown();
}

template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(
    T *instance) {
  (void)instance;
}

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      CallShutdown(instance);                                             \
    }                                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)
