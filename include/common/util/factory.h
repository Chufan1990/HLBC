/**
 * @file factory.h
 * @author chufan kong (chufan1990@gmail.com)
 * @brief defines the Factory class
 * @version 0.1
 * @date 2021-11-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <map>
#include <memory>
#include <utility>

/**
 * @namespace common::util
 * @brief common::util
 */
namespace autoagric {
namespace common {
namespace util {

/**
 * @class Factory
 * @brief
 */
template <typename IdentifierType, class AbstractProduct,
          class ProductorCreater = AbstractProduct* (*)(),
          class MapContainer = std::map<IdentifierType, ProductorCreater>>
class Factory {
 public:
  bool Register(const IdentifierType& id, ProductorCreater creator) {
    return producers_.insert(std::make_pair(id, creator)).second;
  }

  bool Contains(const IdentifierType& id) {
    return producers_.find(id) != producers_.end();
  }

  bool Unregister(const IdentifierType& id) {
    return producers_.erase(id) == 1;
  }

  void Clear() { producers_.clear(); }

  bool Empty() const { return producers_.empty(); }

  // When Args is a forwarding reference (a function argument that is declared
  // as an rvalue reference to a cv-unqualified function template parameter),
  // this overload forwards the argument to another function with the value
  // category it had when passed to the calling function.
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType& id,
                                                      Args&&... args) {
    auto id_iter = producers_.find(id);
    if (id_iter != producers_.end()) {
      return std::unique_ptr<AbstractProduct>(
          (id_iter->second)(std::forward<Args>(args)...));
    }
    return nullptr;
  }

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType& id,
                                                Args&&... args) {
    auto result = CreateObjectOrNull(id, std::forward<Args>(args)...);
    AERROR_IF(!result, "", "Factory could not create Object of type : " << id);
    return result;
  }

 private:
  MapContainer producers_;
};
}  // namespace util
}  // namespace common
}  // namespace autoagric