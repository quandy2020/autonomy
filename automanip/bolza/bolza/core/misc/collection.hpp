/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file collection.hpp
 * @brief Implements the common add/get interface for cost and constraint collections.
 */


#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace bolza {

/**
 * Implements the common add/get interface for cost and constraint collections.
 *
 * @tparam T : Type of the terms in the collection.
 */
template <typename T>
class Collection {
 public:
  /**
   * @brief Constructs a Collection.
   */
  Collection() = default;
  /**
   * @brief Destructor.
   */
  virtual ~Collection() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual Collection* Clone() const { /**
  * @brief Collection.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new Collection(*this); }

  /**
   * @brief Checks if the collection has no elements.
   * @return True on success.
   */
  bool empty() const { /**
  * @brief Empty.
  * @return Result value.
  */
 return terms_.empty(); }

  /** Erases all elements from the Collection. */
  void clear();

  /**
   * Adds a term to the collection, and transfer ownership to the collection
   * The provided name must be unique and is later used to access the cost term.
   * @param [in] name: Name stored along with the term.
   * @param [in] term: Term to be added.
   */
  void Add(std::string name, std::unique_ptr<T> term);

  /**
   * @brief Erases a term from the collection.
   * @param [in] nullptr: Nullptr.
   * @return True if the term was in the Collection and false if the term was not found in the Collection.
   */
  bool erase(const std::string& name) { return (Extract(name) != nullptr); }

  /**
   * Removes a term from the Collection and returns it as a unique_ptr.
   *
   * @param [in] name: Name of the term.
   * @return A unique pointer to the extracted term. If the term was not found it returns nullptr.
   */
  std::unique_ptr<T> Extract(const std::string& name);

  /**
   * Use to modify a term.
   * @tparam Derived: derived class of base type T to cast to. Casts to the base class by default
   * @param [in] name: Name of the term to modify
   * @return A reference to the underlying term
   */
  template <typename Derived = T>
  /**
   * @brief Get.
   * @param [in] name: Name.
   * @return Result value.
   */
  Derived& get(const std::string& name);

  /**
   * Finds the index of the term in the stored map.
   *
   * @param [in] name: Name of the term.
   * @param [out] index: Term index.
   * @return True if the name found in the collection.
   */
  bool GetTermIndex(const std::string& name, size_t& index) const;

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  Collection(const Collection& other);

  //! Contains all terms in the order they were added
  std::vector<std::unique_ptr<T>> terms_;

 private:
  //! Lookup from cost term name to index in the cost term vector
  std::unordered_map<std::string, size_t> termNameMap_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Clear.
 * @return Result value.
 */
void Collection<T>::clear() {
  /**
   * @brief Clear.
   * @return Result value.
   */
  terms_.clear();
  /**
   * @brief Clear.
   * @return Result value.
   */
  termNameMap_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Add.
 * @param [in] name: Name.
 * @param [in] term: Term.
 * @return Result value.
 */
void Collection<T>::Add(std::string name, std::unique_ptr<T> term) {
  /**
   * @brief Size.
   * @return Result value.
   */
  const size_t nextIndex = terms_.size();
  /**
   * @brief Emplace.
   * @param [in] nextIndex: Next Index.
   * @return Result value.
   */
  auto info = termNameMap_.emplace(std::move(name), nextIndex);
  if (info.second) {
    /**
     * @brief Push back.
     * @return Result value.
     */
    terms_.push_back(std::move(term));
  } else {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error(std::string("[Collection::Add] Term with name \"") + info.first->first + "\" already exists");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Extract.
 * @param [in] name: Name.
 * @return Owning pointer.
 */
std::unique_ptr<T> Collection<T>::Extract(const std::string& name) {
  // find the term iterator with the name
  /**
   * @brief Find.
   * @param [in] name: Name.
   * @return Result value.
   */
  const auto termItr = termNameMap_.find(name);

  // term was not found
  if (termItr == termNameMap_.end()) {
    return nullptr;
  }

  // adjust the index of the terms added after the requested term
  const size_t termInd = termItr->second;
  for (auto itr = termNameMap_.begin(); /**
  * @brief End.
  * @return Result value.
  */
 itr != termNameMap_.end(); ++itr) {
    if (itr->second > termInd) {
      --itr->second;
    }
  }

  // remove term from map
  /**
   * @brief Erase.
   * @param [in] termItr: Term Itr.
   * @return Result value.
   */
  termNameMap_.erase(termItr);

  // get the term
  /**
   * @brief Move.
   * @return Result value.
   */
  auto term = (std::move(terms_[termInd]));
  // remove the term
  /**
   * @brief Erase.
   * @param [in] termInd: Term Ind.
   * @return Result value.
   */
  terms_.erase(terms_.begin() + termInd);

  return term;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
template <typename Derived>
/**
 * @brief Get.
 * @param [in] name: Name.
 * @return Result value.
 */
Derived& Collection<T>::get(const std::string& name) {
  static_assert(std::is_base_of<T, Derived>::value, "Template argument must derive from the base type of this collection");
  // if the key does not exist throws an exception
  /**
   * @brief At.
   * @param [in] name: Name.
   * @return Result value.
   */
  const auto index = termNameMap_.at(name);
  return dynamic_cast<Derived&>(*terms_[index]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Collection.
 * @param [in] termNameMap_: Term Name Map.
 * @return Result value.
 */
Collection<T>::Collection(const Collection& other) : termNameMap_(other.termNameMap_) {
  // Loop through all terms and clone. The name map can be copied directly because the order stays the same.
  /**
   * @brief Reserve.
   * @return Result value.
   */
  terms_.reserve(other.terms_.size());
  for (const auto& term : other.terms_) {
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    terms_.emplace_back(term->Clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
/**
 * @brief Get Term Index.
 * @param [in] name: Name.
 * @param [in,out] index: Index.
 * @return Result value.
 */
bool Collection<T>::GetTermIndex(const std::string& name, size_t& index) const {
  /**
   * @brief Find.
   * @param [in] name: Name.
   * @return Result value.
   */
  auto itr = termNameMap_.find(name);
  if (itr != termNameMap_.cend()) {
    index = itr->second;
    return true;
  } else {
    /**
     * @brief Size.
     * @return Result value.
     */
    index = termNameMap_.size();
    return false;
  }
}

/**
 * Helper function for merging two vectors by moving objects.
 * @param [in] v1: vector to move objects to
 * @param [in] v2: vector to move objects from
 */
template <typename T, typename Allocator>
/**
 * @brief Append Vector To Vector By Moving.
 * @param [in,out] v1: V1.
 * @param [in,out] v2: V2.
 */
inline void AppendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>&& v2) {
  /**
   * @brief Insert.
   * @return Result value.
   */
  v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
}

}  // namespace bolza
