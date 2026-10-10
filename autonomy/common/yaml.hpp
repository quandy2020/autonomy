/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file yaml.hpp
 * @brief yaml-cpp API compatibility layer backed by fkYAML.
 *
 * Header-only. Provides a `namespace YAML` that mimics the subset of the
 * yaml-cpp API used across the autonomy tree:
 *
 *   - YAML::Node (shared-ownership handle: `n["k"] = v` mutates the tree),
 *     YAML::Load / LoadFile / LoadAll
 *   - YAML::convert<T> customisation point and Node::as<T>() / as<T>(fallback)
 *   - YAML::Emitter with the usual manipulators (BeginMap, Key, Flow, ...)
 *   - YAML::Exception / ParserException / BadFile / InvalidNode / ...
 *
 * Parsing is delegated to `fkyaml::node::deserialize`; the result is converted
 * into a small yaml-cpp-style document tree (scalars are kept as text, so
 * `as<int>()`, `as<bool>()`, `as<std::string>()` ... behave like yaml-cpp).
 * Mapping order is preserved (fkYAML `ordered_map`).
 *
 * Link `fkYAML::fkYAML` (alias `autonomy::fkYAML`) to use this header. The
 * `yaml-cpp/yaml.h` / `yaml-cpp/emitter.h` shims under
 * `autonomy/common/yaml_cpp_shim/` simply include this file.
 *
 * Known differences from yaml-cpp:
 *   - Parse errors carry no line/column (`Mark` is null).
 *   - Scalar text of floats is re-formatted (shortest round-trip form).
 *   - Anchors/aliases are expanded into copies at load time.
 *   - Tags, comments and `Node::SetStyle` are not supported.
 */

#ifndef AUTONOMY_COMMON_YAML_HPP_
#define AUTONOMY_COMMON_YAML_HPP_

#include <fkYAML/node.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace YAML {

// ============================================================================
// Mark / exceptions
// ============================================================================

struct Mark {
  Mark() : pos(-1), line(-1), column(-1) {}
  static Mark null_mark() { return Mark(); }
  bool is_null() const { return pos == -1 && line == -1 && column == -1; }

  int pos;
  int line;
  int column;
};

class Exception : public std::runtime_error {
 public:
  Exception(const Mark& mark_, const std::string& msg_)
      : std::runtime_error(build_what(mark_, msg_)), mark(mark_), msg(msg_) {}
  explicit Exception(const std::string& msg_)
      : Exception(Mark::null_mark(), msg_) {}
  ~Exception() noexcept override = default;

  Mark mark;
  std::string msg;

 private:
  static std::string build_what(const Mark& mark, const std::string& msg) {
    if (mark.is_null()) {
      return msg;
    }
    std::stringstream output;
    output << "yaml-cpp: error at line " << mark.line + 1 << ", column "
           << mark.column + 1 << ": " << msg;
    return output.str();
  }
};

class ParserException : public Exception {
 public:
  ParserException(const Mark& mark_, const std::string& msg_)
      : Exception(mark_, msg_) {}
  explicit ParserException(const std::string& msg_) : Exception(msg_) {}
};

class RepresentationException : public Exception {
 public:
  RepresentationException(const Mark& mark_, const std::string& msg_)
      : Exception(mark_, msg_) {}
  explicit RepresentationException(const std::string& msg_)
      : Exception(msg_) {}
};

class InvalidNode : public RepresentationException {
 public:
  explicit InvalidNode(const std::string& key = std::string())
      : RepresentationException(
            Mark::null_mark(),
            key.empty() ? std::string("invalid node; this may result from "
                                      "using a map iterator as a sequence "
                                      "iterator, or vice-versa")
                        : "invalid node; first invalid key: \"" + key + "\"") {}
};

class BadConversion : public RepresentationException {
 public:
  explicit BadConversion(const Mark& mark_ = Mark::null_mark(),
                         const std::string& detail = std::string())
      : RepresentationException(
            mark_, detail.empty() ? std::string("bad conversion")
                                  : "bad conversion: " + detail) {}
};

template <typename T>
class TypedBadConversion : public BadConversion {
 public:
  explicit TypedBadConversion(const Mark& mark_ = Mark::null_mark(),
                              const std::string& detail = std::string())
      : BadConversion(mark_, detail) {}
};

class BadSubscript : public RepresentationException {
 public:
  explicit BadSubscript(const std::string& key = std::string())
      : RepresentationException(
            Mark::null_mark(),
            "operator[] call on a scalar" +
                (key.empty() ? std::string() : " (key: \"" + key + "\")")) {}
};

class BadPushback : public RepresentationException {
 public:
  BadPushback()
      : RepresentationException(Mark::null_mark(),
                                "appending to a non-sequence") {}
};

class BadInsert : public RepresentationException {
 public:
  BadInsert()
      : RepresentationException(Mark::null_mark(),
                                "inserting in a non-convertible-to-map") {}
};

/// Thrown by LoadFile() when the file cannot be opened. Derives from
/// ParserException so that `catch (ParserException&)` covers every load error.
class BadFile : public ParserException {
 public:
  explicit BadFile(const std::string& filename = std::string())
      : ParserException(Mark::null_mark(),
                        filename.empty() ? std::string("bad file")
                                         : "bad file: " + filename) {}
};

// ============================================================================
// NodeType
// ============================================================================

struct NodeType {
  enum value { Undefined, Null, Scalar, Sequence, Map };
};

// ============================================================================
// Internal storage
// ============================================================================

class Node;

namespace detail {

struct Ref;
using RefPtr = std::shared_ptr<Ref>;

/// Shared payload of a node (what yaml-cpp calls node_data).
struct Content {
  NodeType::value type = NodeType::Undefined;
  std::string scalar;
  std::vector<RefPtr> seq;
  std::vector<std::pair<RefPtr, RefPtr>> map;
  Mark mark;
};

/// A position in the tree (what yaml-cpp calls node / node_ref).
struct Ref {
  std::shared_ptr<Content> c = std::make_shared<Content>();
  bool defined = true;
  /// Parents that created this node provisionally via non-const operator[];
  /// they become defined once this node is assigned.
  std::vector<std::weak_ptr<Ref>> deps;
};

inline RefPtr make_ref(NodeType::value type, bool defined) {
  auto r = std::make_shared<Ref>();
  r->c->type = type;
  r->defined = defined;
  return r;
}

inline void mark_defined(const RefPtr& r) {
  if (!r || r->defined) {
    return;
  }
  r->defined = true;
  auto deps = std::move(r->deps);
  r->deps.clear();
  for (auto& w : deps) {
    if (auto p = w.lock()) {
      mark_defined(p);
    }
  }
}

struct NodeAccess;

template <typename T>
inline std::string integral_text(T v) {
  if (std::is_signed<T>::value) {
    return std::to_string(static_cast<long long>(v));
  }
  return std::to_string(static_cast<unsigned long long>(v));
}

}  // namespace detail

// ============================================================================
// convert<T> customisation point
// ============================================================================

template <typename T, typename Enable = void>
struct convert;

namespace detail {
template <typename T, typename S>
struct as_if;
}  // namespace detail

class iterator;
using const_iterator = iterator;

// ============================================================================
// Node
// ============================================================================

class Node {
 public:
  using iterator = ::YAML::iterator;
  using const_iterator = ::YAML::iterator;

  Node() : valid_(true) {}
  explicit Node(NodeType::value type)
      : ref_(detail::make_ref(type, type != NodeType::Undefined)),
        valid_(true) {}
  template <typename T,
            typename = typename std::enable_if<
                !std::is_base_of<Node, T>::value &&
                !std::is_same<T, NodeType::value>::value>::type>
  explicit Node(const T& rhs) : valid_(true) {
    Assign(rhs);
  }
  Node(const Node&) = default;
  Node(Node&&) = default;
  ~Node() = default;

  // -- type queries ---------------------------------------------------------
  NodeType::value Type() const {
    if (!valid_) {
      return NodeType::Undefined;
    }
    if (!ref_) {
      return NodeType::Null;
    }
    return ref_->defined ? ref_->c->type : NodeType::Undefined;
  }
  bool IsDefined() const {
    if (!valid_) {
      return false;
    }
    return ref_ ? ref_->defined : true;
  }
  bool IsNull() const { return Type() == NodeType::Null; }
  bool IsScalar() const { return Type() == NodeType::Scalar; }
  bool IsSequence() const { return Type() == NodeType::Sequence; }
  bool IsMap() const { return Type() == NodeType::Map; }

  explicit operator bool() const { return IsDefined(); }
  bool operator!() const { return !IsDefined(); }

  // -- content --------------------------------------------------------------
  ::YAML::Mark Mark() const {
    if (valid_ && ref_) {
      return ref_->c->mark;
    }
    return ::YAML::Mark::null_mark();
  }
  const std::string& Scalar() const {
    static const std::string kEmpty;
    if (valid_ && ref_ && ref_->defined &&
        ref_->c->type == NodeType::Scalar) {
      return ref_->c->scalar;
    }
    return kEmpty;
  }
  const std::string& Tag() const {
    static const std::string kEmpty;
    return kEmpty;
  }

  template <typename T>
  T as() const;
  template <typename T, typename S>
  T as(const S& fallback) const;

  // -- assignment -----------------------------------------------------------
  Node& operator=(const Node& rhs) {
    if (this == &rhs || is(rhs)) {
      return *this;
    }
    AssignNode(rhs);
    return *this;
  }
  Node& operator=(Node&& rhs) {
    if (this == &rhs || is(rhs)) {
      return *this;
    }
    AssignNode(rhs);
    return *this;
  }
  template <typename T,
            typename = typename std::enable_if<
                !std::is_base_of<Node, T>::value>::type>
  Node& operator=(const T& rhs) {
    Assign(rhs);
    return *this;
  }
  Node& operator=(const char* rhs) {
    Assign(rhs);
    return *this;
  }
  Node& operator=(char* rhs) {
    Assign(static_cast<const char*>(rhs));
    return *this;
  }

  void reset(const Node& rhs = Node()) {
    ref_ = rhs.ref_;
    valid_ = rhs.valid_;
    invalid_key_ = rhs.invalid_key_;
  }
  bool is(const Node& rhs) const {
    if (!valid_ || !rhs.valid_ || !ref_ || !rhs.ref_) {
      return false;
    }
    return ref_ == rhs.ref_;
  }

  void Assign(const std::string& rhs) {
    EnsureNodeExists();
    auto& c = *ref_->c;
    ResetContent(c);
    c.type = NodeType::Scalar;
    c.scalar = rhs;
    detail::mark_defined(ref_);
  }
  void Assign(const char* rhs) { Assign(std::string(rhs)); }
  void Assign(char* rhs) { Assign(std::string(rhs)); }
  template <typename T>
  void Assign(const T& rhs) {
    AssignData(convert<T>::encode(rhs));
  }

  // -- sequence / map -------------------------------------------------------
  std::size_t size() const {
    if (!IsDefined() || !ref_) {
      return 0;
    }
    const auto& c = *ref_->c;
    std::size_t n = 0;
    if (c.type == NodeType::Sequence) {
      for (const auto& e : c.seq) {
        n += e->defined ? 1 : 0;
      }
    } else if (c.type == NodeType::Map) {
      for (const auto& kv : c.map) {
        n += kv.second->defined ? 1 : 0;
      }
    }
    return n;
  }

  iterator begin() const;
  iterator end() const;

  template <typename T>
  void push_back(const T& rhs) {
    push_back(Node(rhs));
  }
  void push_back(const Node& rhs) {
    EnsureNodeExists();
    rhs.EnsureNodeExists();
    auto& c = *ref_->c;
    if (c.type == NodeType::Undefined || c.type == NodeType::Null) {
      ResetContent(c);
      c.type = NodeType::Sequence;
    }
    if (c.type != NodeType::Sequence) {
      throw BadPushback();
    }
    c.seq.push_back(rhs.ref_);
    detail::mark_defined(ref_);
  }

  template <typename Key>
  bool remove(const Key& key) {
    const std::string k = KeyText(key);
    if (!IsDefined() || !ref_) {
      return false;
    }
    auto& c = *ref_->c;
    if (c.type == NodeType::Map) {
      for (auto it = c.map.begin(); it != c.map.end(); ++it) {
        if (it->first->c->type == NodeType::Scalar &&
            it->first->c->scalar == k) {
          c.map.erase(it);
          return true;
        }
      }
    } else if (c.type == NodeType::Sequence) {
      std::size_t idx = 0;
      if (ParseIndex(k, &idx) && idx < c.seq.size()) {
        c.seq.erase(c.seq.begin() + static_cast<std::ptrdiff_t>(idx));
        return true;
      }
    }
    return false;
  }

  template <typename Key, typename Value>
  void force_insert(const Key& key, const Value& value) {
    EnsureNodeExists();
    auto& c = *ref_->c;
    if (c.type == NodeType::Undefined || c.type == NodeType::Null) {
      ResetContent(c);
      c.type = NodeType::Map;
    }
    if (c.type != NodeType::Map) {
      throw BadInsert();
    }
    Node k(key);
    Node v(value);
    k.EnsureNodeExists();
    v.EnsureNodeExists();
    c.map.emplace_back(k.ref_, v.ref_);
    detail::mark_defined(ref_);
  }

  // -- subscripting ---------------------------------------------------------
  const Node operator[](const std::string& key) const {
    return GetByKey(key, false);
  }
  Node operator[](const std::string& key) {
    return GetByKey(key, true);
  }
  const Node operator[](const char* key) const {
    return GetByKey(std::string(key), false);
  }
  Node operator[](const char* key) {
    return GetByKey(std::string(key), true);
  }
  template <typename Key,
            typename = typename std::enable_if<
                std::is_integral<Key>::value &&
                !std::is_same<Key, bool>::value>::type>
  const Node operator[](Key key) const {
    return GetByIntegral(key, false);
  }
  template <typename Key,
            typename = typename std::enable_if<
                std::is_integral<Key>::value &&
                !std::is_same<Key, bool>::value>::type>
  Node operator[](Key key) {
    return GetByIntegral(key, true);
  }
  const Node operator[](const Node& key) const {
    return GetByKey(key.Scalar(), false);
  }
  Node operator[](const Node& key) {
    return GetByKey(key.Scalar(), true);
  }

 private:
  friend struct detail::NodeAccess;
  template <typename T, typename S>
  friend struct detail::as_if;

  static void ResetContent(detail::Content& c) {
    c.scalar.clear();
    c.seq.clear();
    c.map.clear();
  }

  void EnsureNodeExists() const {
    if (!valid_) {
      throw InvalidNode(invalid_key_);
    }
    if (!ref_) {
      ref_ = detail::make_ref(NodeType::Null, true);
    }
  }

  void AssignData(const Node& data) {
    EnsureNodeExists();
    data.EnsureNodeExists();
    if (ref_->c != data.ref_->c) {
      *ref_->c = *data.ref_->c;
    }
    detail::mark_defined(ref_);
  }

  void AssignNode(const Node& rhs) {
    if (!valid_) {
      throw InvalidNode(invalid_key_);
    }
    rhs.EnsureNodeExists();
    if (!ref_) {
      ref_ = rhs.ref_;
      return;
    }
    ref_->c = rhs.ref_->c;
    if (rhs.ref_->defined) {
      detail::mark_defined(ref_);
    }
  }

  static Node MakeZombie(const std::string& key) {
    Node n;
    n.valid_ = false;
    n.invalid_key_ = key;
    return n;
  }

  template <typename Key>
  static std::string KeyText(const Key& key) {
    return Node(key).Scalar();
  }
  static std::string KeyText(const std::string& key) { return key; }
  static std::string KeyText(const char* key) { return std::string(key); }

  static bool ParseIndex(const std::string& s, std::size_t* out) {
    if (s.empty() || s.size() > 18) {
      return false;
    }
    std::size_t v = 0;
    for (char ch : s) {
      if (ch < '0' || ch > '9') {
        return false;
      }
      v = v * 10 + static_cast<std::size_t>(ch - '0');
    }
    *out = v;
    return true;
  }

  template <typename Key>
  Node GetByIntegral(Key key, bool create) const {
    bool negative = false;
    if constexpr (std::is_signed<Key>::value) {
      negative = key < 0;
    }
    if (Type() == NodeType::Sequence && !negative) {
      return GetByIndex(static_cast<std::size_t>(key), create);
    }
    return GetByKey(detail::integral_text(key), create);
  }

  Node GetByIndex(std::size_t index, bool create) const {
    if (!valid_) {
      return MakeZombie(std::to_string(index));
    }
    if (!ref_) {
      return MakeZombie(std::to_string(index));
    }
    auto& c = *ref_->c;
    if (c.type != NodeType::Sequence) {
      return MakeZombie(std::to_string(index));
    }
    if (index < c.seq.size()) {
      Node n;
      n.ref_ = c.seq[index];
      return n;
    }
    if (!create) {
      return MakeZombie(std::to_string(index));
    }
    while (c.seq.size() <= index) {
      auto child = detail::make_ref(NodeType::Undefined, false);
      child->deps.push_back(ref_);
      c.seq.push_back(child);
    }
    Node n;
    n.ref_ = c.seq[index];
    return n;
  }

  Node GetByKey(const std::string& key, bool create) const {
    if (!valid_) {
      return MakeZombie(invalid_key_.empty() ? key : invalid_key_);
    }
    if (!ref_) {
      if (!create) {
        return MakeZombie(key);
      }
      EnsureNodeExists();
    }
    auto& c = *ref_->c;
    switch (c.type) {
      case NodeType::Scalar:
        if (!ref_->defined) {
          return MakeZombie(key);
        }
        throw BadSubscript(key);
      case NodeType::Sequence:
        if (!ref_->defined) {
          return MakeZombie(key);
        }
        if (!create) {
          return MakeZombie(key);
        }
        throw BadSubscript(key);
      case NodeType::Undefined:
      case NodeType::Null:
        if (!create) {
          return MakeZombie(key);
        }
        ResetContent(c);
        c.type = NodeType::Map;
        break;
      case NodeType::Map:
        break;
    }
    for (const auto& kv : c.map) {
      if (kv.first->c->type == NodeType::Scalar && kv.first->c->scalar == key) {
        Node n;
        n.ref_ = kv.second;
        return n;
      }
    }
    if (!create) {
      return MakeZombie(key);
    }
    auto k = detail::make_ref(NodeType::Scalar, true);
    k->c->scalar = key;
    auto v = detail::make_ref(NodeType::Undefined, false);
    v->deps.push_back(ref_);
    c.map.emplace_back(k, v);
    Node n;
    n.ref_ = v;
    return n;
  }

  mutable detail::RefPtr ref_;
  bool valid_ = true;
  std::string invalid_key_;
};

namespace detail {
struct NodeAccess {
  static Node wrap(const RefPtr& r) {
    Node n;
    n.ref_ = r;
    return n;
  }
  static RefPtr ref(const Node& n) {
    n.EnsureNodeExists();
    return n.ref_;
  }
  static bool valid(const Node& n) { return n.valid_; }
  static bool has_ref(const Node& n) { return static_cast<bool>(n.ref_); }
  static const std::string& invalid_key(const Node& n) { return n.invalid_key_; }
  static Node zombie(const std::string& key) { return Node::MakeZombie(key); }
};
}  // namespace detail

// ============================================================================
// Iteration
// ============================================================================

struct iterator_value : public Node {
  iterator_value() = default;
  explicit iterator_value(const Node& rhs) : Node(rhs) {}
  iterator_value(const Node& key, const Node& value)
      : Node(), first(key), second(value) {}

  Node first;
  Node second;
};

class iterator {
 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = iterator_value;
  using difference_type = std::ptrdiff_t;
  using reference = iterator_value;

  struct proxy {
    iterator_value value;
    const iterator_value* operator->() const { return &value; }
    iterator_value* operator->() { return &value; }
  };
  using pointer = proxy;

  iterator() = default;
  iterator(std::shared_ptr<detail::Content> c, std::size_t idx)
      : c_(std::move(c)), idx_(idx) {
    SkipUndefined();
  }

  iterator& operator++() {
    ++idx_;
    SkipUndefined();
    return *this;
  }
  iterator operator++(int) {
    iterator copy(*this);
    ++(*this);
    return copy;
  }

  iterator_value operator*() const {
    assert(c_);
    if (c_->type == NodeType::Map) {
      const auto& kv = c_->map.at(idx_);
      return iterator_value(detail::NodeAccess::wrap(kv.first),
                            detail::NodeAccess::wrap(kv.second));
    }
    return iterator_value(detail::NodeAccess::wrap(c_->seq.at(idx_)));
  }
  proxy operator->() const { return proxy{**this}; }

  friend bool operator==(const iterator& a, const iterator& b) {
    if (a.c_ != b.c_) {
      return false;
    }
    return !a.c_ || a.idx_ == b.idx_;
  }
  friend bool operator!=(const iterator& a, const iterator& b) {
    return !(a == b);
  }

 private:
  std::size_t Limit() const {
    if (!c_) {
      return 0;
    }
    return c_->type == NodeType::Map ? c_->map.size() : c_->seq.size();
  }
  void SkipUndefined() {
    if (!c_) {
      return;
    }
    const std::size_t limit = Limit();
    while (idx_ < limit) {
      const bool defined = (c_->type == NodeType::Map)
                               ? c_->map[idx_].second->defined
                               : c_->seq[idx_]->defined;
      if (defined) {
        break;
      }
      ++idx_;
    }
  }

  std::shared_ptr<detail::Content> c_;
  std::size_t idx_ = 0;
};

inline iterator Node::begin() const {
  if (!IsDefined() || !ref_) {
    return iterator();
  }
  const auto& c = ref_->c;
  if (c->type != NodeType::Map && c->type != NodeType::Sequence) {
    return iterator();
  }
  return iterator(c, 0);
}

inline iterator Node::end() const {
  if (!IsDefined() || !ref_) {
    return iterator();
  }
  const auto& c = ref_->c;
  if (c->type == NodeType::Map) {
    return iterator(c, c->map.size());
  }
  if (c->type == NodeType::Sequence) {
    return iterator(c, c->seq.size());
  }
  return iterator();
}

// ============================================================================
// Scalar <-> text helpers
// ============================================================================

namespace detail {

/// Shortest round-trip (or fixed precision when `precision` >= 0) text.
template <typename F>
inline std::string format_floating(F v, int precision) {
  if (std::isnan(v)) {
    return ".nan";
  }
  if (std::isinf(v)) {
    return v < 0 ? "-.inf" : ".inf";
  }
  char buf[64];
  std::to_chars_result r;
  if (precision >= 0) {
    r = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::general,
                      precision);
  } else {
    r = std::to_chars(buf, buf + sizeof(buf), v);
  }
  return std::string(buf, r.ptr);
}

/// Same, but guarantees the text still looks like a float ("1.0", not "1").
template <typename F>
inline std::string format_floating_marked(F v) {
  std::string s = format_floating(v, -1);
  if (s.find_first_of(".eEn") == std::string::npos) {
    s += ".0";
  }
  return s;
}

inline bool iequals(const std::string& s, const char* lit) {
  const std::size_t n = std::strlen(lit);
  if (s.size() != n) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (std::tolower(static_cast<unsigned char>(s[i])) != lit[i]) {
      return false;
    }
  }
  return true;
}

inline bool parse_bool(const std::string& s, bool* out) {
  static const char* kTrue[] = {"y", "yes", "true", "on"};
  static const char* kFalse[] = {"n", "no", "false", "off"};
  for (const char* t : kTrue) {
    if (iequals(s, t)) {
      *out = true;
      return true;
    }
  }
  for (const char* f : kFalse) {
    if (iequals(s, f)) {
      *out = false;
      return true;
    }
  }
  return false;
}

template <typename T>
inline bool parse_integral(const std::string& s, T* out) {
  if (s.empty() || std::isspace(static_cast<unsigned char>(s.front())) ||
      std::isspace(static_cast<unsigned char>(s.back()))) {
    return false;
  }
  std::size_t pos = 0;
  bool neg = false;
  if (s[pos] == '-' || s[pos] == '+') {
    neg = (s[pos] == '-');
    ++pos;
  }
  if (pos >= s.size()) {
    return false;
  }
  if (neg && std::is_unsigned<T>::value) {
    return false;
  }
  int base = 10;
  if (s.size() - pos > 2 && s[pos] == '0') {
    const char p = s[pos + 1];
    if (p == 'x' || p == 'X') {
      base = 16;
      pos += 2;
    } else if (p == 'o' || p == 'O') {
      base = 8;
      pos += 2;
    }
  }
  const char* begin = s.data() + pos;
  const char* end = s.data() + s.size();
  if (begin == end || *begin == '-' || *begin == '+') {
    return false;
  }
  unsigned long long mag = 0;
  const auto res = std::from_chars(begin, end, mag, base);
  if (res.ec != std::errc() || res.ptr != end) {
    return false;
  }
  if (neg) {
    // Only reachable for signed T (unsigned rejected above).
    const unsigned long long limit =
        static_cast<unsigned long long>(std::numeric_limits<T>::max()) + 1ULL;
    if (mag > limit) {
      return false;
    }
    if (mag == limit) {
      *out = std::numeric_limits<T>::min();
    } else {
      *out = static_cast<T>(-static_cast<long long>(mag));
    }
    return true;
  }
  if (mag > static_cast<unsigned long long>(std::numeric_limits<T>::max())) {
    return false;
  }
  *out = static_cast<T>(mag);
  return true;
}

template <typename F>
inline bool parse_floating(const std::string& s, F* out) {
  if (s.empty()) {
    return false;
  }
  if (s == ".inf" || s == ".Inf" || s == ".INF" || s == "+.inf" ||
      s == "+.Inf" || s == "+.INF") {
    *out = std::numeric_limits<F>::infinity();
    return true;
  }
  if (s == "-.inf" || s == "-.Inf" || s == "-.INF") {
    *out = -std::numeric_limits<F>::infinity();
    return true;
  }
  if (s == ".nan" || s == ".NaN" || s == ".NAN") {
    *out = std::numeric_limits<F>::quiet_NaN();
    return true;
  }
  if (s.find_first_not_of("+-0123456789.eE") != std::string::npos) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size()) {
    return false;
  }
  *out = static_cast<F>(v);
  return true;
}

}  // namespace detail

// ============================================================================
// convert<T>
// ============================================================================

template <>
struct convert<Node> {
  static Node encode(const Node& rhs) { return rhs; }
  static bool decode(const Node& node, Node& rhs) {
    rhs = node;
    return true;
  }
};

template <>
struct convert<std::string> {
  static Node encode(const std::string& rhs) {
    Node n;
    n.Assign(rhs);
    return n;
  }
  static bool decode(const Node& node, std::string& rhs) {
    if (!node.IsScalar()) {
      return false;
    }
    rhs = node.Scalar();
    return true;
  }
};

template <>
struct convert<std::string_view> {
  static Node encode(std::string_view rhs) {
    Node n;
    n.Assign(std::string(rhs));
    return n;
  }
};

template <>
struct convert<const char*> {
  static Node encode(const char* rhs) {
    Node n;
    n.Assign(std::string(rhs));
    return n;
  }
};

template <std::size_t N>
struct convert<char[N]> {
  static Node encode(const char* rhs) {
    Node n;
    n.Assign(std::string(rhs));
    return n;
  }
};

struct _Null {};
inline _Null Null;

template <>
struct convert<_Null> {
  static Node encode(const _Null&) { return Node(NodeType::Null); }
  static bool decode(const Node& node, _Null&) { return node.IsNull(); }
};

template <>
struct convert<bool> {
  static Node encode(bool rhs) {
    Node n;
    n.Assign(std::string(rhs ? "true" : "false"));
    return n;
  }
  static bool decode(const Node& node, bool& rhs) {
    if (!node.IsScalar()) {
      return false;
    }
    return detail::parse_bool(node.Scalar(), &rhs);
  }
};

template <typename T>
struct convert<T, typename std::enable_if<std::is_integral<T>::value &&
                                          !std::is_same<T, bool>::value>::type> {
  static Node encode(T rhs) {
    Node n;
    n.Assign(detail::integral_text(rhs));
    return n;
  }
  static bool decode(const Node& node, T& rhs) {
    if (!node.IsScalar()) {
      return false;
    }
    return detail::parse_integral(node.Scalar(), &rhs);
  }
};

template <typename T>
struct convert<T, typename std::enable_if<std::is_floating_point<T>::value>::type> {
  static Node encode(T rhs) {
    Node n;
    n.Assign(detail::format_floating_marked(rhs));
    return n;
  }
  static bool decode(const Node& node, T& rhs) {
    if (!node.IsScalar()) {
      return false;
    }
    return detail::parse_floating(node.Scalar(), &rhs);
  }
};

template <typename T, typename A>
struct convert<std::vector<T, A>> {
  static Node encode(const std::vector<T, A>& rhs) {
    Node n(NodeType::Sequence);
    for (const auto& v : rhs) {
      n.push_back(v);
    }
    return n;
  }
  static bool decode(const Node& node, std::vector<T, A>& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    rhs.clear();
    for (auto it = node.begin(); it != node.end(); ++it) {
      rhs.push_back((*it).template as<T>());
    }
    return true;
  }
};

template <typename T, typename A>
struct convert<std::list<T, A>> {
  static Node encode(const std::list<T, A>& rhs) {
    Node n(NodeType::Sequence);
    for (const auto& v : rhs) {
      n.push_back(v);
    }
    return n;
  }
  static bool decode(const Node& node, std::list<T, A>& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    rhs.clear();
    for (auto it = node.begin(); it != node.end(); ++it) {
      rhs.push_back((*it).template as<T>());
    }
    return true;
  }
};

template <typename T, typename C, typename A>
struct convert<std::set<T, C, A>> {
  static Node encode(const std::set<T, C, A>& rhs) {
    Node n(NodeType::Sequence);
    for (const auto& v : rhs) {
      n.push_back(v);
    }
    return n;
  }
  static bool decode(const Node& node, std::set<T, C, A>& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    rhs.clear();
    for (auto it = node.begin(); it != node.end(); ++it) {
      rhs.insert((*it).template as<T>());
    }
    return true;
  }
};

template <typename T, std::size_t N>
struct convert<std::array<T, N>> {
  static Node encode(const std::array<T, N>& rhs) {
    Node n(NodeType::Sequence);
    for (const auto& v : rhs) {
      n.push_back(v);
    }
    return n;
  }
  static bool decode(const Node& node, std::array<T, N>& rhs) {
    if (!node.IsSequence() || node.size() != N) {
      return false;
    }
    std::size_t i = 0;
    for (auto it = node.begin(); it != node.end(); ++it) {
      rhs[i++] = (*it).template as<T>();
    }
    return true;
  }
};

template <typename K, typename V, typename C, typename A>
struct convert<std::map<K, V, C, A>> {
  static Node encode(const std::map<K, V, C, A>& rhs) {
    Node n(NodeType::Map);
    for (const auto& kv : rhs) {
      n.force_insert(kv.first, kv.second);
    }
    return n;
  }
  static bool decode(const Node& node, std::map<K, V, C, A>& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    rhs.clear();
    for (auto it = node.begin(); it != node.end(); ++it) {
      rhs[it->first.template as<K>()] = it->second.template as<V>();
    }
    return true;
  }
};

template <typename T, typename U>
struct convert<std::pair<T, U>> {
  static Node encode(const std::pair<T, U>& rhs) {
    Node n(NodeType::Sequence);
    n.push_back(rhs.first);
    n.push_back(rhs.second);
    return n;
  }
  static bool decode(const Node& node, std::pair<T, U>& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.first = node[0].template as<T>();
    rhs.second = node[1].template as<U>();
    return true;
  }
};

// ============================================================================
// Node::as<T>
// ============================================================================

namespace detail {

template <typename T, typename S>
struct as_if {
  explicit as_if(const Node& n) : node(n) {}
  const Node& node;

  T operator()(const S& fallback) const {
    if (!NodeAccess::valid(node)) {
      return fallback;
    }
    T t;
    if (convert<T>::decode(node, t)) {
      return t;
    }
    return fallback;
  }
};

template <typename S>
struct as_if<std::string, S> {
  explicit as_if(const Node& n) : node(n) {}
  const Node& node;

  std::string operator()(const S& fallback) const {
    if (node.Type() != NodeType::Scalar) {
      return fallback;
    }
    return node.Scalar();
  }
};

template <typename T>
struct as_if<T, void> {
  explicit as_if(const Node& n) : node(n) {}
  const Node& node;

  T operator()() const {
    if (!NodeAccess::valid(node)) {
      throw InvalidNode(NodeAccess::invalid_key(node));
    }
    T t;
    if (convert<T>::decode(node, t)) {
      return t;
    }
    throw TypedBadConversion<T>(
        node.Mark(),
        node.IsScalar() ? "cannot convert '" + node.Scalar() + "'"
                        : std::string());
  }
};

template <>
struct as_if<std::string, void> {
  explicit as_if(const Node& n) : node(n) {}
  const Node& node;

  std::string operator()() const {
    if (!NodeAccess::valid(node)) {
      throw InvalidNode(NodeAccess::invalid_key(node));
    }
    if (node.Type() == NodeType::Null) {
      return "null";
    }
    if (node.Type() != NodeType::Scalar) {
      throw TypedBadConversion<std::string>(node.Mark());
    }
    return node.Scalar();
  }
};

}  // namespace detail

template <typename T>
inline T Node::as() const {
  return detail::as_if<T, void>(*this)();
}

template <typename T, typename S>
inline T Node::as(const S& fallback) const {
  return detail::as_if<T, S>(*this)(fallback);
}

// ============================================================================
// Loading (fkYAML backend)
// ============================================================================

namespace detail {

/// fkYAML node type with insertion-ordered mappings (matches yaml-cpp order).
using FkNode = fkyaml::basic_node<std::vector, fkyaml::ordered_map>;

inline RefPtr from_fk(const FkNode& n) {
  switch (n.get_type()) {
    case fkyaml::node_type::NULL_OBJECT:
      return make_ref(NodeType::Null, true);
    case fkyaml::node_type::BOOLEAN: {
      auto r = make_ref(NodeType::Scalar, true);
      r->c->scalar = n.as_bool() ? "true" : "false";
      return r;
    }
    case fkyaml::node_type::INTEGER: {
      auto r = make_ref(NodeType::Scalar, true);
      r->c->scalar = std::to_string(static_cast<long long>(n.as_int()));
      return r;
    }
    case fkyaml::node_type::FLOAT: {
      auto r = make_ref(NodeType::Scalar, true);
      r->c->scalar = format_floating_marked(n.as_float());
      return r;
    }
    case fkyaml::node_type::STRING: {
      auto r = make_ref(NodeType::Scalar, true);
      r->c->scalar = n.as_str();
      return r;
    }
    case fkyaml::node_type::SEQUENCE: {
      auto r = make_ref(NodeType::Sequence, true);
      for (const auto& child : n.as_seq()) {
        r->c->seq.push_back(from_fk(child));
      }
      return r;
    }
    case fkyaml::node_type::MAPPING: {
      auto r = make_ref(NodeType::Map, true);
      for (const auto& kv : n.as_map()) {
        r->c->map.emplace_back(from_fk(kv.first), from_fk(kv.second));
      }
      return r;
    }
  }
  return make_ref(NodeType::Null, true);
}

inline Node parse_document(const std::string& text) {
  try {
    const FkNode doc = FkNode::deserialize(text);
    return NodeAccess::wrap(from_fk(doc));
  } catch (const Exception&) {
    throw;
  } catch (const std::exception& e) {
    throw ParserException(Mark::null_mark(), e.what());
  }
}

inline std::vector<Node> parse_documents(const std::string& text) {
  try {
    const std::vector<FkNode> docs = FkNode::deserialize_docs(text);
    std::vector<Node> out;
    out.reserve(docs.size());
    for (const auto& d : docs) {
      out.push_back(NodeAccess::wrap(from_fk(d)));
    }
    return out;
  } catch (const Exception&) {
    throw;
  } catch (const std::exception& e) {
    throw ParserException(Mark::null_mark(), e.what());
  }
}

inline std::string slurp(std::istream& in) {
  return std::string(std::istreambuf_iterator<char>(in),
                     std::istreambuf_iterator<char>());
}

inline std::string slurp_file(const std::string& filename) {
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (!in) {
    throw BadFile(filename);
  }
  return slurp(in);
}

}  // namespace detail

inline Node Load(const std::string& input) {
  return detail::parse_document(input);
}
inline Node Load(const char* input) {
  return detail::parse_document(std::string(input));
}
inline Node Load(std::istream& input) {
  return detail::parse_document(detail::slurp(input));
}
inline Node LoadFile(const std::string& filename) {
  return detail::parse_document(detail::slurp_file(filename));
}
inline std::vector<Node> LoadAll(const std::string& input) {
  return detail::parse_documents(input);
}
inline std::vector<Node> LoadAll(const char* input) {
  return detail::parse_documents(std::string(input));
}
inline std::vector<Node> LoadAll(std::istream& input) {
  return detail::parse_documents(detail::slurp(input));
}
inline std::vector<Node> LoadAllFromFile(const std::string& filename) {
  return detail::parse_documents(detail::slurp_file(filename));
}

// ============================================================================
// Emitter
// ============================================================================

enum EMITTER_MANIP {
  Auto,
  Newline,
  DoubleQuoted,
  SingleQuoted,
  Literal,
  Flow,
  Block,
  Key,
  Value,
  BeginSeq,
  EndSeq,
  BeginMap,
  EndMap
};

struct _Precision {
  _Precision(int fp, int dp) : floatPrecision(fp), doublePrecision(dp) {}
  int floatPrecision;
  int doublePrecision;
};
inline _Precision FloatPrecision(std::size_t n) {
  return _Precision(static_cast<int>(n), -1);
}
inline _Precision DoublePrecision(std::size_t n) {
  return _Precision(-1, static_cast<int>(n));
}
inline _Precision Precision(std::size_t n) {
  return _Precision(static_cast<int>(n), static_cast<int>(n));
}

namespace detail {

struct ENode {
  enum Kind { kScalar, kNull, kSeq, kMap };
  enum Quote { kAuto, kDouble, kSingle };

  explicit ENode(Kind k) : kind(k) {}
  Kind kind;
  Quote quote = kAuto;
  bool flow = false;
  std::string text;
  std::vector<std::unique_ptr<ENode>> kids;
};

inline bool needs_quotes(const std::string& s, bool flow) {
  if (s.empty()) {
    return true;
  }
  if (s.front() == ' ' || s.front() == '\t' || s.back() == ' ' ||
      s.back() == '\t') {
    return true;
  }
  for (char ch : s) {
    const unsigned char u = static_cast<unsigned char>(ch);
    if (u < 0x20 || u == 0x7f) {
      return true;
    }
    if (flow && (ch == ',' || ch == '[' || ch == ']' || ch == '{' ||
                 ch == '}')) {
      return true;
    }
  }
  const char first = s.front();
  switch (first) {
    case ',': case '[': case ']': case '{': case '}': case '#': case '&':
    case '*': case '!': case '|': case '>': case '\'': case '"': case '%':
    case '@': case '`':
      return true;
    case '-': case '?': case ':':
      if (s.size() == 1 || s[1] == ' ') {
        return true;
      }
      break;
    default:
      break;
  }
  if (s.back() == ':') {
    return true;
  }
  if (s.find(": ") != std::string::npos || s.find(" #") != std::string::npos) {
    return true;
  }
  if (s == "---" || s == "...") {
    return true;
  }
  return false;
}

inline std::string double_quote(const std::string& s) {
  std::string out = "\"";
  for (char ch : s) {
    switch (ch) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      case '\r': out += "\\r"; break;
      case '\0': out += "\\0"; break;
      default: {
        const unsigned char u = static_cast<unsigned char>(ch);
        if (u < 0x20 || u == 0x7f) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\x%02X", u);
          out += buf;
        } else {
          out += ch;
        }
      }
    }
  }
  out += '"';
  return out;
}

inline std::string single_quote(const std::string& s) {
  for (char ch : s) {
    const unsigned char u = static_cast<unsigned char>(ch);
    if (u < 0x20 || u == 0x7f) {
      return double_quote(s);
    }
  }
  std::string out = "'";
  for (char ch : s) {
    if (ch == '\'') {
      out += "''";
    } else {
      out += ch;
    }
  }
  out += '\'';
  return out;
}

inline std::string scalar_text(const ENode& n, bool flow) {
  if (n.kind == ENode::kNull) {
    return "~";
  }
  if (n.quote == ENode::kDouble) {
    return double_quote(n.text);
  }
  if (n.quote == ENode::kSingle) {
    return single_quote(n.text);
  }
  if (needs_quotes(n.text, flow)) {
    return double_quote(n.text);
  }
  return n.text;
}

inline bool is_block(const ENode& n) {
  return (n.kind == ENode::kSeq || n.kind == ENode::kMap) && !n.flow &&
         !n.kids.empty();
}

inline std::string flow_text(const ENode& n, bool in_flow) {
  switch (n.kind) {
    case ENode::kScalar:
    case ENode::kNull:
      return scalar_text(n, in_flow);
    case ENode::kSeq: {
      if (n.kids.empty()) {
        return "[]";
      }
      std::string out = "[";
      for (std::size_t i = 0; i < n.kids.size(); ++i) {
        if (i > 0) {
          out += ", ";
        }
        out += flow_text(*n.kids[i], true);
      }
      out += "]";
      return out;
    }
    case ENode::kMap: {
      if (n.kids.empty()) {
        return "{}";
      }
      std::string out = "{";
      for (std::size_t i = 0; i < n.kids.size(); i += 2) {
        if (i > 0) {
          out += ", ";
        }
        out += flow_text(*n.kids[i], true);
        out += ": ";
        if (i + 1 < n.kids.size()) {
          out += flow_text(*n.kids[i + 1], true);
        } else {
          out += "~";
        }
      }
      out += "}";
      return out;
    }
  }
  return std::string();
}

inline void write_block(const ENode& n, int indent, bool first_inline,
                        int width, std::string& out) {
  const ENode null_node(ENode::kNull);
  if (n.kind == ENode::kSeq) {
    for (std::size_t i = 0; i < n.kids.size(); ++i) {
      if (!(i == 0 && first_inline)) {
        out.append(static_cast<std::size_t>(indent), ' ');
      }
      out += '-';
      const ENode& v = *n.kids[i];
      if (is_block(v)) {
        out += ' ';
        write_block(v, indent + 2, true, width, out);
      } else {
        out += ' ';
        out += flow_text(v, false);
        out += '\n';
      }
    }
    return;
  }
  for (std::size_t i = 0; i < n.kids.size(); i += 2) {
    if (!(i == 0 && first_inline)) {
      out.append(static_cast<std::size_t>(indent), ' ');
    }
    out += flow_text(*n.kids[i], false);
    out += ':';
    const ENode& v = (i + 1 < n.kids.size()) ? *n.kids[i + 1] : null_node;
    if (is_block(v)) {
      out += '\n';
      write_block(v, indent + width, false, width, out);
    } else {
      out += ' ';
      out += flow_text(v, false);
      out += '\n';
    }
  }
}

inline void write_document(const ENode& n, int width, std::string& out) {
  if (is_block(n)) {
    write_block(n, 0, false, width, out);
  } else {
    out += flow_text(n, false);
    out += '\n';
  }
}

inline void set_flow(ENode& n) {
  n.flow = true;
  for (auto& k : n.kids) {
    set_flow(*k);
  }
}

inline std::unique_ptr<ENode> to_enode(const Node& node) {
  switch (node.Type()) {
    case NodeType::Scalar: {
      auto e = std::make_unique<ENode>(ENode::kScalar);
      e->text = node.Scalar();
      return e;
    }
    case NodeType::Sequence: {
      auto e = std::make_unique<ENode>(ENode::kSeq);
      for (auto it = node.begin(); it != node.end(); ++it) {
        e->kids.push_back(to_enode(*it));
      }
      return e;
    }
    case NodeType::Map: {
      auto e = std::make_unique<ENode>(ENode::kMap);
      for (auto it = node.begin(); it != node.end(); ++it) {
        e->kids.push_back(to_enode(it->first));
        e->kids.push_back(to_enode(it->second));
      }
      return e;
    }
    case NodeType::Null:
    case NodeType::Undefined:
    default:
      return std::make_unique<ENode>(ENode::kNull);
  }
}

}  // namespace detail

class Emitter {
 public:
  Emitter() = default;
  Emitter(const Emitter&) = delete;
  Emitter& operator=(const Emitter&) = delete;

  // -- output ---------------------------------------------------------------
  const char* c_str() const {
    Render();
    return out_.c_str();
  }
  std::size_t size() const {
    Render();
    return out_.size();
  }
  bool good() const { return true; }
  std::string GetLastError() const { return std::string(); }

  // -- settings -------------------------------------------------------------
  bool SetIndent(std::size_t n) {
    if (n < 2) {
      return false;
    }
    indent_ = static_cast<int>(n);
    return true;
  }
  bool SetFloatPrecision(std::size_t n) {
    float_precision_ = static_cast<int>(n);
    return true;
  }
  bool SetDoublePrecision(std::size_t n) {
    double_precision_ = static_cast<int>(n);
    return true;
  }
  bool SetMapFormat(EMITTER_MANIP value) {
    if (value == Flow) {
      map_flow_ = true;
    } else if (value == Block) {
      map_flow_ = false;
    } else {
      return false;
    }
    return true;
  }
  bool SetSeqFormat(EMITTER_MANIP value) {
    if (value == Flow) {
      seq_flow_ = true;
    } else if (value == Block) {
      seq_flow_ = false;
    } else {
      return false;
    }
    return true;
  }

  // -- writing --------------------------------------------------------------
  Emitter& SetLocalValue(EMITTER_MANIP value) {
    switch (value) {
      case BeginSeq:
        Begin(detail::ENode::kSeq);
        break;
      case BeginMap:
        Begin(detail::ENode::kMap);
        break;
      case EndSeq:
      case EndMap:
        End();
        break;
      case Key:
        if (!stack_.empty() && stack_.back()->kind == detail::ENode::kMap &&
            stack_.back()->kids.size() % 2 == 1) {
          stack_.back()->kids.push_back(
              std::make_unique<detail::ENode>(detail::ENode::kNull));
        }
        break;
      case Flow:
        pending_flow_ = 1;
        break;
      case Block:
        pending_flow_ = 0;
        break;
      case DoubleQuoted:
        pending_quote_ = detail::ENode::kDouble;
        break;
      case SingleQuoted:
        pending_quote_ = detail::ENode::kSingle;
        break;
      case Value:
      case Auto:
      case Newline:
      case Literal:
      default:
        break;
    }
    return *this;
  }
  Emitter& SetLocalPrecision(const _Precision& p) {
    if (p.floatPrecision >= 0) {
      float_precision_ = p.floatPrecision;
    }
    if (p.doublePrecision >= 0) {
      double_precision_ = p.doublePrecision;
    }
    return *this;
  }

  Emitter& Write(const std::string& s) {
    auto e = std::make_unique<detail::ENode>(detail::ENode::kScalar);
    e->text = s;
    e->quote = pending_quote_;
    pending_quote_ = detail::ENode::kAuto;
    Add(std::move(e));
    return *this;
  }
  Emitter& Write(const char* s) { return Write(std::string(s)); }
  Emitter& Write(char c) { return Write(std::string(1, c)); }
  Emitter& Write(bool b) { return Write(std::string(b ? "true" : "false")); }
  Emitter& Write(const _Null&) {
    Add(std::make_unique<detail::ENode>(detail::ENode::kNull));
    return *this;
  }
  Emitter& Write(const Node& node) {
    auto e = detail::to_enode(node);
    if (pending_flow_ == 1) {
      detail::set_flow(*e);
    }
    pending_flow_ = -1;
    Add(std::move(e));
    return *this;
  }
  Emitter& Write(float v) {
    return WritePlain(detail::format_floating(v, float_precision_));
  }
  Emitter& Write(double v) {
    return WritePlain(detail::format_floating(v, double_precision_));
  }
  Emitter& Write(long double v) {
    return Write(static_cast<double>(v));
  }
  template <typename T>
  typename std::enable_if<std::is_integral<T>::value &&
                              !std::is_same<T, bool>::value &&
                              !std::is_same<T, char>::value,
                          Emitter&>::type
  Write(T v) {
    return WritePlain(detail::integral_text(v));
  }

 private:
  Emitter& WritePlain(const std::string& text) {
    auto e = std::make_unique<detail::ENode>(detail::ENode::kScalar);
    e->text = text;
    pending_quote_ = detail::ENode::kAuto;
    Add(std::move(e));
    return *this;
  }

  void Add(std::unique_ptr<detail::ENode> e) {
    pending_flow_ = -1;
    if (stack_.empty()) {
      roots_.push_back(std::move(e));
    } else {
      stack_.back()->kids.push_back(std::move(e));
    }
    dirty_ = true;
  }

  void Begin(detail::ENode::Kind kind) {
    auto e = std::make_unique<detail::ENode>(kind);
    bool flow = (kind == detail::ENode::kMap) ? map_flow_ : seq_flow_;
    if (pending_flow_ >= 0) {
      flow = pending_flow_ == 1;
    }
    pending_flow_ = -1;
    if (!stack_.empty() && stack_.back()->flow) {
      flow = true;
    }
    e->flow = flow;
    detail::ENode* raw = e.get();
    Add(std::move(e));
    stack_.push_back(raw);
  }

  void End() {
    if (!stack_.empty()) {
      stack_.pop_back();
    }
    dirty_ = true;
  }

  void Render() const {
    if (!dirty_) {
      return;
    }
    out_.clear();
    for (std::size_t i = 0; i < roots_.size(); ++i) {
      if (i > 0) {
        out_ += "---\n";
      }
      detail::write_document(*roots_[i], indent_, out_);
    }
    while (!out_.empty() && out_.back() == '\n') {
      out_.pop_back();
    }
    dirty_ = false;
  }

  std::vector<std::unique_ptr<detail::ENode>> roots_;
  std::vector<detail::ENode*> stack_;
  mutable std::string out_;
  mutable bool dirty_ = true;
  int indent_ = 2;
  int float_precision_ = -1;
  int double_precision_ = -1;
  bool map_flow_ = false;
  bool seq_flow_ = false;
  int pending_flow_ = -1;  // -1 unset, 0 block, 1 flow
  detail::ENode::Quote pending_quote_ = detail::ENode::kAuto;
};

// -- operator<< ---------------------------------------------------------------

inline Emitter& operator<<(Emitter& out, EMITTER_MANIP value) {
  return out.SetLocalValue(value);
}
inline Emitter& operator<<(Emitter& out, const _Precision& p) {
  return out.SetLocalPrecision(p);
}
inline Emitter& operator<<(Emitter& out, const std::string& v) {
  return out.Write(v);
}
inline Emitter& operator<<(Emitter& out, std::string_view v) {
  return out.Write(std::string(v));
}
inline Emitter& operator<<(Emitter& out, const char* v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, char v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, bool v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, float v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, double v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, long double v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, const _Null& v) { return out.Write(v); }
inline Emitter& operator<<(Emitter& out, const Node& v) { return out.Write(v); }
template <typename T>
inline typename std::enable_if<std::is_integral<T>::value &&
                                   !std::is_same<T, bool>::value &&
                                   !std::is_same<T, char>::value,
                               Emitter&>::type
operator<<(Emitter& out, T v) {
  return out.Write(v);
}

template <typename Seq>
inline Emitter& EmitSeq(Emitter& emitter, const Seq& seq) {
  emitter << BeginSeq;
  for (const auto& v : seq) {
    emitter << v;
  }
  emitter << EndSeq;
  return emitter;
}
template <typename T, typename A>
inline Emitter& operator<<(Emitter& emitter, const std::vector<T, A>& v) {
  return EmitSeq(emitter, v);
}
template <typename T, typename A>
inline Emitter& operator<<(Emitter& emitter, const std::list<T, A>& v) {
  return EmitSeq(emitter, v);
}
template <typename T, std::size_t N>
inline Emitter& operator<<(Emitter& emitter, const std::array<T, N>& v) {
  return EmitSeq(emitter, v);
}
template <typename K, typename V, typename C, typename A>
inline Emitter& operator<<(Emitter& emitter, const std::map<K, V, C, A>& m) {
  emitter << BeginMap;
  for (const auto& kv : m) {
    emitter << Key << kv.first << Value << kv.second;
  }
  emitter << EndMap;
  return emitter;
}

inline std::ostream& operator<<(std::ostream& out, const Emitter& e) {
  return out << e.c_str();
}

/// Serialises a node to YAML text (block style, no trailing newline).
inline std::string Dump(const Node& node) {
  Emitter e;
  e << node;
  return e.c_str();
}

inline std::ostream& operator<<(std::ostream& out, const Node& node) {
  return out << Dump(node);
}

}  // namespace YAML

#endif  // AUTONOMY_COMMON_YAML_HPP_
