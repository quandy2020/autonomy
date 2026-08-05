/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace google {
namespace protobuf {
class FieldDescriptor;
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace plot {

struct MessagePathFilter {
  std::string field;
  std::string op;
  std::string value;
};

struct MessagePathSegment {
  std::string field;
  /** Empty index with has_bracket=true means [:] (all elements). */
  bool has_bracket = false;
  bool all_elements = false;
  int index = 0;
  std::optional<MessagePathFilter> filter;
};

/** Split a dotted path into segments, parsing [index] and {filter} suffixes. */
std::vector<MessagePathSegment> ParseMessagePath(const std::string& path);

/** Resolve a numeric leaf field, including repeated-field index/filter syntax. */
bool ExtractNumericByMessagePath(const google::protobuf::Message& message,
                                 const std::string& field_path, double* value);

/** Resolve a scalar leaf field for string/boolean/numeric indicator values. */
bool ResolveLeafFieldByMessagePath(
    const google::protobuf::Message& message, const std::string& field_path,
    const google::protobuf::Message** container,
    const google::protobuf::FieldDescriptor** leaf_field);

struct ResolvedRepeatedField {
  const google::protobuf::Message* container = nullptr;
  const google::protobuf::FieldDescriptor* repeated_field = nullptr;
  std::optional<MessagePathFilter> element_filter;
  bool use_single_index = false;
  int single_index = 0;
};

/** Resolve a path whose last segment is a repeated protobuf field. */
bool ResolveRepeatedFieldPath(const google::protobuf::Message& message,
                              const std::string& field_path,
                              ResolvedRepeatedField* resolved);

/** Format the protobuf value at a message path for display (Raw Messages). */
std::optional<std::string> FormatMessagePathValue(
    const google::protobuf::Message& message, const std::string& field_path);

/** Returns true when an array element satisfies a path filter expression. */
bool ElementMatchesPathFilter(const google::protobuf::Message& element,
                              const MessagePathFilter& filter);

}  // namespace plot
}  // namespace autoviz
