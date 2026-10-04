/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file message_path_navigation.hpp
 * @brief Parse and resolve Foxglove-style message paths on protobuf messages.
 *
 * Supports dotted fields, repeated indices (@c [n] / @c [:]), and element
 * filters (@c {field==value}). Used by plot extraction, indicators, and Raw
 * Messages formatting.
 *
 * @see PlotFieldExtractor
 * @see message_field_tree.hpp
 */

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

/**
 * @struct MessagePathFilter
 * @brief Element filter attached to a repeated-field path segment.
 *
 * Example path fragment: @c detections{id==3} → field @c id, op @c ==,
 * value @c 3.
 */
struct MessagePathFilter {
  std::string field;  /**< Element sub-field to compare. */
  std::string op;     /**< Comparison operator (e.g. @c ==, @c !=). */
  std::string value;  /**< Right-hand side literal as text. */
};

/**
 * @struct MessagePathSegment
 * @brief One segment of a parsed dotted message path.
 */
struct MessagePathSegment {
  std::string field;  /**< Field name for this segment. */
  /**
   * Empty index with @c has_bracket=@c true means @c [:] (all elements).
   */
  bool has_bracket = false;   /**< Whether @c [..] was present. */
  bool all_elements = false;  /**< @c [:] — iterate all repeated elements. */
  int index = 0;              /**< Single index when not @c all_elements. */
  std::optional<MessagePathFilter> filter;  /**< Optional @c {..} filter. */
};

/**
 * @brief Split a dotted path into segments, parsing @c [index] and @c {filter}.
 *
 * @param path Full field path string.
 * @return Ordered segments from root to leaf.
 */
std::vector<MessagePathSegment> ParseMessagePath(const std::string& path);

/**
 * @brief Resolve a numeric leaf field, including repeated-field index/filter.
 *
 * @param message Root protobuf message.
 * @param field_path Path to a numeric scalar leaf.
 * @param value Out-parameter for the extracted number.
 * @return @c true when a numeric value was written to @p value.
 */
bool ExtractNumericByMessagePath(const google::protobuf::Message& message,
                                 const std::string& field_path, double* value);

/**
 * @brief Extract every numeric leaf matched by @c [:] / @c [] expansion.
 *
 * Paths without expand still yield a single value (same as
 * @ref ExtractNumericByMessagePath). Supports repeated message children
 * (@c points[:].x) and repeated numeric leaves (@c data[:]).
 *
 * @param message Root protobuf message.
 * @param field_path Path that may contain @c [:].
 * @param values Out: appended numeric samples (cleared by the caller if desired).
 * @return @c true when at least one value was appended.
 */
bool ExtractAllNumericsByMessagePath(const google::protobuf::Message& message,
                                     const std::string& field_path,
                                     std::vector<double>* values);

/**
 * @brief Compute L2 norm of a vector-like message at @p field_path.
 *
 * Reads optional @c x / @c y / @c z numeric children (at least one required).
 *
 * @param message Root protobuf message.
 * @param field_path Path to a message containing vector components.
 * @param value Out: sqrt(x²+y²+z²).
 * @return @c true when a finite norm was written.
 */
bool ExtractVectorNormByMessagePath(const google::protobuf::Message& message,
                                    const std::string& field_path,
                                    double* value);

/**
 * @brief Resolve a scalar leaf field for string/boolean/numeric indicator values.
 *
 * @param message Root protobuf message.
 * @param field_path Path to the leaf.
 * @param container Out: message that owns the leaf field.
 * @param leaf_field Out: descriptor of the leaf field.
 * @return @c true when the leaf was resolved.
 */
bool ResolveLeafFieldByMessagePath(
    const google::protobuf::Message& message, const std::string& field_path,
    const google::protobuf::Message** container,
    const google::protobuf::FieldDescriptor** leaf_field);

/**
 * @struct ResolvedRepeatedField
 * @brief Result of resolving a path whose final segment is repeated.
 */
struct ResolvedRepeatedField {
  const google::protobuf::Message* container = nullptr;  /**< Owning message. */
  const google::protobuf::FieldDescriptor* repeated_field = nullptr;  /**< Repeated field. */
  std::optional<MessagePathFilter> element_filter;  /**< Filter from the path. */
  bool use_single_index = false;  /**< When @c true, only @c single_index applies. */
  int single_index = 0;           /**< Index when @c use_single_index is set. */
};

/**
 * @brief Resolve a path whose last segment is a repeated protobuf field.
 *
 * @param message Root protobuf message.
 * @param field_path Path ending at a repeated field.
 * @param resolved Out-parameter filled on success.
 * @return @c true when @p resolved was populated.
 */
bool ResolveRepeatedFieldPath(const google::protobuf::Message& message,
                              const std::string& field_path,
                              ResolvedRepeatedField* resolved);

/**
 * @brief Format the protobuf value at a message path for display (Raw Messages).
 *
 * @param message Root protobuf message.
 * @param field_path Path to format.
 * @return Formatted string, or @c std::nullopt if unresolved.
 */
std::optional<std::string> FormatMessagePathValue(
    const google::protobuf::Message& message, const std::string& field_path);

/**
 * @brief Returns @c true when an array element satisfies a path filter.
 *
 * @param element Repeated-field element message.
 * @param filter Filter expression from the path.
 * @return Whether the element matches.
 */
bool ElementMatchesPathFilter(const google::protobuf::Message& element,
                              const MessagePathFilter& filter);

}  // namespace plot
}  // namespace autoviz
