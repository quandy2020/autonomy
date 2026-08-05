/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/message_path_navigation.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

namespace autoviz {
namespace plot {
namespace {

double ReadNumericField(const google::protobuf::Message& message,
                        const google::protobuf::FieldDescriptor* field) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return reflection->GetDouble(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return reflection->GetFloat(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return reflection->GetInt32(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return static_cast<double>(reflection->GetInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return reflection->GetUInt32(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return static_cast<double>(reflection->GetUInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetBool(message, field) ? 1.0 : 0.0;
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return std::strtod(reflection->GetString(message, field).c_str(), nullptr);
    default:
      return 0.0;
  }
}

std::string ReadStringField(const google::protobuf::Message& message,
                            const google::protobuf::FieldDescriptor* field) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return reflection->GetString(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetBool(message, field) ? "true" : "false";
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return std::to_string(reflection->GetDouble(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return std::to_string(reflection->GetFloat(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return std::to_string(reflection->GetInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return std::to_string(reflection->GetInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return std::to_string(reflection->GetUInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return std::to_string(reflection->GetUInt64(message, field));
    default:
      return {};
  }
}

bool CompareFilterValues(const std::string& lhs, const std::string& op,
                         const std::string& rhs) {
  char* lhs_end = nullptr;
  char* rhs_end = nullptr;
  const double lhs_num = std::strtod(lhs.c_str(), &lhs_end);
  const double rhs_num = std::strtod(rhs.c_str(), &rhs_end);
  const bool numeric = lhs_end != lhs.c_str() && rhs_end != rhs.c_str();
  if (numeric) {
    if (op == "==") {
      return std::abs(lhs_num - rhs_num) < 1e-9;
    }
    if (op == "!=") {
      return std::abs(lhs_num - rhs_num) >= 1e-9;
    }
    if (op == "<") {
      return lhs_num < rhs_num;
    }
    if (op == ">") {
      return lhs_num > rhs_num;
    }
    if (op == "<=") {
      return lhs_num <= rhs_num;
    }
    if (op == ">=") {
      return lhs_num >= rhs_num;
    }
    return false;
  }
  if (op == "==") {
    return lhs == rhs;
  }
  if (op == "!=") {
    return lhs != rhs;
  }
  return false;
}

bool MatchesFilter(const google::protobuf::Message& element,
                   const MessagePathFilter& filter) {
  const google::protobuf::Descriptor* desc = element.GetDescriptor();
  const google::protobuf::FieldDescriptor* field =
      desc->FindFieldByName(filter.field);
  if (field == nullptr || field->is_repeated()) {
    return false;
  }
  const std::string actual = ReadStringField(element, field);
  return CompareFilterValues(actual, filter.op, filter.value);
}

std::optional<MessagePathFilter> ParseFilterSuffix(const std::string& text) {
  if (text.empty() || text.front() != '{' || text.back() != '}') {
    return std::nullopt;
  }
  const std::string inner = text.substr(1, text.size() - 2);
  static const char* kOps[] = {"==", "!=", "<=", ">=", "<", ">"};
  for (const char* op : kOps) {
    const std::size_t pos = inner.find(op);
    if (pos == std::string::npos) {
      continue;
    }
    MessagePathFilter filter;
    filter.field = inner.substr(0, pos);
    filter.op = op;
    filter.value = inner.substr(pos + std::char_traits<char>::length(op));
    if (filter.field.empty()) {
      return std::nullopt;
    }
    return filter;
  }
  return std::nullopt;
}

MessagePathSegment ParseSegment(const std::string& raw) {
  MessagePathSegment segment;
  const std::size_t bracket = raw.find('[');
  const std::size_t filter_pos = raw.find('{');
  const std::size_t suffix_pos =
      std::min(bracket == std::string::npos ? raw.size() : bracket,
               filter_pos == std::string::npos ? raw.size() : filter_pos);
  segment.field = raw.substr(0, suffix_pos);
  if (segment.field.empty()) {
    return segment;
  }

  std::string suffix = raw.substr(suffix_pos);
  if (!suffix.empty() && suffix.front() == '[') {
    const std::size_t close = suffix.find(']');
    if (close != std::string::npos) {
      segment.has_bracket = true;
      const std::string index_text = suffix.substr(1, close - 1);
      segment.all_elements = index_text.empty() || index_text == ":";
      if (!segment.all_elements) {
        segment.index = std::atoi(index_text.c_str());
      }
      suffix = suffix.substr(close + 1);
    }
  }
  if (!suffix.empty() && suffix.front() == '{') {
    segment.filter = ParseFilterSuffix(suffix);
  }
  return segment;
}

bool ExtractNumericLeaf(const google::protobuf::Message& message,
                        const google::protobuf::FieldDescriptor* field,
                        double* value) {
  if (field == nullptr || field->is_repeated() ||
      field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  *value = ReadNumericField(message, field);
  return std::isfinite(*value);
}

bool NavigateRepeated(const google::protobuf::Message& parent,
                      const google::protobuf::FieldDescriptor* field,
                      const MessagePathSegment& segment,
                      const google::protobuf::Message** selected) {
  if (field == nullptr || !field->is_repeated() ||
      field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  const google::protobuf::Reflection* reflection = parent.GetReflection();
  const int count = reflection->FieldSize(parent, field);
  if (count <= 0) {
    return false;
  }

  if (segment.has_bracket && !segment.all_elements && !segment.filter) {
    if (segment.index < 0 || segment.index >= count) {
      return false;
    }
    *selected = &reflection->GetRepeatedMessage(parent, field, segment.index);
    return true;
  }

  if (segment.filter.has_value()) {
    for (int i = 0; i < count; ++i) {
      const google::protobuf::Message& candidate =
          reflection->GetRepeatedMessage(parent, field, i);
      if (MatchesFilter(candidate, *segment.filter)) {
        *selected = &candidate;
        return true;
      }
    }
    return false;
  }

  *selected = &reflection->GetRepeatedMessage(parent, field, 0);
  return true;
}

bool ExtractNumericFromSegments(const google::protobuf::Message& message,
                                const std::vector<MessagePathSegment>& segments,
                                std::size_t index, double* value) {
  if (index >= segments.size()) {
    return false;
  }
  const MessagePathSegment& segment = segments[index];
  const google::protobuf::Descriptor* desc = message.GetDescriptor();
  const google::protobuf::FieldDescriptor* field =
      desc->FindFieldByName(segment.field);
  if (field == nullptr) {
    return false;
  }

  const bool is_last = index + 1 >= segments.size();
  if (field->is_repeated()) {
    const google::protobuf::Message* selected = nullptr;
    if (!NavigateRepeated(message, field, segment, &selected) ||
        selected == nullptr) {
      return false;
    }
    if (is_last) {
      return false;
    }
    return ExtractNumericFromSegments(*selected, segments, index + 1, value);
  }

  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    const google::protobuf::Message& child =
        message.GetReflection()->GetMessage(message, field);
    if (is_last) {
      return false;
    }
    return ExtractNumericFromSegments(child, segments, index + 1, value);
  }

  if (!is_last) {
    return false;
  }
  return ExtractNumericLeaf(message, field, value);
}

bool ResolveLeafFromSegments(
    const google::protobuf::Message& message,
    const std::vector<MessagePathSegment>& segments, std::size_t index,
    const google::protobuf::Message** container,
    const google::protobuf::FieldDescriptor** leaf_field) {
  if (index >= segments.size() || container == nullptr || leaf_field == nullptr) {
    return false;
  }
  const MessagePathSegment& segment = segments[index];
  const google::protobuf::Descriptor* desc = message.GetDescriptor();
  const google::protobuf::FieldDescriptor* field =
      desc->FindFieldByName(segment.field);
  if (field == nullptr) {
    return false;
  }

  const bool is_last = index + 1 >= segments.size();
  if (field->is_repeated()) {
    const google::protobuf::Message* selected = nullptr;
    if (!NavigateRepeated(message, field, segment, &selected) ||
        selected == nullptr) {
      return false;
    }
    if (is_last) {
      return false;
    }
    return ResolveLeafFromSegments(*selected, segments, index + 1, container,
                                   leaf_field);
  }

  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    const google::protobuf::Message& child =
        message.GetReflection()->GetMessage(message, field);
    if (is_last) {
      return false;
    }
    return ResolveLeafFromSegments(child, segments, index + 1, container,
                                   leaf_field);
  }

  if (!is_last) {
    return false;
  }
  *container = &message;
  *leaf_field = field;
  return true;
}

}  // namespace

std::vector<MessagePathSegment> ParseMessagePath(const std::string& path) {
  std::vector<MessagePathSegment> segments;
  if (path.empty()) {
    return segments;
  }
  std::istringstream stream(path);
  std::string token;
  while (std::getline(stream, token, '.')) {
    if (token.empty()) {
      continue;
    }
    segments.push_back(ParseSegment(token));
  }
  return segments;
}

bool ExtractNumericByMessagePath(const google::protobuf::Message& message,
                                 const std::string& field_path, double* value) {
  if (field_path.empty() || value == nullptr) {
    return false;
  }
  const std::vector<MessagePathSegment> segments = ParseMessagePath(field_path);
  if (segments.empty()) {
    return false;
  }
  return ExtractNumericFromSegments(message, segments, 0, value);
}

bool ResolveLeafFieldByMessagePath(
    const google::protobuf::Message& message, const std::string& field_path,
    const google::protobuf::Message** container,
    const google::protobuf::FieldDescriptor** leaf_field) {
  if (field_path.empty()) {
    return false;
  }
  const std::vector<MessagePathSegment> segments = ParseMessagePath(field_path);
  if (segments.empty()) {
    return false;
  }
  return ResolveLeafFromSegments(message, segments, 0, container, leaf_field);
}

bool ElementMatchesPathFilter(const google::protobuf::Message& element,
                                const MessagePathFilter& filter) {
  return MatchesFilter(element, filter);
}

bool NavigateToChildMessage(const google::protobuf::Message& message,
                            const MessagePathSegment& segment,
                            const google::protobuf::Message** child) {
  if (child == nullptr) {
    return false;
  }
  const google::protobuf::FieldDescriptor* field =
      message.GetDescriptor()->FindFieldByName(segment.field);
  if (field == nullptr) {
    return false;
  }
  if (field->is_repeated()) {
    return NavigateRepeated(message, field, segment, child);
  }
  if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  *child = &message.GetReflection()->GetMessage(message, field);
  return true;
}

bool ResolveRepeatedFieldPath(const google::protobuf::Message& message,
                              const std::string& field_path,
                              ResolvedRepeatedField* resolved) {
  if (field_path.empty() || resolved == nullptr) {
    return false;
  }
  const std::vector<MessagePathSegment> segments = ParseMessagePath(field_path);
  if (segments.empty()) {
    return false;
  }

  const google::protobuf::Message* current = &message;
  for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
    const google::protobuf::Message* next = nullptr;
    if (!NavigateToChildMessage(*current, segments[i], &next) || next == nullptr) {
      return false;
    }
    current = next;
  }

  const MessagePathSegment& last = segments.back();
  const google::protobuf::FieldDescriptor* field =
      current->GetDescriptor()->FindFieldByName(last.field);
  if (field == nullptr || !field->is_repeated()) {
    return false;
  }

  resolved->container = current;
  resolved->repeated_field = field;
  resolved->element_filter = last.filter;
  resolved->use_single_index =
      last.has_bracket && !last.all_elements && !last.filter.has_value();
  resolved->single_index = last.index;
  return true;
}

std::optional<std::string> FormatMessagePathValue(
    const google::protobuf::Message& message, const std::string& field_path) {
  if (field_path.empty()) {
    return std::nullopt;
  }

  double numeric_value = 0.0;
  if (ExtractNumericByMessagePath(message, field_path, &numeric_value)) {
    return std::to_string(numeric_value);
  }

  const google::protobuf::Message* container = nullptr;
  const google::protobuf::FieldDescriptor* leaf_field = nullptr;
  if (ResolveLeafFieldByMessagePath(message, field_path, &container, &leaf_field) &&
      container != nullptr && leaf_field != nullptr) {
    if (leaf_field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return container->GetReflection()
          ->GetMessage(*container, leaf_field)
          .ShortDebugString();
    }
    return ReadStringField(*container, leaf_field);
  }

  ResolvedRepeatedField repeated;
  if (!ResolveRepeatedFieldPath(message, field_path, &repeated) ||
      repeated.container == nullptr || repeated.repeated_field == nullptr) {
    return std::nullopt;
  }

  const google::protobuf::Reflection* reflection =
      repeated.container->GetReflection();
  const int count = reflection->FieldSize(*repeated.container, repeated.repeated_field);
  if (count <= 0) {
    return std::string("(empty array)");
  }

  std::ostringstream out;
  int emitted = 0;
  for (int i = 0; i < count; ++i) {
    if (repeated.use_single_index && i != repeated.single_index) {
      continue;
    }
    if (repeated.repeated_field->type() ==
        google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      const google::protobuf::Message& element =
          reflection->GetRepeatedMessage(*repeated.container, repeated.repeated_field,
                                         i);
      if (repeated.element_filter.has_value() &&
          !MatchesFilter(element, *repeated.element_filter)) {
        continue;
      }
      if (emitted++ > 0) {
        out << "\n---\n";
      }
      out << element.ShortDebugString();
      continue;
    }

    if (repeated.element_filter.has_value()) {
      continue;
    }
    if (repeated.use_single_index && i != repeated.single_index) {
      continue;
    }
    if (emitted++ > 0) {
      out << '\n';
    }
    switch (repeated.repeated_field->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        out << reflection->GetRepeatedDouble(*repeated.container,
                                             repeated.repeated_field, i);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        out << reflection->GetRepeatedFloat(*repeated.container,
                                            repeated.repeated_field, i);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        out << reflection->GetRepeatedInt32(*repeated.container,
                                            repeated.repeated_field, i);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        out << reflection->GetRepeatedInt64(*repeated.container,
                                            repeated.repeated_field, i);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        out << reflection->GetRepeatedUInt32(*repeated.container,
                                             repeated.repeated_field, i);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        out << reflection->GetRepeatedUInt64(*repeated.container,
                                             repeated.repeated_field, i);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        out << (reflection->GetRepeatedBool(*repeated.container,
                                            repeated.repeated_field, i)
                    ? "true"
                    : "false");
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        out << reflection->GetRepeatedString(*repeated.container,
                                             repeated.repeated_field, i);
        break;
      default:
        break;
    }
  }
  if (emitted == 0) {
    return std::string("(no matching array elements)");
  }
  return out.str();
}

}  // namespace plot
}  // namespace autoviz
