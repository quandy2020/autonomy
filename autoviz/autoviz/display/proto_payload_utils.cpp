/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/proto_payload_utils.hpp"

#include <cstring>

namespace autoviz {
namespace display {
namespace proto_wire {
namespace {

bool ReadVarint(const std::string& data, size_t* pos, uint64_t* value) {
  if (pos == nullptr || value == nullptr) {
    return false;
  }
  *value = 0;
  int shift = 0;
  while (*pos < data.size() && shift < 64) {
    const uint8_t byte = static_cast<uint8_t>(data[(*pos)++]);
    *value |= static_cast<uint64_t>(byte & 0x7f) << shift;
    if ((byte & 0x80) == 0) {
      return true;
    }
    shift += 7;
  }
  return false;
}

bool ReadLengthDelimited(const std::string& data, size_t* pos,
                         std::string* out) {
  if (pos == nullptr || out == nullptr) {
    return false;
  }
  uint64_t len = 0;
  if (!ReadVarint(data, pos, &len)) {
    return false;
  }
  if (*pos + len > data.size()) {
    return false;
  }
  out->assign(data.data() + static_cast<std::ptrdiff_t>(*pos),
              static_cast<size_t>(len));
  *pos += static_cast<size_t>(len);
  return true;
}

bool SkipField(const std::string& data, size_t* pos, uint32_t wire_type) {
  if (pos == nullptr) {
    return false;
  }
  switch (wire_type) {
    case 0: {
      uint64_t dummy = 0;
      return ReadVarint(data, pos, &dummy);
    }
    case 1:
      if (*pos + 8 > data.size()) {
        return false;
      }
      *pos += 8;
      return true;
    case 2: {
      std::string ignored;
      return ReadLengthDelimited(data, pos, &ignored);
    }
    case 5:
      if (*pos + 4 > data.size()) {
        return false;
      }
      *pos += 4;
      return true;
    default:
      return false;
  }
}

void AppendPackedDoubles(const std::string& packed, std::vector<double>* out) {
  if (out == nullptr) {
    return;
  }
  for (size_t i = 0; i + 8 <= packed.size(); i += 8) {
    double value = 0.0;
    std::memcpy(&value, packed.data() + static_cast<std::ptrdiff_t>(i), 8);
    out->push_back(value);
  }
}

}  // namespace

bool UnwrapStdStringPayload(const std::string& payload, std::string* out) {
  if (out == nullptr || payload.empty()) {
    return false;
  }
  if (payload[0] == '<') {
    *out = payload;
    return true;
  }

  size_t pos = 0;
  while (pos < payload.size()) {
    uint64_t tag = 0;
    if (!ReadVarint(payload, &pos, &tag)) {
      break;
    }
    const uint32_t field = static_cast<uint32_t>(tag >> 3);
    const uint32_t wire = static_cast<uint32_t>(tag & 0x7u);
    if (field == 1 && wire == 2) {
      return ReadLengthDelimited(payload, &pos, out);
    }
    if (!SkipField(payload, &pos, wire)) {
      break;
    }
  }

  *out = payload;
  return true;
}

bool ParseJointStatePayload(const std::string& payload,
                            ParsedJointState* out) {
  if (out == nullptr) {
    return false;
  }
  out->names.clear();
  out->positions.clear();
  out->velocities.clear();
  out->efforts.clear();

  size_t pos = 0;
  while (pos < payload.size()) {
    uint64_t tag = 0;
    if (!ReadVarint(payload, &pos, &tag)) {
      break;
    }
    const uint32_t field = static_cast<uint32_t>(tag >> 3);
    const uint32_t wire = static_cast<uint32_t>(tag & 0x7u);

    if (field == 1) {
      if (!SkipField(payload, &pos, wire)) {
        return false;
      }
    } else if (field == 2 && wire == 2) {
      std::string name;
      if (!ReadLengthDelimited(payload, &pos, &name)) {
        return false;
      }
      out->names.push_back(std::move(name));
    } else if (field == 3 || field == 4 || field == 5) {
      std::vector<double>* target = &out->positions;
      if (field == 4) {
        target = &out->velocities;
      } else if (field == 5) {
        target = &out->efforts;
      }
      if (wire == 1) {
        if (pos + 8 > payload.size()) {
          return false;
        }
        double value = 0.0;
        std::memcpy(&value, payload.data() + static_cast<std::ptrdiff_t>(pos),
                    8);
        pos += 8;
        target->push_back(value);
      } else if (wire == 2) {
        std::string packed;
        if (!ReadLengthDelimited(payload, &pos, &packed)) {
          return false;
        }
        AppendPackedDoubles(packed, target);
      } else if (!SkipField(payload, &pos, wire)) {
        return false;
      }
    } else if (!SkipField(payload, &pos, wire)) {
      return false;
    }
  }

  return !out->names.empty() || !out->positions.empty();
}

}  // namespace proto_wire
}  // namespace display
}  // namespace autoviz
