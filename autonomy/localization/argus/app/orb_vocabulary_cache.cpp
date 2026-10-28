/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "orb_vocabulary_cache.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <system_error>

#include <unistd.h>
#include <zlib.h>

namespace argus {
namespace app {
namespace {

bool EndsWith(const std::string& text, const std::string& suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool ReadExact(gzFile archive, void* data, std::size_t size) {
  auto* output = static_cast<unsigned char*>(data);
  while (size > 0) {
    const unsigned int chunk =
        static_cast<unsigned int>(std::min<std::size_t>(size, 1u << 20));
    const int count = gzread(archive, output, chunk);
    if (count <= 0) return false;
    output += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

std::uint64_t ParseOctal(const char* data, std::size_t size) {
  std::uint64_t value = 0;
  std::size_t index = 0;
  while (index < size && (data[index] == ' ' || data[index] == '\0')) ++index;
  for (; index < size && data[index] >= '0' && data[index] <= '7'; ++index) {
    if (value > (std::numeric_limits<std::uint64_t>::max() >> 3)) return 0;
    value = (value << 3) + static_cast<unsigned>(data[index] - '0');
  }
  return value;
}

bool Skip(gzFile archive, std::uint64_t size) {
  std::array<unsigned char, 8192> buffer{};
  while (size > 0) {
    const std::size_t chunk =
        std::min<std::uint64_t>(size, buffer.size());
    if (!ReadExact(archive, buffer.data(), chunk)) return false;
    size -= chunk;
  }
  return true;
}

bool Extract(const std::string& archive_path,
             const std::filesystem::path& output_path) {
  gzFile archive = gzopen(archive_path.c_str(), "rb");
  if (archive == nullptr) return false;

  bool success = false;
  std::array<char, 512> header{};
  while (ReadExact(archive, header.data(), header.size())) {
    bool all_zero = true;
    for (char byte : header) all_zero = all_zero && byte == '\0';
    if (all_zero) break;

    const std::string name(header.data(),
                           strnlen(header.data(), 100));
    const std::uint64_t size = ParseOctal(header.data() + 124, 12);
    const std::uint64_t padded = (size + 511u) & ~std::uint64_t(511u);
    const bool vocabulary =
        name == "ORBvoc.txt" || EndsWith(name, "/ORBvoc.txt");

    if (!vocabulary) {
      if (!Skip(archive, padded)) break;
      continue;
    }

    const std::filesystem::path temporary =
        output_path.string() + ".part." + std::to_string(::getpid());
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    std::array<unsigned char, 1 << 20> buffer{};
    std::uint64_t remaining = size;
    while (output && remaining > 0) {
      const std::size_t chunk =
          std::min<std::uint64_t>(remaining, buffer.size());
      if (!ReadExact(archive, buffer.data(), chunk)) {
        output.setstate(std::ios::badbit);
        break;
      }
      output.write(reinterpret_cast<const char*>(buffer.data()), chunk);
      remaining -= chunk;
    }
    output.close();
    if (remaining == 0 && Skip(archive, padded - size) && output) {
      std::error_code error;
      std::filesystem::rename(temporary, output_path, error);
      if (error && !std::filesystem::exists(output_path)) {
        std::filesystem::remove(temporary, error);
        break;
      }
      std::filesystem::remove(temporary, error);
      success = true;
    } else {
      std::error_code error;
      std::filesystem::remove(temporary, error);
    }
    break;
  }
  gzclose(archive);
  return success;
}

}  // namespace

std::string PrepareOrbVocabulary(const std::string& path) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error)) return {};
  if (!EndsWith(path, ".tar.gz") && !EndsWith(path, ".tgz")) return path;

  const auto bytes = std::filesystem::file_size(path, error);
  if (error) return {};
  const std::string cache_key =
      std::to_string(std::hash<std::string>{}(path)) + "-" +
      std::to_string(bytes);
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path(error) /
      "argus_orb_vocabulary";
  if (error) return {};
  std::filesystem::create_directories(directory, error);
  if (error) return {};
  const std::filesystem::path output = directory / (cache_key + ".txt");
  if (std::filesystem::is_regular_file(output, error) &&
      std::filesystem::file_size(output, error) > 0) {
    return output.string();
  }
  return Extract(path, output) ? output.string() : std::string();
}

}  // namespace app
}  // namespace argus
