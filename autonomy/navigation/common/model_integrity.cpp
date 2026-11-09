/* Copyright 2026 The Navigation Authors (navigation) */
#include "navigation/common/model_integrity.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

#include <openssl/evp.h>

namespace navigation::common {

Status ValidateModelIntegrity(const std::string& path,
                              const std::string& sha256) {
    if (sha256.size() != 64 ||
        !std::all_of(sha256.begin(), sha256.end(), [](char c) {
            return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        })) {
        return Status::Configuration(
            "model sha256 must contain 64 lowercase hex digits");
    }
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
        return Status::Unavailable("cannot open model: " + path);
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> digest(
        EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1)
        return Status::Unavailable("cannot initialize model SHA-256");
    std::array<char, 65536> buffer;
    while (stream) {
        stream.read(buffer.data(), buffer.size());
        if (EVP_DigestUpdate(digest.get(), buffer.data(), stream.gcount()) != 1)
            return Status::Unavailable("cannot hash model: " + path);
    }
    if (!stream.eof())
        return Status::Unavailable("cannot read model: " + path);
    std::array<unsigned char, EVP_MAX_MD_SIZE> bytes;
    unsigned int count = 0;
    if (EVP_DigestFinal_ex(digest.get(), bytes.data(), &count) != 1)
        return Status::Unavailable("cannot finish model SHA-256");
    std::ostringstream actual;
    actual << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < count; ++i)
        actual << std::setw(2) << static_cast<unsigned int>(bytes[i]);
    if (actual.str() != sha256)
        return Status::Configuration("model sha256 mismatch: " + path);
    return Status{};
}

}  // namespace navigation::common
