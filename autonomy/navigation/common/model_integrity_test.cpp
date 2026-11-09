/* Copyright 2026 The Navigation Authors (navigation) */
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "navigation/common/model_integrity.hpp"

int main() {
    namespace fs = std::filesystem;
    const auto dir =
        fs::temp_directory_path() /
        ("navigation-model-integrity-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(dir);
    struct Cleanup {
        fs::path dir;
        ~Cleanup() {
            fs::remove_all(dir);
        }
    } cleanup{dir};
    const auto file = dir / "model";
    std::ofstream(file, std::ios::binary) << "abc";
    const std::string hash =
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
    using navigation::common::ValidateModelIntegrity;
    if (!ValidateModelIntegrity(file.string(), hash).Ok() ||
        ValidateModelIntegrity(file.string(), std::string(64, '0')).Ok() ||
        ValidateModelIntegrity(file.string(), "").Ok() ||
        ValidateModelIntegrity(file.string(), std::string(64, 'g')).Ok() ||
        ValidateModelIntegrity((dir / "missing").string(), hash).Ok()) {
        std::cerr << "model integrity validation failed\n";
        return 1;
    }
    std::ofstream(file, std::ios::app) << "tampered";
    if (ValidateModelIntegrity(file.string(), hash).Ok())
        return 1;
    return 0;
}
