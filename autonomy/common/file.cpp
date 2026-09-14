/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/file.hpp"

namespace autonomy {
namespace common {

std::string EnsureTrailingSlash(const std::string& str) {
    if (str.length() > 0) {
        if (str.back() != '/') {
            return str + "/";
        }
    } else {
        return str + "/";
    }
    return str;
}

bool HasFileExtension(const std::string& file_name, const std::string& ext) {
    THROW_CHECK(!ext.empty());
    THROW_CHECK_EQ(ext.at(0), '.');
    std::string ext_lower = ext;
    StringToLower(&ext_lower);
    if (file_name.size() >= ext_lower.size() &&
        file_name.substr(file_name.size() - ext_lower.size(),
                         ext_lower.size()) == ext_lower) {
        return true;
    }
    return false;
}

void SplitFileExtension(const std::string& path, std::string* root,
                        std::string* ext) {
    if (path.empty()) {
        *root = "";
        *ext = "";
        return;
    }
    const auto separator = path.find_last_of("/\\");
    const auto dot = path.find_last_of('.');
    if (dot == std::string::npos ||
        (separator != std::string::npos && dot < separator)) {
        *root = path;
        *ext = "";
    } else if (dot == path.size() - 1) {
        *root = path.substr(0, dot);
        *ext = "";
    } else {
        *root = path.substr(0, dot);
        *ext = path.substr(dot);
    }
}

void FileCopy(const std::string& src_path, const std::string& dst_path,
              CopyType type) {
    switch (type) {
        case CopyType::COPY:
            std::filesystem::copy_file(src_path, dst_path);
            break;
        case CopyType::HARD_LINK:
            std::filesystem::create_hard_link(src_path, dst_path);
            break;
        case CopyType::SOFT_LINK:
            std::filesystem::create_symlink(src_path, dst_path);
            break;
    }
}

bool ExistsFile(const std::string& path) {
    return std::filesystem::is_regular_file(path);
}

bool ExistsDir(const std::string& path) {
    return std::filesystem::is_directory(path);
}

bool ExistsPath(const std::string& path) {
    return std::filesystem::exists(path);
}

void CreateDirIfNotExists(const std::string& path, bool recursive) {
    if (ExistsDir(path)) {
        return;
    }
    if (recursive) {
        THROW_CHECK(std::filesystem::create_directories(path));
    } else {
        THROW_CHECK(std::filesystem::create_directory(path));
    }
}

std::string GetPathBaseName(const std::string& path) {
    std::string normalized = StringReplace(path, "\\", "/");
    while (!normalized.empty() && normalized.back() == '/') {
        normalized.pop_back();
    }
    if (normalized.empty()) {
        return "";
    }
    const auto separator = normalized.find_last_of('/');
    return separator == std::string::npos
               ? normalized
               : normalized.substr(separator + 1);
}

std::string GetParentDir(const std::string& path) {
    return std::filesystem::path(path).parent_path().string();
}

std::vector<std::string> GetFileList(const std::string& path) {
    std::vector<std::string> file_list;
    for (auto it = std::filesystem::directory_iterator(path);
         it != std::filesystem::directory_iterator(); ++it) {
        if (std::filesystem::is_regular_file(*it)) {
            const std::filesystem::path file_path = *it;
            file_list.push_back(file_path.string());
        }
    }
    return file_list;
}

std::vector<std::string> GetRecursiveFileList(const std::string& path) {
    std::vector<std::string> file_list;
    for (auto it = std::filesystem::recursive_directory_iterator(path);
         it != std::filesystem::recursive_directory_iterator(); ++it) {
        if (std::filesystem::is_regular_file(*it)) {
            const std::filesystem::path file_path = *it;
            file_list.push_back(file_path.string());
        }
    }
    return file_list;
}

std::vector<std::string> GetDirList(const std::string& path) {
    std::vector<std::string> dir_list;
    for (auto it = std::filesystem::directory_iterator(path);
         it != std::filesystem::directory_iterator(); ++it) {
        if (std::filesystem::is_directory(*it)) {
            const std::filesystem::path dir_path = *it;
            dir_list.push_back(dir_path.string());
        }
    }
    return dir_list;
}

std::vector<std::string> GetRecursiveDirList(const std::string& path) {
    std::vector<std::string> dir_list;
    for (auto it = std::filesystem::recursive_directory_iterator(path);
         it != std::filesystem::recursive_directory_iterator(); ++it) {
        if (std::filesystem::is_directory(*it)) {
            const std::filesystem::path dir_path = *it;
            dir_list.push_back(dir_path.string());
        }
    }
    return dir_list;
}

size_t GetFileSize(const std::string& path) {
    std::ifstream file(path, std::ifstream::ate | std::ifstream::binary);
    THROW_CHECK_FILE_OPEN(file, path);
    return file.tellg();
}

std::vector<std::string> ReadTextFileLines(const std::string& path) {
    std::ifstream file(path);
    THROW_CHECK_FILE_OPEN(file, path);

    std::string line;
    std::vector<std::string> lines;
    while (std::getline(file, line)) {
        StringTrim(&line);

        if (line.empty()) {
            continue;
        }

        lines.push_back(line);
    }

    return lines;
}

}  // namespace common
}  // namespace autonomy
