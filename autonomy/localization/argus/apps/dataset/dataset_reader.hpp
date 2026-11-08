/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_APPS_DATASET_DATASET_READER_HPP_
#define ARGUS_APPS_DATASET_DATASET_READER_HPP_

#include <cstddef>
#include <string>

namespace argus {
namespace datasets {

/**
 * @brief Minimal sequential dataset reader contract (apps-only).
 *
 * Concrete readers (KITTI / Newer College / TUM RGB-D / Oxford Spires) expose
 * typed frames via their own Read() overloads; this base only captures the
 * shared open / size lifecycle used by offline runners.
 */
class DatasetReader {
public:
    virtual ~DatasetReader() = default;

    /** @brief Open a dataset root; return true if at least one frame indexed. */
    virtual bool Open(const std::string& root) = 0;

    /** @return Number of indexed frames. */
    virtual std::size_t size() const = 0;

    /** @return True when size() == 0. */
    virtual bool empty() const { return size() == 0; }

    /** @return Path passed to Open(). */
    virtual const std::string& root() const = 0;
};

}  // namespace datasets
}  // namespace argus

#endif
