/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoviz {
namespace rendering {

struct MeshResource {
  std::vector<uint8_t> data;
  /** Absolute path or logical URI used for relative texture lookup. */
  std::string resolved_uri;
};

class MeshResourceError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/** Lightweight resource_retriever subset for mesh_loader (file / package://). */
class MeshResourceResolver {
 public:
  static MeshResourceResolver& instance();

  void addPackageSharePath(const std::string& package_name,
                           const std::string& share_root);
  void clearPackagePaths();

  bool exists(const std::string& uri) const;
  std::optional<std::string> resolvePath(const std::string& uri) const;
  std::shared_ptr<MeshResource> fetch(const std::string& uri) const;

 private:
  std::optional<std::string> resolveToPath(const std::string& uri) const;
  std::shared_ptr<MeshResource> fetchHttp(const std::string& uri) const;

  std::map<std::string, std::string> package_share_paths_;
};

}  // namespace rendering
}  // namespace autoviz

#endif
