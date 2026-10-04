/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file mesh_resource.hpp
 * @brief Lightweight mesh URI resolver (file / package://) for Ogre mesh loading.
 *
 * Subset of ROS @c resource_retriever used by @ref OgreMeshLoader to locate
 * mesh bytes and resolve relative texture paths.
 *
 * @see OgreMeshLoader
 * @see MeshResourceResolver
 */

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoviz {
namespace rendering {

/**
 * @struct MeshResource
 * @brief In-memory mesh file bytes plus a resolved URI for relative lookups.
 */
struct MeshResource {
  /** @brief Raw file contents. */
  std::vector<uint8_t> data;

  /**
   * @brief Absolute path or logical URI used for relative texture lookup.
   *
   * Typically the filesystem path after resolving @c package:// or @c file://.
   */
  std::string resolved_uri;
};

/**
 * @class MeshResourceError
 * @brief Thrown when a mesh URI cannot be resolved or fetched.
 */
class MeshResourceError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/**
 * @class MeshResourceResolver
 * @brief Singleton resolver for @c file://, absolute paths, and @c package://.
 *
 * Register package share roots with @ref addPackageSharePath() before loading
 * meshes that use @c package://&lt;name&gt;/… URIs.
 */
class MeshResourceResolver {
 public:
  /**
   * @brief Returns the process singleton.
   * @return Mutable reference to the shared resolver.
   */
  static MeshResourceResolver& instance();

  /**
   * @brief Maps a ROS package name to its share root on disk.
   *
   * @param package_name Package name as used in @c package:// URIs.
   * @param share_root Absolute filesystem path to the package share directory.
   */
  void addPackageSharePath(const std::string& package_name,
                           const std::string& share_root);

  /**
   * @brief Clears all registered package share paths.
   */
  void clearPackagePaths();

  /**
   * @brief Returns whether @p uri can be resolved to an existing file.
   *
   * @param uri Absolute path, @c file://, or @c package:// URI.
   * @return @c true if @ref resolvePath() would succeed and the file exists.
   */
  bool exists(const std::string& uri) const;

  /**
   * @brief Resolves @p uri to a local filesystem path without reading bytes.
   *
   * @param uri Resource URI.
   * @return Absolute path, or @c std::nullopt on failure.
   */
  std::optional<std::string> resolvePath(const std::string& uri) const;

  /**
   * @brief Fetches mesh bytes for @p uri.
   *
   * @param uri Resource URI (file / package / optionally http).
   * @return Shared @ref MeshResource, or empty shared_ptr on failure.
   * @throws MeshResourceError on hard failures (implementation-defined).
   */
  std::shared_ptr<MeshResource> fetch(const std::string& uri) const;

 private:
  /**
   * @brief Internal path resolution without existence checks.
   * @param uri Resource URI.
   * @return Absolute path or nullopt.
   */
  std::optional<std::string> resolveToPath(const std::string& uri) const;

  /**
   * @brief Optional HTTP fetch path for remote mesh URIs.
   * @param uri HTTP(S) URL.
   * @return Shared resource or empty.
   */
  std::shared_ptr<MeshResource> fetchHttp(const std::string& uri) const;

  /** @brief package name → share root. */
  std::map<std::string, std::string> package_share_paths_;
};

}  // namespace rendering
}  // namespace autoviz

