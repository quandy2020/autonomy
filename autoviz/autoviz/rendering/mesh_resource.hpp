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
   * @brief Maps a package URI prefix onto a flat content root (source tree).
   *
   * Example: @c ocs2_robotic_assets + content_root=@c …/assets + strip
   * @c "resources/" resolves
   * @c package://ocs2_robotic_assets/resources/quadrotor/meshes/x.obj →
   * @c …/assets/quadrotor/meshes/x.obj (RViz / OCS2 layout without install).
   *
   * @param package_name Package name as used in @c package:// URIs.
   * @param content_root Absolute directory that contains robot folders.
   * @param uri_prefix_to_strip Leading subpath stripped from the URI rest.
   */
  void addPackageContentRoot(const std::string& package_name,
                             const std::string& content_root,
                             const std::string& uri_prefix_to_strip);

  /**
   * @brief Registers default @c ocs2_robotic_assets roots from env / install.
   *
   * Idempotent. Called automatically from @ref resolvePath / @ref exists.
   */
  void ensureDefaultPackagesRegistered();

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

  struct PackageContentRoot {
    std::string content_root;
    std::string uri_prefix_to_strip;
  };

  /** @brief package name → share root. */
  std::map<std::string, std::string> package_share_paths_;

  /** @brief package name → source-tree content mapping. */
  std::map<std::string, PackageContentRoot> package_content_roots_;

  /** @brief Whether @ref ensureDefaultPackagesRegistered ran. */
  bool defaults_registered_ = false;
};

}  // namespace rendering
}  // namespace autoviz

