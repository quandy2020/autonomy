/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace map {
namespace geonoma {

/**
 * @struct OsmTag
 * @brief OSM tag (key-value pair) for semantic information
 */
struct OsmTag {
  std::string key;
  std::string value;

  OsmTag() = default;
  OsmTag(const std::string& k, const std::string& v) : key(k), value(v) {}
};

/**
 * @struct OsmNode
 * @brief OSM node representing a point of interest
 */
struct OsmNode {
  int64_t id;
  double latitude;
  double longitude;
  std::vector<OsmTag> tags;

  OsmNode() : id(0), latitude(0.0), longitude(0.0) {}
  OsmNode(int64_t node_id, double lat, double lon) : id(node_id), latitude(lat), longitude(lon) {}
};

/**
 * @struct OsmWay
 * @brief OSM way representing a path or area boundary
 */
struct OsmWay {
  int64_t id;
  std::vector<int64_t> node_refs;  // References to node IDs
  std::vector<OsmTag> tags;

  OsmWay() : id(0) {}
  OsmWay(int64_t way_id) : id(way_id) {}
};

/**
 * @enum OsmMemberType
 * @brief Type of OSM relation member
 */
enum class OsmMemberType { NODE = 0, WAY = 1, RELATION = 2 };

/**
 * @struct OsmMember
 * @brief Member of an OSM relation
 */
struct OsmMember {
  OsmMemberType type;
  int64_t ref;
  std::string role;

  OsmMember() : type(OsmMemberType::NODE), ref(0) {}
  OsmMember(OsmMemberType t, int64_t r, const std::string& ro) : type(t), ref(r), role(ro) {}
};

/**
 * @struct OsmRelation
 * @brief OSM relation representing a logical grouping of nodes, ways, and other
 * relations
 */
struct OsmRelation {
  int64_t id;
  std::vector<OsmMember> members;
  std::vector<OsmTag> tags;

  OsmRelation() : id(0) {}
  OsmRelation(int64_t relation_id) : id(relation_id) {}
};

/**
 * @class OsmMap
 * @brief Semantic map based on OpenStreetMap (OSM) format
 *
 * This class provides a semantic map representation using OSM data structures.
 * It supports loading and saving OSM XML files, and provides query interfaces
 * for semantic information such as roads, buildings, landmarks, etc.
 */
class OsmMap : public common::MapInterface {
 public:
  /**
   * Define OsmMap::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(OsmMap)

  /**
   * @brief Constructor for OsmMap
   * @param name Optional name for the OSM map
   */
  explicit OsmMap(const std::string& name = "osm_map");

  /**
   * @brief Destructor for OsmMap
   */
  ~OsmMap();

  /**
   * @brief Start the OSM map (no-op for semantic maps, but required by
   * interface)
   */
  void Start() override;

  /**
   * @brief Stop the OSM map (no-op for semantic maps, but required by
   * interface)
   */
  void Stop() override;

  /**
   * @brief Pause the OSM map (no-op for semantic maps, but required by
   * interface)
   */
  void Pause() override;

  /**
   * @brief Resume the OSM map (no-op for semantic maps, but required by
   * interface)
   */
  void Resume() override;

  /**
   * @brief Load OSM map from XML file
   * @param filename Path to the OSM XML file
   * @return true if loaded successfully, false otherwise
   */
  bool LoadFromFile(const std::string& filename);

  /**
   * @brief Save OSM map to XML file
   * @param filename Path to save the OSM XML file
   * @return true if saved successfully, false otherwise
   */
  bool SaveToFile(const std::string& filename) const;

  /**
   * @brief Add a node to the map
   * @param node OSM node to add
   */
  void AddNode(const OsmNode& node);

  /**
   * @brief Add a way to the map
   * @param way OSM way to add
   */
  void AddWay(const OsmWay& way);

  /**
   * @brief Add a relation to the map
   * @param relation OSM relation to add
   */
  void AddRelation(const OsmRelation& relation);

  /**
   * @brief Get a node by ID
   * @param node_id Node ID
   * @return Pointer to the node, or nullptr if not found
   */
  const OsmNode* GetNode(int64_t node_id) const;

  /**
   * @brief Get a way by ID
   * @param way_id Way ID
   * @return Pointer to the way, or nullptr if not found
   */
  const OsmWay* GetWay(int64_t way_id) const;

  /**
   * @brief Get a relation by ID
   * @param relation_id Relation ID
   * @return Pointer to the relation, or nullptr if not found
   */
  const OsmRelation* GetRelation(int64_t relation_id) const;

  /**
   * @brief Find nodes by tag key-value pair
   * @param key Tag key
   * @param value Tag value (optional, empty string matches any value)
   * @return Vector of node IDs matching the criteria
   */
  std::vector<int64_t> FindNodesByTag(const std::string& key, const std::string& value = "") const;

  /**
   * @brief Find ways by tag key-value pair
   * @param key Tag key
   * @param value Tag value (optional, empty string matches any value)
   * @return Vector of way IDs matching the criteria
   */
  std::vector<int64_t> FindWaysByTag(const std::string& key, const std::string& value = "") const;

  /**
   * @brief Find relations by tag key-value pair
   * @param key Tag key
   * @param value Tag value (optional, empty string matches any value)
   * @return Vector of relation IDs matching the criteria
   */
  std::vector<int64_t> FindRelationsByTag(const std::string& key, const std::string& value = "") const;

  /**
   * @brief Get all nodes within a bounding box
   * @param min_lat Minimum latitude
   * @param min_lon Minimum longitude
   * @param max_lat Maximum latitude
   * @param max_lon Maximum longitude
   * @return Vector of node IDs within the bounding box
   */
  std::vector<int64_t> GetNodesInBounds(double min_lat, double min_lon, double max_lat, double max_lon) const;

  /**
   * @brief Get the name of the map
   * @return Map name
   */
  std::string GetName() const { return name_; }

  /**
   * @brief Get the number of nodes
   * @return Number of nodes
   */
  size_t GetNodeCount() const { return nodes_.size(); }

  /**
   * @brief Get the number of ways
   * @return Number of ways
   */
  size_t GetWayCount() const { return ways_.size(); }

  /**
   * @brief Get the number of relations
   * @return Number of relations
   */
  size_t GetRelationCount() const { return relations_.size(); }

  /**
   * @brief Clear all map data
   */
  void Clear();

 private:
  // Map name
  std::string name_;

  // OSM data storage
  std::map<int64_t, OsmNode> nodes_;
  std::map<int64_t, OsmWay> ways_;
  std::map<int64_t, OsmRelation> relations_;

  // State management
  std::atomic<bool> stopped_{true};
  std::atomic<bool> paused_{false};
  // mutable 允许在 const 成员函数中加锁，保证线程安全
  mutable std::mutex mutex_;

  // Helper functions for XML parsing
  bool ParseOsmXml(const std::string& xml_content);
  bool WriteOsmXml(const std::string& filename) const;

  // Helper function to check if a tag matches
  bool TagMatches(const std::vector<OsmTag>& tags, const std::string& key, const std::string& value) const;
};

}  // namespace geonoma
}  // namespace map
}  // namespace autonomy
