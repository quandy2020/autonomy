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

#include "autonomy/map/geonoma/osm.hpp"

#include <fstream>
#include <regex>
#include <sstream>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace map {
namespace geonoma {

OsmMap::OsmMap(const std::string& name) : name_(name.empty() ? "osm_map" : name), stopped_(true), paused_(false) {
    LOG(INFO) << "[OsmMap] OsmMap created: " << name_;
}

OsmMap::~OsmMap() {
    Stop();
    LOG(INFO) << "[OsmMap] OsmMap destroyed: " << name_;
}

void OsmMap::Start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!stopped_) {
        LOG(WARNING) << "[OsmMap] Map is already started.";
        return;
    }

    stopped_ = false;
    paused_ = false;
    LOG(INFO) << "[OsmMap] Map started: " << name_;
}

void OsmMap::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        return;
    }

    stopped_ = true;
    paused_ = false;
    LOG(INFO) << "[OsmMap] Map stopped: " << name_;
}

void OsmMap::Pause() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        LOG(WARNING) << "[OsmMap] Cannot pause stopped map.";
        return;
    }

    paused_ = true;
    LOG(INFO) << "[OsmMap] Map paused: " << name_;
}

void OsmMap::Resume() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        LOG(WARNING) << "[OsmMap] Cannot resume stopped map.";
        return;
    }

    paused_ = false;
    LOG(INFO) << "[OsmMap] Map resumed: " << name_;
}

bool OsmMap::LoadFromFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::ifstream file(filename);
    if (!file.is_open()) {
        LOG(ERROR) << "[OsmMap] Failed to open file: " << filename;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    std::string xml_content = buffer.str();

    // Clear existing data
    Clear();

    bool success = ParseOsmXml(xml_content);
    if (success) {
        LOG(INFO) << "[OsmMap] Loaded OSM map from file: " << filename << " (Nodes: " << nodes_.size()
                  << ", Ways: " << ways_.size() << ", Relations: " << relations_.size() << ")";
    } else {
        LOG(ERROR) << "[OsmMap] Failed to parse OSM XML from file: " << filename;
    }

    return success;
}

bool OsmMap::SaveToFile(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(mutex_);

    bool success = WriteOsmXml(filename);
    if (success) {
        LOG(INFO) << "[OsmMap] Saved OSM map to file: " << filename;
    } else {
        LOG(ERROR) << "[OsmMap] Failed to save OSM map to file: " << filename;
    }

    return success;
}

void OsmMap::AddNode(const OsmNode& node) {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_[node.id] = node;
}

void OsmMap::AddWay(const OsmWay& way) {
    std::lock_guard<std::mutex> lock(mutex_);
    ways_[way.id] = way;
}

void OsmMap::AddRelation(const OsmRelation& relation) {
    std::lock_guard<std::mutex> lock(mutex_);
    relations_[relation.id] = relation;
}

const OsmNode* OsmMap::GetNode(int64_t node_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodes_.find(node_id);
    if (it != nodes_.end()) {
        return &it->second;
    }
    return nullptr;
}

const OsmWay* OsmMap::GetWay(int64_t way_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ways_.find(way_id);
    if (it != ways_.end()) {
        return &it->second;
    }
    return nullptr;
}

const OsmRelation* OsmMap::GetRelation(int64_t relation_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = relations_.find(relation_id);
    if (it != relations_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<int64_t> OsmMap::FindNodesByTag(const std::string& key, const std::string& value) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<int64_t> result;
    for (const auto& pair : nodes_) {
        if (TagMatches(pair.second.tags, key, value)) {
            result.push_back(pair.first);
        }
    }
    return result;
}

std::vector<int64_t> OsmMap::FindWaysByTag(const std::string& key, const std::string& value) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<int64_t> result;
    for (const auto& pair : ways_) {
        if (TagMatches(pair.second.tags, key, value)) {
            result.push_back(pair.first);
        }
    }
    return result;
}

std::vector<int64_t> OsmMap::FindRelationsByTag(const std::string& key, const std::string& value) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<int64_t> result;
    for (const auto& pair : relations_) {
        if (TagMatches(pair.second.tags, key, value)) {
            result.push_back(pair.first);
        }
    }
    return result;
}

std::vector<int64_t> OsmMap::GetNodesInBounds(double min_lat, double min_lon, double max_lat, double max_lon) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<int64_t> result;
    for (const auto& pair : nodes_) {
        const OsmNode& node = pair.second;
        if (node.latitude >= min_lat && node.latitude <= max_lat && node.longitude >= min_lon &&
            node.longitude <= max_lon) {
            result.push_back(pair.first);
        }
    }
    return result;
}

void OsmMap::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_.clear();
    ways_.clear();
    relations_.clear();
}

bool OsmMap::TagMatches(const std::vector<OsmTag>& tags, const std::string& key, const std::string& value) const {
    for (const auto& tag : tags) {
        if (tag.key == key) {
            if (value.empty() || tag.value == value) {
                return true;
            }
        }
    }
    return false;
}

bool OsmMap::ParseOsmXml(const std::string& xml_content) {
    // Simple XML parser for OSM format
    // This is a basic implementation - for production use, consider using a
    // proper XML library

    std::regex node_regex(R"(<node\s+id=["'](\d+)["']\s+lat=["']([\d\.\-]+)["']\s+lon=["']([\d\.\-]+)["']([^>]*)>)");
    std::regex way_regex(R"(<way\s+id=["'](\d+)["']([^>]*)>)");
    std::regex relation_regex(R"(<relation\s+id=["'](\d+)["']([^>]*)>)");
    std::regex tag_regex(R"(<tag\s+k=["']([^"']+)["']\s+v=["']([^"']+)["']\s*/>)");
    std::regex nd_regex(R"(<nd\s+ref=["'](\d+)["']\s*/>)");
    std::regex member_regex(
        R"(<member\s+type=["'](node|way|relation)["']\s+ref=["'](\d+)["']\s+role=["']([^"']*)["']\s*/>)");

    std::sregex_iterator iter;
    std::sregex_iterator end;

    // Parse nodes
    iter = std::sregex_iterator(xml_content.begin(), xml_content.end(), node_regex);
    for (; iter != end; ++iter) {
        std::smatch match = *iter;
        int64_t id = std::stoll(match[1].str());
        double lat = std::stod(match[2].str());
        double lon = std::stod(match[3].str());

        OsmNode node(id, lat, lon);

        // Find tags for this node (look for tags between this node and the next
        // node/way/relation)
        std::string node_section = match[0].str();
        size_t node_start = match.position(0);
        size_t node_end = xml_content.find("</node>", node_start);
        if (node_end != std::string::npos) {
            std::string node_content = xml_content.substr(node_start, node_end - node_start);
            std::sregex_iterator tag_iter(node_content.begin(), node_content.end(), tag_regex);
            std::sregex_iterator tag_end;
            for (; tag_iter != tag_end; ++tag_iter) {
                std::smatch tag_match = *tag_iter;
                node.tags.emplace_back(tag_match[1].str(), tag_match[2].str());
            }
        }

        nodes_[id] = node;
    }

    // Parse ways
    iter = std::sregex_iterator(xml_content.begin(), xml_content.end(), way_regex);
    for (; iter != end; ++iter) {
        std::smatch match = *iter;
        int64_t id = std::stoll(match[1].str());

        OsmWay way(id);

        // Find nd refs and tags for this way
        size_t way_start = match.position(0);
        size_t way_end = xml_content.find("</way>", way_start);
        if (way_end != std::string::npos) {
            std::string way_content = xml_content.substr(way_start, way_end - way_start);

            // Parse nd refs
            std::sregex_iterator nd_iter(way_content.begin(), way_content.end(), nd_regex);
            std::sregex_iterator nd_end;
            for (; nd_iter != nd_end; ++nd_iter) {
                std::smatch nd_match = *nd_iter;
                way.node_refs.push_back(std::stoll(nd_match[1].str()));
            }

            // Parse tags
            std::sregex_iterator tag_iter(way_content.begin(), way_content.end(), tag_regex);
            std::sregex_iterator tag_end;
            for (; tag_iter != tag_end; ++tag_iter) {
                std::smatch tag_match = *tag_iter;
                way.tags.emplace_back(tag_match[1].str(), tag_match[2].str());
            }
        }

        ways_[id] = way;
    }

    // Parse relations
    iter = std::sregex_iterator(xml_content.begin(), xml_content.end(), relation_regex);
    for (; iter != end; ++iter) {
        std::smatch match = *iter;
        int64_t id = std::stoll(match[1].str());

        OsmRelation relation(id);

        // Find members and tags for this relation
        size_t relation_start = match.position(0);
        size_t relation_end = xml_content.find("</relation>", relation_start);
        if (relation_end != std::string::npos) {
            std::string relation_content = xml_content.substr(relation_start, relation_end - relation_start);

            // Parse members
            std::sregex_iterator member_iter(relation_content.begin(), relation_content.end(), member_regex);
            std::sregex_iterator member_end;
            for (; member_iter != member_end; ++member_iter) {
                std::smatch member_match = *member_iter;
                OsmMemberType type = OsmMemberType::NODE;
                if (member_match[1].str() == "way") {
                    type = OsmMemberType::WAY;
                } else if (member_match[1].str() == "relation") {
                    type = OsmMemberType::RELATION;
                }
                relation.members.emplace_back(type, std::stoll(member_match[2].str()), member_match[3].str());
            }

            // Parse tags
            std::sregex_iterator tag_iter(relation_content.begin(), relation_content.end(), tag_regex);
            std::sregex_iterator tag_end;
            for (; tag_iter != tag_end; ++tag_iter) {
                std::smatch tag_match = *tag_iter;
                relation.tags.emplace_back(tag_match[1].str(), tag_match[2].str());
            }
        }

        relations_[id] = relation;
    }

    return true;
}

bool OsmMap::WriteOsmXml(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG(ERROR) << "[OsmMap] Failed to open file for writing: " << filename;
        return false;
    }

    // Write OSM XML header
    file << "<?xml version='1.0' encoding='UTF-8'?>\n";
    file << "<osm version='0.6' generator='autonomy'>\n";

    // Write nodes
    for (const auto& pair : nodes_) {
        const OsmNode& node = pair.second;
        file << "  <node id='" << node.id << "' lat='" << node.latitude << "' lon='" << node.longitude << "'>\n";

        for (const auto& tag : node.tags) {
            file << "    <tag k='" << tag.key << "' v='" << tag.value << "'/>\n";
        }

        file << "  </node>\n";
    }

    // Write ways
    for (const auto& pair : ways_) {
        const OsmWay& way = pair.second;
        file << "  <way id='" << way.id << "'>\n";

        for (int64_t node_ref : way.node_refs) {
            file << "    <nd ref='" << node_ref << "'/>\n";
        }

        for (const auto& tag : way.tags) {
            file << "    <tag k='" << tag.key << "' v='" << tag.value << "'/>\n";
        }

        file << "  </way>\n";
    }

    // Write relations
    for (const auto& pair : relations_) {
        const OsmRelation& relation = pair.second;
        file << "  <relation id='" << relation.id << "'>\n";

        for (const auto& member : relation.members) {
            std::string type_str = "node";
            if (member.type == OsmMemberType::WAY) {
                type_str = "way";
            } else if (member.type == OsmMemberType::RELATION) {
                type_str = "relation";
            }
            file << "    <member type='" << type_str << "' ref='" << member.ref << "' role='" << member.role << "'/>\n";
        }

        for (const auto& tag : relation.tags) {
            file << "    <tag k='" << tag.key << "' v='" << tag.value << "'/>\n";
        }

        file << "  </relation>\n";
    }

    // Write OSM XML footer
    file << "</osm>\n";

    file.close();
    return true;
}

}  // namespace geonoma
}  // namespace map
}  // namespace autonomy
