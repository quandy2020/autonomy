#include "autonomy/localization/atlas/data/frame.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/camera_database.hpp"
#include "autonomy/localization/atlas/data/orb_params_database.hpp"
#include "autonomy/localization/atlas/data/bow_database.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/io/map_database_io_msgpack.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace io {

bool map_database_io_msgpack::save(const std::string& path,
                                   const data::camera_database* const cam_db,
                                   const data::orb_params_database* const orb_params_db,
                                   const data::map_database* const map_db) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    assert(cam_db && orb_params_db && map_db);
    const auto cameras = cam_db->to_json();
    const auto orb_params = orb_params_db->to_json();
    nlohmann::json keyfrms;
    nlohmann::json landmarks;
    nlohmann::json planes;
    nlohmann::json lines;
    map_db->to_json(keyfrms, landmarks);
    map_db->to_json_planes(planes);
    map_db->to_json_lines(lines);

    nlohmann::json json{{"cameras", cameras},
                        {"orb_params", orb_params},
                        {"keyframes", keyfrms},
                        {"landmarks", landmarks},
                        {"landmark_planes", planes},
                        {"landmarks_line", lines},
                        {"keyframe_next_id", static_cast<unsigned int>(map_db->next_keyframe_id_)},
                        {"landmark_next_id", static_cast<unsigned int>(map_db->next_landmark_id_)},
                        {"landmark_plane_next_id", static_cast<unsigned int>(map_db->next_landmark_plane_id_)},
                        {"landmark_line_next_id", static_cast<unsigned int>(map_db->next_landmark_line_id_)}};

    std::ofstream ofs(path, std::ios::out | std::ios::binary);

    if (ofs.is_open()) {
        AINFO << "save the MessagePack file of database to " << path;
        const auto msgpack = nlohmann::json::to_msgpack(json);
        ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
        ofs.close();
        return true;
    }
    else {
        AFATAL << "cannot create a file at " << path;
        return false;
    }
}

bool map_database_io_msgpack::load(const std::string& path,
                                   data::camera_database* cam_db,
                                   data::orb_params_database* orb_params_db,
                                   data::map_database* map_db,
                                   data::bow_database* bow_db,
                                   data::bow_vocabulary* bow_vocab) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);
    assert(cam_db && orb_params_db && map_db && bow_db && bow_vocab);

    // load binary bytes

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        AFATAL << "cannot load the file at " << path;
        return false;
    }

    AINFO << "load the MessagePack file of database from " << path;
    std::vector<uint8_t> msgpack;
    while (true) {
        uint8_t buffer;
        ifs.read(reinterpret_cast<char*>(&buffer), sizeof(uint8_t));
        if (ifs.eof()) {
            break;
        }
        msgpack.push_back(buffer);
    }
    ifs.close();

    // parse into JSON

    const auto json = nlohmann::json::from_msgpack(msgpack);

    // load database
    const auto json_cameras = json.at("cameras");
    cam_db->from_json(json_cameras);
    const auto json_orb_params = json.at("orb_params");
    orb_params_db->from_json(json_orb_params);
    const auto json_keyfrms = json.at("keyframes");
    const auto json_landmarks = json.at("landmarks");
    map_db->from_json(cam_db, orb_params_db, bow_vocab, json_keyfrms, json_landmarks);
    if (json.contains("landmark_planes")) {
        map_db->from_json_planes(json.at("landmark_planes"));
    }
    if (json.contains("landmarks_line")) {
        map_db->from_json_lines(json.at("landmarks_line"));
        const auto keyfrm_base = map_db->next_keyframe_id_.load();
        for (const auto& item : json_keyfrms.items()) {
            if (!item.value().contains("lm_line_ids")) {
                continue;
            }
            const auto keyfrm_id = std::stoul(item.key()) + keyfrm_base;
            map_db->register_association_line(keyfrm_id, item.value());
        }
    }
    // load next ID
    map_db->next_keyframe_id_ += json.at("keyframe_next_id").get<unsigned int>();
    map_db->next_landmark_id_ += json.at("landmark_next_id").get<unsigned int>();
    if (json.contains("landmark_plane_next_id")) {
        map_db->next_landmark_plane_id_ += json.at("landmark_plane_next_id").get<unsigned int>();
    }
    if (json.contains("landmark_line_next_id")) {
        map_db->next_landmark_line_id_ += json.at("landmark_line_next_id").get<unsigned int>();
    }

    // update bow database
    const auto keyfrms = map_db->get_all_keyframes();
    for (const auto& keyfrm : keyfrms) {
        bow_db->add_keyframe(keyfrm);
    }
    return true;
}

} // namespace io
}  // namespace autonomy::localization::atlas
