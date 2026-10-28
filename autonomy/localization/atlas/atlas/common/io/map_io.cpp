#include "atlas/common/io/map_io.hpp"

#include <exception>
#include <fstream>

#include "atlas/common/io/binary_archive.hpp"
#include "atlas/common/logging.hpp"
#include "atlas/map/atlas.hpp"

namespace autonomy::localization::atlas {

bool MapIO::SaveBinary(const std::string& path, Atlas* atlas) {
    if (atlas == nullptr || path.empty())
        return false;
    try {
        atlas->PreSave();
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        if (!output)
            return false;
        BinaryOutputArchive archive(output);
        archive << atlas;
        return static_cast<bool>(output);
    } catch (const std::exception& error) {
        AERROR << "failed to save Atlas map: " << error.what() << std::endl;
        return false;
    }
}

bool MapIO::LoadBinary(const std::string& path, Atlas** atlas,
                       PlaceRecognizer* place_recognizer,
                       Vocabulary* vocabulary) {
    if (atlas == nullptr || path.empty() || place_recognizer == nullptr ||
        vocabulary == nullptr) {
        return false;
    }
    try {
        std::ifstream input(path, std::ios::binary);
        if (!input)
            return false;
        BinaryInputArchive archive(input);
        archive >> *atlas;
        if (*atlas == nullptr)
            return false;
        (*atlas)->SetPlaceRecognizer(place_recognizer);
        (*atlas)->SetVocabulary(vocabulary);
        (*atlas)->PostLoad();
        return true;
    } catch (const std::exception& error) {
        AERROR << "failed to load Atlas map: " << error.what() << std::endl;
        return false;
    }
}

}  // namespace autonomy::localization::atlas
