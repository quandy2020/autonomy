#pragma once

#include <string>

namespace autonomy::localization::atlas {

class Atlas;
class PlaceRecognizer;
class Vocabulary;

/** @brief C++17 binary persistence helpers for complete Atlas object graphs. */
class MapIO
{
public:
    /**
     * @brief Saves @p atlas to @p path after preparing pointer backup tables.
     * @return false on invalid arguments or any I/O/serialization failure.
     */
    static bool SaveBinary(const std::string& path, Atlas* atlas);
    /**
     * @brief Loads an Atlas and reconnects runtime-only recognizer/vocabulary.
     * @param atlas Receives ownership of the newly allocated Atlas.
     * @return false when the archive is invalid or cannot be read.
     */
    static bool LoadBinary(const std::string& path, Atlas** atlas,
                           PlaceRecognizer* place_recognizer,
                           Vocabulary* vocabulary);
};

}  // namespace autonomy::localization::atlas
