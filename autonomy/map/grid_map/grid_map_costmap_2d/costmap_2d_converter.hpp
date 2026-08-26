/*
 * Costmap2DConverter.hpp
 *
 * Adapted for autonomy::map::costmap_2d (ROS-free).
 * Based on ANYbotics/grid_map grid_map_costmap_2d.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map_core.hpp"

namespace grid_map {

/**
 * Defines the conversion between grid_map and costmap_2d.
 */
template <int64_t noInformation, int64_t lethalObstacle,
          int64_t inscribedInflatedObstacle, int64_t freeSpace>
class Costmap2DTranslationTable {
  static_assert(freeSpace < inscribedInflatedObstacle,
                "[Costmap2DTranslationTable] freeSpace < "
                "inscribedInflatedObstacle violated.");
  static_assert(inscribedInflatedObstacle < lethalObstacle,
                "[Costmap2DTranslationTable] inscribedInflatedObstacle < "
                "lethalObstacle violated.");
  static_assert(noInformation < freeSpace || noInformation > lethalObstacle,
                "[Costmap2DTranslationTable] noInformation placement "
                "violated.");

 public:
  Costmap2DTranslationTable() = delete;

  template <typename DataType>
  static void create(std::vector<DataType>& costTranslationTable) {
    costTranslationTable.resize(256);
    for (unsigned int i = 0; i < costTranslationTable.size(); ++i) {
      costTranslationTable[i] = fromCostmap<DataType>(static_cast<uint8_t>(i));
    }
  }

  template <typename DataType>
  static DataType fromCostmap(const uint8_t costmapValue) {
    using autonomy::map::costmap_2d::FREE_SPACE;
    using autonomy::map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    using autonomy::map::costmap_2d::LETHAL_OBSTACLE;
    using autonomy::map::costmap_2d::NO_INFORMATION;

    if (costmapValue == FREE_SPACE) {
      return freeSpace;
    }
    if (costmapValue == INSCRIBED_INFLATED_OBSTACLE) {
      return inscribedInflatedObstacle;
    }
    if (costmapValue == LETHAL_OBSTACLE) {
      return lethalObstacle;
    }
    if (costmapValue == NO_INFORMATION) {
      return noInformation;
    }

    constexpr DataType costmapIntervalStart = FREE_SPACE;
    constexpr DataType costmapIntervalWidth =
        INSCRIBED_INFLATED_OBSTACLE - costmapIntervalStart;
    constexpr DataType gridmapIntervalStart = freeSpace;
    constexpr DataType gridmapIntervalWidth =
        inscribedInflatedObstacle - gridmapIntervalStart;
    return gridmapIntervalStart +
           (costmapValue - costmapIntervalStart) * gridmapIntervalWidth /
               costmapIntervalWidth;
  }

  template <typename DataType>
  static uint8_t toCostmap(const DataType gridmapValue) {
    using autonomy::map::costmap_2d::FREE_SPACE;
    using autonomy::map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    using autonomy::map::costmap_2d::LETHAL_OBSTACLE;
    using autonomy::map::costmap_2d::NO_INFORMATION;

    if (gridmapValue == static_cast<DataType>(noInformation)) {
      return NO_INFORMATION;
    }
    if (gridmapValue <= static_cast<DataType>(freeSpace)) {
      return FREE_SPACE;
    }
    if (gridmapValue >= static_cast<DataType>(lethalObstacle)) {
      return LETHAL_OBSTACLE;
    }
    if (gridmapValue >= static_cast<DataType>(inscribedInflatedObstacle)) {
      return INSCRIBED_INFLATED_OBSTACLE;
    }

    constexpr DataType costmapIntervalStart = FREE_SPACE;
    constexpr DataType costmapIntervalWidth =
        INSCRIBED_INFLATED_OBSTACLE - costmapIntervalStart;
    constexpr DataType gridmapIntervalStart = freeSpace;
    constexpr DataType gridmapIntervalWidth =
        inscribedInflatedObstacle - gridmapIntervalStart;
    const DataType interpolatedValue =
        costmapIntervalStart +
        (gridmapValue - gridmapIntervalStart) * costmapIntervalWidth /
            gridmapIntervalWidth;
    return static_cast<uint8_t>(std::round(interpolatedValue));
  }
};

using Costmap2DDirectTranslationTable = Costmap2DTranslationTable<
    autonomy::map::costmap_2d::NO_INFORMATION,
    autonomy::map::costmap_2d::LETHAL_OBSTACLE,
    autonomy::map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
    autonomy::map::costmap_2d::FREE_SPACE>;

using Costmap2DCenturyTranslationTable =
    Costmap2DTranslationTable<-1, 100, 99,
                              autonomy::map::costmap_2d::FREE_SPACE>;

template <typename DataType>
class Costmap2DDefaultTranslationTable
    : public Costmap2DDirectTranslationTable {};

template <>
class Costmap2DDefaultTranslationTable<float>
    : public Costmap2DCenturyTranslationTable {};

/**
 * Convert between grid_map::GridMap and autonomy::map::costmap_2d::Costmap2D.
 */
template <typename MapType,
          typename TranslationTable =
              Costmap2DDefaultTranslationTable<typename MapType::DataType>>
class Costmap2DConverter {
 public:
  using DataType = typename MapType::DataType;
  using Costmap2D = autonomy::map::costmap_2d::Costmap2D;

  Costmap2DConverter() = default;
  ~Costmap2DConverter() = default;

  void initializeFromGridMap(const MapType& gridMap, Costmap2D& outputCostmap) {
    const Position position =
        gridMap.getPosition() - Position(0.5 * gridMap.getLength());
    const Size sizeXY = gridMap.getSize();
    outputCostmap.resizeMap(sizeXY(0), sizeXY(1), gridMap.getResolution(),
                            position(0), position(1));
  }

  bool setCostmap2DFromGridMap(const MapType& gridMap, const std::string& layer,
                               Costmap2D& outputCostmap) {
    Size size(outputCostmap.getSizeInCellsX(),
              outputCostmap.getSizeInCellsY());
    if ((gridMap.getSize() != size).any()) {
      errorMessage_ = "Costmap2D and output map have different sizes!";
      return false;
    }
    if (!gridMap.getStartIndex().isZero()) {
      errorMessage_ = "Does not support non-zero start indices!";
      return false;
    }
    const size_t nCells = static_cast<size_t>(gridMap.getSize().prod());
    for (size_t i = 0, j = nCells - 1; i < nCells; ++i, --j) {
      outputCostmap.getCharMap()[j] =
          TranslationTable::template toCostmap<DataType>(
              gridMap.get(layer).data()[i]);
    }
    return true;
  }

  void initializeFromCostmap2D(const Costmap2D& costmap2d, MapType& outputMap) {
    const double resolution = costmap2d.getResolution();
    Length length(costmap2d.getSizeInCellsX() * resolution,
                  costmap2d.getSizeInCellsY() * resolution);
    Position position(costmap2d.getOriginX(), costmap2d.getOriginY());
    position += Position(0.5 * length);
    outputMap.setGeometry(length, resolution, position);
  }

  bool addLayerFromCostmap2D(const Costmap2D& costmap2d,
                             const std::string& layer, MapType& outputMap) {
    Size size(costmap2d.getSizeInCellsX(), costmap2d.getSizeInCellsY());
    if ((outputMap.getSize() != size).any()) {
      errorMessage_ = "Costmap2D and output map have different sizes!";
      return false;
    }
    if (!outputMap.getStartIndex().isZero()) {
      errorMessage_ = "Does not support non-zero start indices!";
      return false;
    }
    typename MapType::Matrix data(size(0), size(1));
    const size_t nCells = static_cast<size_t>(outputMap.getSize().prod());
    for (size_t i = 0, j = nCells - 1; i < nCells; ++i, --j) {
      const unsigned char cost = costmap2d.getCharMap()[j];
      data(static_cast<Eigen::Index>(i)) =
          TranslationTable::template fromCostmap<DataType>(cost);
    }
    outputMap.add(layer, data);
    return true;
  }

  const std::string& errorMessage() const { return errorMessage_; }

 private:
  std::string errorMessage_;
};

}  // namespace grid_map
