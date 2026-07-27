/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_catalog.hpp"

#include <iterator>

namespace autoviz {

namespace {

const PanelCatalogEntry kPanelCatalog[] = {
    {"DisplaysDock", "Displays", "Displays",
     "Show and edit the list of Displays"},
    {"StrataFloorDock", "Displays", "Strata Floors",
     "Select and toggle Strata floor visibility"},
    {"SelectionDock", "Selection", "Selection",
     "Show properties of selected objects"},
    {"ToolPropertiesDock", "ToolProperties", "Tool Properties",
     "Show and edit properties of tools"},
    {"ViewsDock", "Views", "Views", "Show and edit viewpoints"},
    {"TimeDock", "Time", "Time", "Show the current time"},
    {"PlaybackDock", "Playback", "Playback",
     "Replay Autolink .record files and seek"},
    {"ChannelsDock", "Channels", "Autolink Channels",
     "List active Autolink channels"},
    {"TfTreeDock", "TfTree", "TF Tree", "Show the transform tree"},
    {"TransformationDock", "Transformation", "Transformation",
     "Select the transformation plugin"},
    {"ImageDock", "Image", "Image", "Show image from Image display"},
#ifdef AUTOVIZ_USE_QML_DRONE
    {"Drone3DDock", "Drone3D", "Drone 3D",
     "QGC-style Qt Quick 3D drone preview from TF"},
#endif
    {"HelpDock", "Help", "Help", "Show the key and mouse bindings"},
};

}  // namespace

QVector<PanelCatalogEntry> PanelCatalog() {
  return {std::begin(kPanelCatalog), std::end(kPanelCatalog)};
}

}  // namespace autoviz
