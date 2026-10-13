/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_xml_io.hpp
 * @brief Import / export @ref PlotPanelConfig as rqt_multiplot-compatible XML.
 */

#pragma once

#include <QByteArray>
#include <QString>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

/**
 * @brief Serializes @p config to an XML document.
 *
 * @param config Panel configuration.
 * @return UTF-8 XML bytes.
 */
QByteArray ExportPlotXml(const PlotPanelConfig& config);

/**
 * @brief Parses plot XML into @p config.
 *
 * Accepts Autoviz exports and a subset of rqt_multiplot curve configs.
 *
 * @param xml UTF-8 XML document.
 * @param config Out-parameter for the imported panel config.
 * @param error Optional human-readable error.
 * @return @c true on success.
 */
bool ImportPlotXml(const QByteArray& xml, PlotPanelConfig* config,
                   QString* error);

}  // namespace plot
}  // namespace autoviz
