/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>

#include <QString>

#include "autolink/parameter/parameter.hpp"
#include "autolink/proto/parameter.pb.h"

namespace autoviz {
namespace parameters_panel {

bool IsEditableParameterType(autolink::proto::ParamType type);

QString FormatParameterTypeName(const autolink::Parameter& parameter);
QString FormatParameterValue(const autolink::Parameter& parameter);

/** Parses edited text into a Parameter with the same name/type as template_param. */
std::optional<autolink::Parameter> ParseEditedParameterValue(
    const autolink::Parameter& template_param, const QString& text,
    QString* error_message);

}  // namespace parameters_panel
}  // namespace autoviz
