/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

namespace autoviz {
namespace variables {
class VariableStore;
}  // namespace variables

namespace plot {

/** Apply global variable substitution to a message path expression. */
QString ResolveMessagePath(const QString& field_path,
                           const variables::VariableStore* store);

/** Backward-compatible alias used by Plot panels. */
QString ResolvePlotFieldPath(const QString& field_path,
                             const variables::VariableStore* store);

}  // namespace plot
}  // namespace autoviz
