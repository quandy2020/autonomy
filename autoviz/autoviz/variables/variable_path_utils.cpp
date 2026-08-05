/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/variables/variable_path_utils.hpp"

#include "autoviz/variables/variable_store.hpp"

namespace autoviz {
namespace plot {

QString ResolvePlotFieldPath(const QString& field_path,
                             const variables::VariableStore* store) {
  return ResolveMessagePath(field_path, store);
}

QString ResolveMessagePath(const QString& field_path,
                           const variables::VariableStore* store) {
  if (store == nullptr || !field_path.contains(QLatin1Char('$'))) {
    return field_path;
  }
  return store->substitute(field_path);
}

}  // namespace plot
}  // namespace autoviz
