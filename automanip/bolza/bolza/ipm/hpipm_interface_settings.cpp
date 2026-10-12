/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bolza/ipm/hpipm_interface_settings.hpp"

#include "bolza/core/misc/load_data.hpp"

namespace bolza {
namespace ipm {

std::ostream& operator<<(std::ostream& stream, const HpipmSettings& settings) {
  HpipmSettings defaultSettings;

  stream << "\n #### HPIPM Settings:";
  stream << "\n #### =============================================================================\n";
  loadData::PrintValue(stream, settings.hpipmMode, "mode", settings.hpipmMode != defaultSettings.hpipmMode);
  loadData::PrintValue(stream, settings.iter_max, "iter_max", settings.iter_max != defaultSettings.iter_max);
  loadData::PrintValue(stream, settings.alpha_min, "alpha_min", settings.alpha_min != defaultSettings.alpha_min);
  loadData::PrintValue(stream, settings.mu0, "mu0", settings.mu0 != defaultSettings.mu0);
  loadData::PrintValue(stream, settings.tol_stat, "tol_stat", settings.tol_stat != defaultSettings.tol_stat);
  loadData::PrintValue(stream, settings.tol_eq, "tol_eq", settings.tol_eq != defaultSettings.tol_eq);
  loadData::PrintValue(stream, settings.tol_ineq, "tol_ineq", settings.tol_ineq != defaultSettings.tol_ineq);
  loadData::PrintValue(stream, settings.tol_comp, "tol_comp", settings.tol_comp != defaultSettings.tol_comp);
  loadData::PrintValue(stream, settings.reg_prim, "reg_prim", settings.reg_prim != defaultSettings.reg_prim);
  loadData::PrintValue(stream, settings.warm_start, "warm_start", settings.warm_start != defaultSettings.warm_start);
  loadData::PrintValue(stream, settings.pred_corr, "pred_corr", settings.pred_corr != defaultSettings.pred_corr);
  loadData::PrintValue(stream, settings.ric_alg, "ric_alg", settings.ric_alg != defaultSettings.ric_alg);
  stream << " #### =============================================================================" << std::endl;
  return stream;
}

}  // namespace ipm
}  // namespace bolza