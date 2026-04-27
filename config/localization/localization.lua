-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "localization/amcl/amcl.lua"

AUTONOMY_LOCALIZATION = {
    -- Default localization algorithm to use
    -- Options: "amcl", "cartographer", etc.
    default_algorithm = "amcl",
    
    -- Enable/disable localization
    enabled = true,
    
    -- AMCL configuration
    -- 对应 proto: autonomy.localization.proto.AmclOptions
    amcl = AMCL_OPTIONS,
}

-- 返回一个包含 AUTONOMY_LOCALIZATION 的表
return {
    AUTONOMY_LOCALIZATION = AUTONOMY_LOCALIZATION
}