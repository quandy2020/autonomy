/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
 
#include "autonomy/localization/amcl/map/map.hpp"

namespace autonomy {
namespace localization {
namespace amcl {
namespace map {

// // Create a new map
// map_t * map_alloc(void)
// {
//   map_t * map;

//   map = (map_t *) malloc(sizeof(map_t));

//   // Assume we start at (0, 0)
//   map->origin_x = 0;
//   map->origin_y = 0;

//   // Make the size odd
//   map->size_x = 0;
//   map->size_y = 0;
//   map->scale = 0;

//   // Allocate storage for main map
//   map->cells = (map_cell_t *) NULL;

//   return map;
// }

// // Destroy a map
// void map_free(map_t * map)
// {
//   free(map->cells);
//   free(map);
// }

}   // namespace map
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy