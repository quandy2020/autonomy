# Copyright 2026 Autodriver contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# MkDocs: pip install -r docs/requirements.txt
# Preview: cd docs && mkdocs serve
# Build: cmake --build build --target docs
set(DOCS_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/site")
find_program(MkDocs_EXECUTABLE mkdocs)
if(MkDocs_EXECUTABLE)
  add_custom_target(docs
    COMMAND "${MkDocs_EXECUTABLE}" build -d "${DOCS_OUTPUT_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    BYPRODUCTS "${DOCS_OUTPUT_DIR}"
    COMMENT "Building autodriver docs with MkDocs -> ${DOCS_OUTPUT_DIR}"
  )
else()
  message(STATUS "mkdocs not found; docs target disabled. Install: pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")
endif()
