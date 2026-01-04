# Copyright 2016 The OpenRobotic Beginner Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# HTML output directory
set(SPHINX_HTML_DIR "${PROJECT_BINARY_DIR}/docs/html")
set(SPHINX_PDF_OUTPUT "${PROJECT_BINARY_DIR}/docs/Autonomy.pdf")

# Find python
find_package(Python REQUIRED COMPONENTS Interpreter)

# Build HTML documentation
add_custom_target(build_doc ALL
    ${SPHINX_EXECUTABLE} -b html
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${CMAKE_CURRENT_BINARY_DIR}/html
    COMMENT "Building HTML documentation."
)

# Build PDF documentation from HTML (optional, requires html2pdf.py and PDF converter)
find_program(HTML2PDF_SCRIPT
    NAMES html2pdf.py
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}
    NO_DEFAULT_PATH
)

if(HTML2PDF_SCRIPT)
    add_custom_target(build_doc_pdf
        COMMAND ${Python_EXECUTABLE} ${HTML2PDF_SCRIPT}
            ${CMAKE_CURRENT_BINARY_DIR}/html
            ${SPHINX_PDF_OUTPUT}
        DEPENDS build_doc
        COMMENT "Converting HTML documentation to PDF."
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    message(STATUS "PDF documentation target available (html2pdf.py found)")
    message(STATUS "Note: Install playwright or weasyprint to enable PDF conversion:")
    message(STATUS "  pip install playwright && playwright install chromium")
    message(STATUS "  or: pip install weasyprint")
else()
    message(WARNING "html2pdf.py not found. PDF documentation target will not be available.")
endif()

# Install documentation
install(DIRECTORY ${PROJECT_BINARY_DIR}/docs/html/ DESTINATION docs)
if(HTML2PDF_SCRIPT AND EXISTS ${SPHINX_PDF_OUTPUT})
    install(FILES ${SPHINX_PDF_OUTPUT} DESTINATION docs)
endif()
