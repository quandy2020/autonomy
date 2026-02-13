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

# If required theme isn't available, skip docs to avoid build failures.
execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import sphinx_rtd_theme"
    RESULT_VARIABLE SPHINX_RTD_THEME_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
)
if(NOT SPHINX_RTD_THEME_RESULT EQUAL 0)
    message(WARNING "sphinx_rtd_theme not found; skipping docs build target.")
    return()
endif()

# Build HTML documentation (NOT built by default)
add_custom_target(build_doc
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
# build_doc 目标默认不构建，可能导致 ${SPHINX_HTML_DIR} 不存在，从而在 `make install` 阶段失败。
# 这里提前创建目录，保证安装阶段不会因为缺失目录而报错（即使目录为空也可正常安装）。
file(MAKE_DIRECTORY "${SPHINX_HTML_DIR}")
install(DIRECTORY "${SPHINX_HTML_DIR}/" DESTINATION docs)
if(HTML2PDF_SCRIPT AND EXISTS ${SPHINX_PDF_OUTPUT})
    install(FILES ${SPHINX_PDF_OUTPUT} DESTINATION docs)
endif()
