/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <pybind11/pybind11.h>

#include "autolink/common/log.hpp"

namespace autolink::python {

namespace py = pybind11;

inline void CallPyVoid(const std::shared_ptr<py::function>& callback) {
    py::gil_scoped_acquire gil;
    try {
        (*callback)();
    } catch (py::error_already_set& error) {
        AERROR << "Python callback failed: " << error.what();
        error.discard_as_unraisable(__func__);
    }
}

template <typename... Args>
inline void CallPy(const std::shared_ptr<py::function>& callback,
                   Args&&... args) {
    py::gil_scoped_acquire gil;
    try {
        (*callback)(std::forward<Args>(args)...);
    } catch (py::error_already_set& error) {
        AERROR << "Python callback failed: " << error.what();
        error.discard_as_unraisable(__func__);
    }
}

// Call Python with raw bytes; build py::bytes only while holding the GIL.
inline void CallPyBytes(const std::shared_ptr<py::function>& callback,
                        const std::string& data) {
    py::gil_scoped_acquire gil;
    try {
        (*callback)(py::bytes(data));
    } catch (py::error_already_set& error) {
        AERROR << "Python callback failed: " << error.what();
        error.discard_as_unraisable(__func__);
    }
}

template <typename... Args>
inline void CallPyBytesFirst(const std::shared_ptr<py::function>& callback,
                             const std::string& data, Args&&... args) {
    py::gil_scoped_acquire gil;
    try {
        (*callback)(py::bytes(data), std::forward<Args>(args)...);
    } catch (py::error_already_set& error) {
        AERROR << "Python callback failed: " << error.what();
        error.discard_as_unraisable(__func__);
    }
}

}  // namespace autolink::python
