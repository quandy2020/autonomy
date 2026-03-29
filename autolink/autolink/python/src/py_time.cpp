/**
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

#include "autolink/python/internal/py_time.hpp"

#include <set>
#include <string>

#include <Python.h>

using autolink::PyDuration;
using autolink::PyRate;
using autolink::PyTime;

#define PYOBJECT_NULL_STRING PyString_FromStringAndSize("", 0)

template <typename T>
T PyObjectToPtr(PyObject* pyobj, const std::string& type_ptr) {
    T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
    if (obj_ptr == nullptr) {
        AERROR << "PyObjectToPtr failed,type->" << type_ptr
               << "pyobj: " << pyobj;
    }
    return obj_ptr;
}

PyObject* autolink_new_PyTime(PyObject* self, PyObject* args) {
    uint64_t nanoseconds = 0;
    if (!PyArg_ParseTuple(args, const_cast<char*>("K:autolink_new_PyTime"),
                          &nanoseconds)) {
        AERROR << "autolink_new_PyTime parsetuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyTime* pytime = new PyTime(nanoseconds);
    return PyCapsule_New(pytime, "autolink_pytime", nullptr);
}

PyObject* autolink_delete_PyTime(PyObject* self, PyObject* args) {
    PyObject* pyobj_time = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_delete_PyTime"),
                          &pyobj_time)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pytime = reinterpret_cast<PyTime*>(
        PyCapsule_GetPointer(pyobj_time, "autolink_pytime"));
    if (nullptr == pytime) {
        AERROR << "autolink_delete_PyTime:time ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    delete pytime;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyTime_to_sec(PyObject* self, PyObject* args) {
    PyObject* pyobj_time = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_PyTime_to_sec"),
                          &pyobj_time)) {
        AERROR << "autolink_PyTime_to_sec:PyArg_ParseTuple failed!";
        return PyFloat_FromDouble(0);
    }

    auto* pytime = reinterpret_cast<PyTime*>(
        PyCapsule_GetPointer(pyobj_time, "autolink_pytime"));
    if (nullptr == pytime) {
        AERROR << "autolink_PyTime_to_sec ptr is null!";
        return PyFloat_FromDouble(0);
    }

    double num = pytime->to_sec();
    return PyFloat_FromDouble(num);
}

PyObject* autolink_PyTime_to_nsec(PyObject* self, PyObject* args) {
    PyObject* pyobj_time = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_PyTime_to_nsec"),
                          &pyobj_time)) {
        AERROR << "autolink_PyTime_to_nsec:PyArg_ParseTuple failed!";
        return PyLong_FromUnsignedLongLong(0);
    }

    auto* pytime = reinterpret_cast<PyTime*>(
        PyCapsule_GetPointer(pyobj_time, "autolink_pytime"));
    if (nullptr == pytime) {
        AERROR << "autolink_PyTime_to_nsec ptr is null!";
        return PyLong_FromUnsignedLongLong(0);
    }

    uint64_t num = pytime->to_nsec();
    return PyLong_FromUnsignedLongLong(num);
}

PyObject* autolink_PyTime_sleep_until(PyObject* self, PyObject* args) {
    PyObject* pyobj_time = nullptr;
    uint64_t nanoseconds = 0;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("OK:autolink_PyTime_sleep_until"),
                          &pyobj_time, &nanoseconds)) {
        AERROR << "autolink_PyTime_sleep_until:PyArg_ParseTuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pytime = reinterpret_cast<PyTime*>(
        PyCapsule_GetPointer(pyobj_time, "autolink_pytime"));
    if (nullptr == pytime) {
        AERROR << "autolink_PyTime_sleep_until ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    pytime->sleep_until(nanoseconds);
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyTime_now(PyObject* self, PyObject* args) {
    PyTime now = PyTime::now();
    return PyLong_FromUnsignedLongLong(now.to_nsec());
}

PyObject* autolink_PyTime_mono_time(PyObject* self, PyObject* args) {
    PyTime mono_time = PyTime::mono_time();
    return PyLong_FromUnsignedLongLong(mono_time.to_nsec());
}

// duration
PyObject* autolink_new_PyDuration(PyObject* self, PyObject* args) {
    uint64_t nanoseconds = 0;
    if (!PyArg_ParseTuple(args, const_cast<char*>("L:autolink_new_PyDuration"),
                          &nanoseconds)) {
        AERROR << "autolink_new_PyDuration parsetuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyDuration* pyduration = new PyDuration(nanoseconds);
    return PyCapsule_New(pyduration, "autolink_pyduration", nullptr);
}

PyObject* autolink_delete_PyDuration(PyObject* self, PyObject* args) {
    PyObject* pyobj_duration = nullptr;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("O:autolink_delete_PyDuration"),
                          &pyobj_duration)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pyduration = reinterpret_cast<PyDuration*>(
        PyCapsule_GetPointer(pyobj_duration, "autolink_pyduration"));
    if (nullptr == pyduration) {
        AERROR << "autolink_delete_PyDuration:pyduration ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    delete pyduration;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyDuration_sleep(PyObject* self, PyObject* args) {
    PyObject* pyobj_duration = nullptr;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("O:autolink_PyDuration_sleep"),
                          &pyobj_duration)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pyduration = reinterpret_cast<PyDuration*>(
        PyCapsule_GetPointer(pyobj_duration, "autolink_pyduration"));
    if (nullptr == pyduration) {
        AERROR << "autolink_PyDuration_sleep:pyduration ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    pyduration->sleep();
    Py_INCREF(Py_None);
    return Py_None;
}

// rate
PyObject* autolink_new_PyRate(PyObject* self, PyObject* args) {
    uint64_t nanoseconds = 0;
    if (!PyArg_ParseTuple(args, const_cast<char*>("L:autolink_new_PyRate"),
                          &nanoseconds)) {
        AERROR << "autolink_new_PyRate parsetuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyRate* pyrate = new PyRate(nanoseconds);
    return PyCapsule_New(pyrate, "autolink_pyrate", nullptr);
}

PyObject* autolink_delete_PyRate(PyObject* self, PyObject* args) {
    PyObject* pyobj_rate = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_delete_PyRate"),
                          &pyobj_rate)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pyrate = reinterpret_cast<PyRate*>(
        PyCapsule_GetPointer(pyobj_rate, "autolink_pyrate"));
    if (nullptr == pyrate) {
        AERROR << "autolink_delete_PyRate:rate ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    delete pyrate;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyRate_sleep(PyObject* self, PyObject* args) {
    PyObject* pyobj_rate = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_PyRate_sleep"),
                          &pyobj_rate)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pyrate = reinterpret_cast<PyRate*>(
        PyCapsule_GetPointer(pyobj_rate, "autolink_pyrate"));
    if (nullptr == pyrate) {
        AERROR << "autolink_PyRate_sleep:rate ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    pyrate->sleep();
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyRate_reset(PyObject* self, PyObject* args) {
    PyObject* pyobj_rate = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_PyRate_reset"),
                          &pyobj_rate)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pyrate = reinterpret_cast<PyRate*>(
        PyCapsule_GetPointer(pyobj_rate, "autolink_pyrate"));
    if (nullptr == pyrate) {
        AERROR << "autolink_PyRate_reset:rate ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    pyrate->reset();
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyRate_get_cycle_time(PyObject* self, PyObject* args) {
    PyObject* pyobj_rate = nullptr;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("O:autolink_PyRate_get_cycle_time"),
                          &pyobj_rate)) {
        return PyLong_FromUnsignedLongLong(0);
    }

    auto* pyrate = reinterpret_cast<PyRate*>(
        PyCapsule_GetPointer(pyobj_rate, "autolink_pyrate"));
    if (nullptr == pyrate) {
        AERROR << "autolink_PyRate_get_cycle_time:rate ptr is null!";
        return PyLong_FromUnsignedLongLong(0);
    }
    uint64_t ctime = pyrate->get_cycle_time();
    return PyLong_FromUnsignedLongLong(ctime);
}

PyObject* autolink_PyRate_get_expected_cycle_time(PyObject* self,
                                                  PyObject* args) {
    PyObject* pyobj_rate = nullptr;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("O:autolink_PyRate_get_expected_cycle_time"),
            &pyobj_rate)) {
        return PyLong_FromUnsignedLongLong(0);
    }

    auto* pyrate = reinterpret_cast<PyRate*>(
        PyCapsule_GetPointer(pyobj_rate, "autolink_pyrate"));
    if (nullptr == pyrate) {
        AERROR << "autolink_PyRate_get_expected_cycle_time:rate ptr is null!";
        return PyLong_FromUnsignedLongLong(0);
    }
    uint64_t exp_cycle_time = pyrate->get_expected_cycle_time();
    return PyLong_FromUnsignedLongLong(exp_cycle_time);
}

static PyMethodDef _autolink_time_methods[] = {
    // PyTime fun
    {"new_PyTime", autolink_new_PyTime, METH_VARARGS, ""},
    {"delete_PyTime", autolink_delete_PyTime, METH_VARARGS, ""},
    {"PyTime_now", autolink_PyTime_now, METH_VARARGS, ""},
    {"PyTime_mono_time", autolink_PyTime_mono_time, METH_VARARGS, ""},
    {"PyTime_to_sec", autolink_PyTime_to_sec, METH_VARARGS, ""},
    {"PyTime_to_nsec", autolink_PyTime_to_nsec, METH_VARARGS, ""},
    {"PyTime_sleep_until", autolink_PyTime_sleep_until, METH_VARARGS, ""},

    // PyDuration
    {"new_PyDuration", autolink_new_PyDuration, METH_VARARGS, ""},
    {"delete_PyDuration", autolink_delete_PyDuration, METH_VARARGS, ""},
    {"PyDuration_sleep", autolink_PyDuration_sleep, METH_VARARGS, ""},

    // PyRate
    {"new_PyRate", autolink_new_PyRate, METH_VARARGS, ""},
    {"delete_PyRate", autolink_delete_PyRate, METH_VARARGS, ""},
    {"PyRate_sleep", autolink_PyRate_sleep, METH_VARARGS, ""},
    {"PyRate_reset", autolink_PyRate_reset, METH_VARARGS, ""},
    {"PyRate_get_cycle_time", autolink_PyRate_get_cycle_time, METH_VARARGS, ""},
    {"PyRate_get_expected_cycle_time", autolink_PyRate_get_expected_cycle_time,
     METH_VARARGS, ""},

    {nullptr, nullptr, 0, nullptr} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC PyInit__autolink_time_wrapper(void) {
    static struct PyModuleDef module_def = {
        PyModuleDef_HEAD_INIT,
        "_autolink_time_wrapper",  // Module name.
        "AutolinkTime module",     // Module doc.
        -1,                        // Module size.
        _autolink_time_methods,    // Module methods.
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    };

    return PyModule_Create(&module_def);
}