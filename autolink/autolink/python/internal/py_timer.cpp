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

#include "autolink/python/internal/py_timer.hpp"

#include <set>
#include <string>

#include <Python.h>

using autolink::PyTimer;

#define PyInt_AsLong PyLong_AsLong

template <typename T>
T PyObjectToPtr(PyObject* pyobj, const std::string& type_ptr) {
    T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
    if (obj_ptr == nullptr) {
        AERROR << "PyObjectToPtr failed,type->" << type_ptr
               << "pyobj: " << pyobj;
    }
    return obj_ptr;
}

PyObject* autolink_new_PyTimer(PyObject* self, PyObject* args) {
    uint32_t period = 0;
    PyObject* pyobj_regist_fun = nullptr;
    unsigned int oneshot = 0;
    if (!PyArg_ParseTuple(args, const_cast<char*>("kOI:autolink_new_PyTimer"),
                          &period, &pyobj_regist_fun, &oneshot)) {
        AERROR << "autolink_new_PyTimer parsetuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    void (*callback_fun)() = (void (*)())0;
    callback_fun = (void (*)())PyInt_AsLong(pyobj_regist_fun);
    PyTimer* pytimer = new PyTimer(period, callback_fun, oneshot != 0);
    return PyCapsule_New(pytimer, "autolink_autolinktron_pytimer", nullptr);
}

PyObject* autolink_new_PyTimer_noparam(PyObject* self, PyObject* args) {
    PyTimer* pytimer = new PyTimer();
    return PyCapsule_New(pytimer, "autolink_autolinktron_pytimer", nullptr);
}

PyObject* autolink_delete_PyTimer(PyObject* self, PyObject* args) {
    PyObject* pyobj_timer = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_delete_PyTimer"),
                          &pyobj_timer)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pytimer = reinterpret_cast<PyTimer*>(
        PyCapsule_GetPointer(pyobj_timer, "autolink_autolinktron_pytimer"));
    if (nullptr == pytimer) {
        AERROR << "autolink_delete_PyTimer:timer ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    delete pytimer;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyTimer_start(PyObject* self, PyObject* args) {
    PyObject* pyobj_timer = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_delete_PyTimer"),
                          &pyobj_timer)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pytimer = reinterpret_cast<PyTimer*>(
        PyCapsule_GetPointer(pyobj_timer, "autolink_autolinktron_pytimer"));
    if (nullptr == pytimer) {
        AERROR << "autolink_delete_PyTimer:timer ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    pytimer->start();
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyTimer_stop(PyObject* self, PyObject* args) {
    PyObject* pyobj_timer = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_delete_PyTimer"),
                          &pyobj_timer)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    auto* pytimer = reinterpret_cast<PyTimer*>(
        PyCapsule_GetPointer(pyobj_timer, "autolink_autolinktron_pytimer"));
    if (nullptr == pytimer) {
        AERROR << "autolink_delete_PyTimer:timer ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    pytimer->stop();
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyTimer_set_option(PyObject* self, PyObject* args) {
    PyObject* pyobj_timer = nullptr;
    uint32_t period = 0;
    PyObject* pyobj_regist_fun = nullptr;
    unsigned int oneshot = 0;

    void (*callback_fun)() = (void (*)())0;

    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("OkOI:autolink_PyTimer_set_option"),
                          &pyobj_timer, &period, &pyobj_regist_fun, &oneshot)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyTimer* pytimer =
        PyObjectToPtr<PyTimer*>(pyobj_timer, "autolink_autolinktron_pytimer");
    callback_fun = (void (*)())PyInt_AsLong(pyobj_regist_fun);
    if (nullptr == pytimer) {
        AERROR << "autolink_PyTimer_set_option ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    pytimer->set_option(period, callback_fun, oneshot != 0);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyMethodDef _autolink_timer_methods[] = {
    {"new_PyTimer_noparam", autolink_new_PyTimer_noparam, METH_NOARGS, ""},
    {"new_PyTimer", autolink_new_PyTimer, METH_VARARGS, ""},
    {"delete_PyTimer", autolink_delete_PyTimer, METH_VARARGS, ""},
    {"PyTimer_start", autolink_PyTimer_start, METH_VARARGS, ""},
    {"PyTimer_stop", autolink_PyTimer_stop, METH_VARARGS, ""},
    {"PyTimer_set_option", autolink_PyTimer_set_option, METH_VARARGS, ""},

    {nullptr, nullptr, 0, nullptr} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC PyInit__autolink_timer_wrapper(void) {
    static struct PyModuleDef module_def = {
        PyModuleDef_HEAD_INIT,
        "_autolink_timer_wrapper",  // Module name.
        "CyberTimer module",        // Module doc.
        -1,                         // Module size.
        _autolink_timer_methods,    // Module methods.
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    };

    return PyModule_Create(&module_def);
}
