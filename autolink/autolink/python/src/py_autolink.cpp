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

#define PY_SSIZE_T_CLEAN
#include "autolink/python/internal/py_autolink.hpp"

#include <string>
#include <vector>

#include <Python.h>

using autolink::Node;
using autolink::PyChannelUtils;
using autolink::PyClient;
using autolink::PyNode;
using autolink::PyReader;
using autolink::PyService;
using autolink::PyServiceUtils;
using autolink::PyWriter;

#define PyInt_AsLong PyLong_AsLong
#define PyInt_FromLong PyLong_FromLong
#define PYOBJECT_NULL_STRING PyBytes_FromStringAndSize("", 0)
#define C_STR_TO_PY_BYTES(cstr) \
    PyBytes_FromStringAndSize(cstr.c_str(), cstr.size())

google::protobuf::Message* PyChannelUtils::raw_msg_class_ = nullptr;

static PyObject* autolink_py_init(PyObject* self, PyObject* args) {
    char* data = nullptr;
    Py_ssize_t len = 0;
    if (!PyArg_ParseTuple(args, const_cast<char*>("s#:autolink_py_init"), &data,
                          &len)) {
        AERROR << "autolink_py_init:PyArg_ParseTuple failed!";
        Py_RETURN_FALSE;
    }
    std::string module_name(data, len);
    bool is_init = autolink::py_init(module_name);
    if (is_init) {
        Py_RETURN_TRUE;
    } else {
        Py_RETURN_FALSE;
    }
}

static PyObject* autolink_py_ok(PyObject* self, PyObject* args) {
    bool is_ok = autolink::py_ok();
    if (is_ok) {
        Py_RETURN_TRUE;
    } else {
        Py_RETURN_FALSE;
    }
}

static PyObject* autolink_py_shutdown(PyObject* self, PyObject* args) {
    autolink::py_shutdown();
    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* autolink_py_is_shutdown(PyObject* self, PyObject* args) {
    bool is_shutdown = autolink::py_is_shutdown();
    if (is_shutdown) {
        Py_RETURN_TRUE;
    } else {
        Py_RETURN_FALSE;
    }
}

static PyObject* autolink_py_waitforshutdown(PyObject* self, PyObject* args) {
    autolink::py_waitforshutdown();
    Py_INCREF(Py_None);
    return Py_None;
}

template <typename T>
T PyObjectToPtr(PyObject* pyobj, const std::string& type_ptr) {
    T obj_ptr = (T)PyCapsule_GetPointer(pyobj, type_ptr.c_str());
    if (obj_ptr == nullptr) {
        AERROR << "PyObjectToPtr failed,type->" << type_ptr << " "
               << "pyobj: " << pyobj;
    }
    return obj_ptr;
}

PyObject* autolink_new_PyWriter(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* data_type = nullptr;
    uint32_t qos_depth = 1;

    PyObject* node_pyobj = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("ssIO:new_PyWriter"),
                          &channel_name, &data_type, &qos_depth, &node_pyobj)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    Node* node = reinterpret_cast<Node*>(
        PyCapsule_GetPointer(node_pyobj, "autolink_autolink_pynode"));
    if (nullptr == node) {
        AERROR << "node is null";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyWriter* writer =
        new PyWriter((std::string const&)*channel_name,
                     (std::string const&)*data_type, qos_depth, node);
    PyObject* pyobj_writer =
        PyCapsule_New(writer, "autolink_autolink_pywriter", nullptr);
    return pyobj_writer;
}

PyObject* autolink_delete_PyWriter(PyObject* self, PyObject* args) {
    PyObject* writer_py = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:delete_PyWriter"),
                          &writer_py)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyWriter* writer = reinterpret_cast<PyWriter*>(
        PyCapsule_GetPointer(writer_py, "autolink_autolink_pywriter"));
    delete writer;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyWriter_write(PyObject* self, PyObject* args) {
    PyObject* pyobj_writer = nullptr;
    Py_buffer buffer;
    // Py_ssize_t len = 0;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("Oy*:autolink_PyWriter_write"),
                          &pyobj_writer, &buffer)) {
        AERROR << "autolink_PyWriter_write:autolink_PyWriter_write failed!";
        PyBuffer_Release(&buffer);
        return PyInt_FromLong(1);
    }

    PyWriter* writer =
        PyObjectToPtr<PyWriter*>(pyobj_writer, "autolink_autolink_pywriter");

    if (nullptr == writer) {
        AERROR << "autolink_PyWriter_write:writer ptr is null!";
        PyBuffer_Release(&buffer);
        return PyInt_FromLong(1);
    }

    std::string data_str(
        const_cast<const char*>(static_cast<char*>(buffer.buf)), buffer.len);
    int ret = writer->write(data_str);
    PyBuffer_Release(&buffer);
    return PyInt_FromLong(ret);
}

PyObject* autolink_new_PyReader(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* data_type = nullptr;
    PyObject* node_pyobj = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("ssO:new_PyReader"),
                          &channel_name, &data_type, &node_pyobj)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    Node* node = reinterpret_cast<Node*>(
        PyCapsule_GetPointer(node_pyobj, "autolink_autolink_pynode"));
    if (!node) {
        AERROR << "node is null";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyReader* reader = new PyReader((std::string const&)*channel_name,
                                    (std::string const&)*data_type, node);
    PyObject* pyobj_reader =
        PyCapsule_New(reader, "autolink_autolink_pyreader", nullptr);
    return pyobj_reader;
}

PyObject* autolink_delete_PyReader(PyObject* self, PyObject* args) {
    PyObject* reader_py = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:delete_PyReader"),
                          &reader_py)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyReader* reader = reinterpret_cast<PyReader*>(
        PyCapsule_GetPointer(reader_py, "autolink_autolink_pyreader"));
    delete reader;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyReader_read(PyObject* self, PyObject* args) {
    PyObject* pyobj_reader = nullptr;
    PyObject* pyobj_iswait = nullptr;

    if (!PyArg_ParseTuple(args, const_cast<char*>("OO:autolink_PyReader_read"),
                          &pyobj_reader, &pyobj_iswait)) {
        AERROR << "autolink_PyReader_read:PyArg_ParseTuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    PyReader* reader =
        PyObjectToPtr<PyReader*>(pyobj_reader, "autolink_autolink_pyreader");
    if (nullptr == reader) {
        AERROR << "autolink_PyReader_read:PyReader ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    int r = PyObject_IsTrue(pyobj_iswait);
    if (r == -1) {
        AERROR << "autolink_PyReader_read:pyobj_iswait is error!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    bool wait = (r == 1);

    const std::string reader_ret = reader->read(wait);
    return C_STR_TO_PY_BYTES(reader_ret);
}

PyObject* autolink_PyReader_register_func(PyObject* self, PyObject* args) {
    PyObject* pyobj_regist_fun = nullptr;
    PyObject* pyobj_reader = nullptr;

    int (*callback_fun)(char const*) = (int (*)(char const*))0;

    if (!PyArg_ParseTuple(args, const_cast<char*>("OO:PyReader_register_func"),
                          &pyobj_reader, &pyobj_regist_fun)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyReader* reader =
        PyObjectToPtr<PyReader*>(pyobj_reader, "autolink_autolink_pyreader");
    callback_fun = (int (*)(const char* i))PyInt_AsLong(pyobj_regist_fun);
    if (reader) {
        ADEBUG << "reader regist fun";
        reader->register_func(callback_fun);
    }

    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_new_PyClient(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* data_type = nullptr;
    PyObject* node_pyobj = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("ssO:new_PyClient"),
                          &channel_name, &data_type, &node_pyobj)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    Node* node = reinterpret_cast<Node*>(
        PyCapsule_GetPointer(node_pyobj, "autolink_autolink_pynode"));
    if (!node) {
        AERROR << "node is null";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyClient* client = new PyClient((std::string const&)*channel_name,
                                    (std::string const&)*data_type, node);
    PyObject* pyobj_client =
        PyCapsule_New(client, "autolink_autolink_pyclient", nullptr);
    return pyobj_client;
}

PyObject* autolink_delete_PyClient(PyObject* self, PyObject* args) {
    PyObject* client_py = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:delete_PyClient"),
                          &client_py)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyClient* client = reinterpret_cast<PyClient*>(
        PyCapsule_GetPointer(client_py, "autolink_autolink_pyclient"));
    delete client;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyClient_send_request(PyObject* self, PyObject* args) {
    PyObject* pyobj_client = nullptr;
    Py_buffer buffer;
    // Py_ssize_t len = 0;
    if (!PyArg_ParseTuple(
            args, const_cast<char*>("Oy*:autolink_PyClient_send_request"),
            &pyobj_client, &buffer)) {
        AERROR << "autolink_PyClient_send_request:PyArg_ParseTuple failed!";
        PyBuffer_Release(&buffer);
        return PYOBJECT_NULL_STRING;
    }

    PyClient* client =
        PyObjectToPtr<PyClient*>(pyobj_client, "autolink_autolink_pyclient");

    if (nullptr == client) {
        AERROR << "autolink_PyClient_send_request:client ptr is null!";
        PyBuffer_Release(&buffer);
        return PYOBJECT_NULL_STRING;
    }

    std::string data_str(
        const_cast<const char*>(static_cast<char*>(buffer.buf)), buffer.len);
    ADEBUG << "c++:autolink_PyClient_send_request data->[ " << data_str << "]";
    const std::string response_str =
        client->send_request((std::string const&)data_str);
    ADEBUG << "c++:response data->[ " << response_str << "]";
    PyBuffer_Release(&buffer);
    return C_STR_TO_PY_BYTES(response_str);
}

PyObject* autolink_new_PyService(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* data_type = nullptr;
    PyObject* node_pyobj = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("ssO:autolink_new_PyService"),
                          &channel_name, &data_type, &node_pyobj)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    Node* node = reinterpret_cast<Node*>(
        PyCapsule_GetPointer(node_pyobj, "autolink_autolink_pynode"));
    if (!node) {
        AERROR << "node is null";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyService* service = new PyService((std::string const&)*channel_name,
                                       (std::string const&)*data_type, node);
    PyObject* pyobj_service =
        PyCapsule_New(service, "autolink_autolink_pyservice", nullptr);
    return pyobj_service;
}

PyObject* autolink_delete_PyService(PyObject* self, PyObject* args) {
    PyObject* pyobj_service = nullptr;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("O:autolink_delete_PyService"),
                          &pyobj_service)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyService* service = reinterpret_cast<PyService*>(
        PyCapsule_GetPointer(pyobj_service, "autolink_autolink_pyservice"));
    delete service;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyService_register_func(PyObject* self, PyObject* args) {
    PyObject* pyobj_regist_fun = nullptr;
    PyObject* pyobj_service = nullptr;

    int (*callback_fun)(char const*) = (int (*)(char const*))0;

    if (!PyArg_ParseTuple(
            args, const_cast<char*>("OO:autolink_PyService_register_func"),
            &pyobj_service, &pyobj_regist_fun)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyService* service =
        PyObjectToPtr<PyService*>(pyobj_service, "autolink_autolink_pyservice");
    callback_fun = (int (*)(const char* i))PyInt_AsLong(pyobj_regist_fun);
    if (service) {
        AINFO << "service regist fun";
        service->register_func(callback_fun);
    }

    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyService_read(PyObject* self, PyObject* args) {
    PyObject* pyobj_service = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_PyService_read"),
                          &pyobj_service)) {
        AERROR << "autolink_PyService_read:PyArg_ParseTuple failed!";
        return PYOBJECT_NULL_STRING;
    }
    PyService* service =
        PyObjectToPtr<PyService*>(pyobj_service, "autolink_autolink_pyservice");
    if (nullptr == service) {
        AERROR << "autolink_PyService_read:service ptr is null!";
        return PYOBJECT_NULL_STRING;
    }

    const std::string reader_ret = service->read();
    ADEBUG << "c++:autolink_PyService_read -> " << reader_ret;
    return C_STR_TO_PY_BYTES(reader_ret);
}

PyObject* autolink_PyService_write(PyObject* self, PyObject* args) {
    PyObject* pyobj_service = nullptr;
    Py_buffer buffer;
    // Py_ssize_t len = 0;
    if (!PyArg_ParseTuple(args,
                          const_cast<char*>("Oy*:autolink_PyService_write"),
                          &pyobj_service, &buffer)) {
        AERROR << "autolink_PyService_write:PyArg_ParseTuple failed!";
        PyBuffer_Release(&buffer);
        return PyInt_FromLong(1);
    }

    PyService* service =
        PyObjectToPtr<PyService*>(pyobj_service, "autolink_autolink_pyservice");

    if (nullptr == service) {
        AERROR << "autolink_PyService_write:writer ptr is null!";
        PyBuffer_Release(&buffer);
        return PyInt_FromLong(1);
    }

    std::string data_str(
        const_cast<const char*>(static_cast<char*>(buffer.buf)), buffer.len);
    ADEBUG << "c++:autolink_PyService_write data->[ " << data_str << "]";
    int ret = service->write((std::string const&)data_str);
    PyBuffer_Release(&buffer);
    return PyInt_FromLong(ret);
}

PyObject* autolink_new_PyNode(PyObject* self, PyObject* args) {
    char* node_name = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("s:autolink_new_PyNode"),
                          &node_name)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node = new PyNode((std::string const&)node_name);
    PyObject* pyobj_node =
        PyCapsule_New(node, "autolink_autolink_pynode", nullptr);
    return pyobj_node;
}

PyObject* autolink_delete_PyNode(PyObject* self, PyObject* args) {
    PyObject* pyobj_node = nullptr;
    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_delete_PyNode"),
                          &pyobj_node)) {
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node = reinterpret_cast<PyNode*>(
        PyCapsule_GetPointer(pyobj_node, "autolink_autolink_pynode"));
    delete node;
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyNode_create_writer(PyObject* self, PyObject* args) {
    PyObject* pyobj_node = nullptr;
    char* channel_name = nullptr;
    char* type_name = nullptr;
    uint32_t qos_depth = 1;

    if (!PyArg_ParseTuple(
            args, const_cast<char*>("OssI:autolink_PyNode_create_writer"),
            &pyobj_node, &channel_name, &type_name, &qos_depth)) {
        AERROR << "autolink_PyNode_create_writer:PyArg_ParseTuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node =
        PyObjectToPtr<PyNode*>(pyobj_node, "autolink_autolink_pynode");
    if (nullptr == node) {
        AERROR << "autolink_PyNode_create_writer:node ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyWriter* writer = reinterpret_cast<PyWriter*>(
        (node->create_writer((std::string const&)channel_name,
                             (std::string const&)type_name, qos_depth)));

    if (nullptr == writer) {
        AERROR << "autolink_PyNode_create_writer:writer is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    PyObject* pyobj_writer =
        PyCapsule_New(writer, "autolink_autolink_pywriter", nullptr);
    return pyobj_writer;
}

PyObject* autolink_PyNode_create_reader(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* type_name = nullptr;
    PyObject* pyobj_node = nullptr;

    if (!PyArg_ParseTuple(
            args, const_cast<char*>("Oss:autolink_PyNode_create_reader"),
            &pyobj_node, &channel_name, &type_name)) {
        AERROR << "PyNode_create_reader:PyArg_ParseTuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node =
        PyObjectToPtr<PyNode*>(pyobj_node, "autolink_autolink_pynode");
    if (nullptr == node) {
        AERROR << "autolink_PyNode_create_reader:node ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyReader* reader = reinterpret_cast<PyReader*>((node->create_reader(
        (std::string const&)channel_name, (std::string const&)type_name)));
    ACHECK(reader) << "PyReader is NULL!";

    PyObject* pyobj_reader =
        PyCapsule_New(reader, "autolink_autolink_pyreader", nullptr);
    return pyobj_reader;
}

PyObject* autolink_PyNode_create_client(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* type_name = nullptr;
    PyObject* pyobj_node = nullptr;

    if (!PyArg_ParseTuple(
            args, const_cast<char*>("Oss:autolink_PyNode_create_client"),
            &pyobj_node, &channel_name, &type_name)) {
        AERROR << "autolink_PyNode_create_client:PyArg_ParseTuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node =
        PyObjectToPtr<PyNode*>(pyobj_node, "autolink_autolink_pynode");
    if (nullptr == node) {
        AERROR << "autolink_PyNode_create_client:node ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyClient* client = reinterpret_cast<PyClient*>((node->create_client(
        (std::string const&)channel_name, (std::string const&)type_name)));
    PyObject* pyobj_client =
        PyCapsule_New(client, "autolink_autolink_pyclient", nullptr);

    return pyobj_client;
}

PyObject* autolink_PyNode_create_service(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    char* type_name = nullptr;
    PyObject* pyobj_node = nullptr;

    if (!PyArg_ParseTuple(
            args, const_cast<char*>("Oss:autolink_PyNode_create_service"),
            &pyobj_node, &channel_name, &type_name)) {
        AERROR << "autolink_PyNode_create_service:PyArg_ParseTuple failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node =
        PyObjectToPtr<PyNode*>(pyobj_node, "autolink_autolink_pynode");
    if (nullptr == node) {
        AERROR << "autolink_PyNode_create_service:node ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyService* service = reinterpret_cast<PyService*>((node->create_service(
        (std::string const&)channel_name, (std::string const&)type_name)));
    PyObject* pyobj_service =
        PyCapsule_New(service, "autolink_autolink_pyservice", nullptr);
    return pyobj_service;
}

PyObject* autolink_PyNode_shutdown(PyObject* self, PyObject* args) {
    PyObject* pyobj_node = nullptr;

    if (!PyArg_ParseTuple(args, const_cast<char*>("O:autolink_PyNode_shutdown"),
                          &pyobj_node)) {
        AERROR << "autolink_PyNode_shutdown:PyNode_shutdown failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    PyNode* node =
        PyObjectToPtr<PyNode*>(pyobj_node, "autolink_autolink_pynode");
    if (nullptr == node) {
        AERROR << "autolink_PyNode_shutdown:node ptr is null!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    node->shutdown();
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* autolink_PyNode_register_message(PyObject* self, PyObject* args) {
    PyObject* pyobj_node = nullptr;
    Py_buffer buffer;
    // int len = 0;
    if (!PyArg_ParseTuple(
            args, const_cast<char*>("Oy*:autolink_PyNode_register_message"),
            &pyobj_node, &buffer)) {
        AERROR << "autolink_PyNode_register_message: failed!";
        Py_INCREF(Py_None);
        PyBuffer_Release(&buffer);
        return Py_None;
    }
    std::string desc_str(
        const_cast<const char*>(static_cast<char*>(buffer.buf)), buffer.len);
    PyNode* node =
        PyObjectToPtr<PyNode*>(pyobj_node, "autolink_autolink_pynode");
    if (nullptr == node) {
        AERROR << "autolink_PyNode_register_message:node ptr is null! desc->"
               << desc_str;
        Py_INCREF(Py_None);
        PyBuffer_Release(&buffer);
        return Py_None;
    }
    node->register_message((std::string const&)desc_str);
    Py_INCREF(Py_None);
    PyBuffer_Release(&buffer);
    return Py_None;
}

PyObject* autolink_PyChannelUtils_get_msg_type(PyObject* self, PyObject* args) {
    char* channel_name = nullptr;
    Py_ssize_t len = 0;
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args, const_cast<char*>("s#B:autolink_PyChannelUtils_get_msg_type"),
            &channel_name, &len, &sleep_s)) {
        AERROR << "autolink_PyChannelUtils_get_msg_type failed!";
        return PYOBJECT_NULL_STRING;
    }
    std::string channel(channel_name, len);
    const std::string msg_type =
        PyChannelUtils::get_msgtype_by_channelname(channel, sleep_s);
    return C_STR_TO_PY_BYTES(msg_type);
}

PyObject* autolink_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata(
    PyObject* self, PyObject* args) {
    char* msgtype = nullptr;
    char* rawdata = nullptr;
    Py_ssize_t len = 0;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("ss#:autolink_PyChannelUtils_get_debugstring_by_"
                              "msgtype_rawmsgdata"),
            &msgtype, &rawdata, &len)) {
        AERROR
            << "autolink_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata "
               "failed!";
        return PYOBJECT_NULL_STRING;
    }
    std::string raw_data(rawdata, len);
    const std::string debug_string =
        PyChannelUtils::get_debugstring_by_msgtype_rawmsgdata(msgtype,
                                                              raw_data);
    return C_STR_TO_PY_BYTES(debug_string);
}

static PyObject* autolink_PyChannelUtils_get_active_channels(PyObject* self,
                                                             PyObject* args) {
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("B:autolink_PyChannelUtils_get_active_channels"),
            &sleep_s)) {
        AERROR << "autolink_PyChannelUtils_get_active_channels failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    std::vector<std::string> channel_list =
        PyChannelUtils::get_active_channels(sleep_s);
    PyObject* pyobj_list = PyList_New(channel_list.size());
    size_t pos = 0;
    for (const std::string& channel : channel_list) {
        PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
        pos++;
    }

    return pyobj_list;
}

// return dict value look like:
// {  'channel1':[atrr1, atrr2, atrr3],
//    'channel2':[atrr1, atrr2]
// }
static PyObject* autolink_PyChannelUtils_get_channels_info(PyObject* self,
                                                           PyObject* args) {
    auto channelsinfo = PyChannelUtils::get_channels_info();
    PyObject* pyobj_channelinfo_dict = PyDict_New();
    for (auto& channelinfo : channelsinfo) {
        std::string channel_name = channelinfo.first;
        PyObject* bld_name = Py_BuildValue("s", channel_name.c_str());
        std::vector<std::string>& roleAttr_list = channelinfo.second;
        PyObject* pyobj_list = PyList_New(roleAttr_list.size());

        size_t pos = 0;
        for (auto& attr : roleAttr_list) {
            PyList_SetItem(pyobj_list, pos, C_STR_TO_PY_BYTES(attr));
            pos++;
        }
        PyDict_SetItem(pyobj_channelinfo_dict, bld_name, pyobj_list);
        Py_DECREF(bld_name);
    }
    return pyobj_channelinfo_dict;
}

PyObject* autolink_PyNodeUtils_get_active_nodes(PyObject* self,
                                                PyObject* args) {
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args, const_cast<char*>("B:autolink_PyNodeUtils_get_active_nodes"),
            &sleep_s)) {
        AERROR << "autolink_PyNodeUtils_get_active_nodes failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    std::vector<std::string> nodes_name =
        autolink::PyNodeUtils::get_active_nodes(sleep_s);
    PyObject* pyobj_list = PyList_New(nodes_name.size());
    size_t pos = 0;
    for (const std::string& name : nodes_name) {
        PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", name.c_str()));
        pos++;
    }

    return pyobj_list;
}

PyObject* autolink_PyNodeUtils_get_node_attr(PyObject* self, PyObject* args) {
    char* node_name = nullptr;
    Py_ssize_t len = 0;
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args, const_cast<char*>("s#B:autolink_PyNodeUtils_get_node_attr"),
            &node_name, &len, &sleep_s)) {
        AERROR << "autolink_PyNodeUtils_get_node_attr failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    std::string name(node_name, len);
    const std::string node_attr =
        autolink::PyNodeUtils::get_node_attr(name, sleep_s);
    return C_STR_TO_PY_BYTES(node_attr);
}

PyObject* autolink_PyNodeUtils_get_readersofnode(PyObject* self,
                                                 PyObject* args) {
    char* node_name = nullptr;
    Py_ssize_t len = 0;
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("s#B:autolink_PyNodeUtils_get_readersofnode"),
            &node_name, &len, &sleep_s)) {
        AERROR << "autolink_PyNodeUtils_get_readersofnode failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    std::string name(node_name, len);
    std::vector<std::string> readers_channel =
        autolink::PyNodeUtils::get_readersofnode(name, sleep_s);
    PyObject* pyobj_list = PyList_New(readers_channel.size());
    size_t pos = 0;
    for (const std::string& channel : readers_channel) {
        PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
        pos++;
    }

    return pyobj_list;
}

PyObject* autolink_PyNodeUtils_get_writersofnode(PyObject* self,
                                                 PyObject* args) {
    char* node_name = nullptr;
    Py_ssize_t len = 0;
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("s#B:autolink_PyNodeUtils_get_writersofnode"),
            &node_name, &len, &sleep_s)) {
        AERROR << "autolink_PyNodeUtils_get_writersofnode failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    std::string name(node_name, len);
    std::vector<std::string> writers_channel =
        autolink::PyNodeUtils::get_writersofnode(name, sleep_s);
    PyObject* pyobj_list = PyList_New(writers_channel.size());
    size_t pos = 0;
    for (const std::string& channel : writers_channel) {
        PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", channel.c_str()));
        pos++;
    }

    return pyobj_list;
}

PyObject* autolink_PyServiceUtils_get_active_services(PyObject* self,
                                                      PyObject* args) {
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("B:autolink_PyServiceUtils_get_active_services"),
            &sleep_s)) {
        AERROR << "autolink_PyServiceUtils_get_active_services failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }

    std::vector<std::string> services_name =
        PyServiceUtils::get_active_services(sleep_s);
    PyObject* pyobj_list = PyList_New(services_name.size());
    size_t pos = 0;
    for (const std::string& name : services_name) {
        PyList_SetItem(pyobj_list, pos, Py_BuildValue("s", name.c_str()));
        pos++;
    }

    return pyobj_list;
}

PyObject* autolink_PyServiceUtils_get_service_attr(PyObject* self,
                                                   PyObject* args) {
    char* srv_name = nullptr;
    Py_ssize_t len = 0;
    unsigned char sleep_s = 0;
    if (!PyArg_ParseTuple(
            args,
            const_cast<char*>("s#B:autolink_PyServiceUtils_get_service_attr"),
            &srv_name, &len, &sleep_s)) {
        AERROR << "autolink_PyServiceUtils_get_service_attr failed!";
        Py_INCREF(Py_None);
        return Py_None;
    }
    std::string name(srv_name, len);
    const std::string srv_attr =
        PyServiceUtils::get_service_attr(name, sleep_s);
    return C_STR_TO_PY_BYTES(srv_attr);
}

/////////////////////////////////////////////////////////////////////
//// global for whole page, init module
/////////////////////////////////////////////////////////////////////
static PyMethodDef _autolink_methods[] = {
    // PyInit fun
    {"py_init", autolink_py_init, METH_VARARGS, ""},
    {"py_ok", autolink_py_ok, METH_NOARGS, ""},
    {"py_shutdown", autolink_py_shutdown, METH_NOARGS, ""},
    {"py_is_shutdown", autolink_py_is_shutdown, METH_NOARGS, ""},
    {"py_waitforshutdown", autolink_py_waitforshutdown, METH_NOARGS, ""},

    // PyWriter fun
    {"new_PyWriter", autolink_new_PyWriter, METH_VARARGS, ""},
    {"delete_PyWriter", autolink_delete_PyWriter, METH_VARARGS, ""},
    {"PyWriter_write", autolink_PyWriter_write, METH_VARARGS, ""},

    // PyReader fun
    {"new_PyReader", autolink_new_PyReader, METH_VARARGS, ""},
    {"delete_PyReader", autolink_delete_PyReader, METH_VARARGS, ""},
    {"PyReader_register_func", autolink_PyReader_register_func, METH_VARARGS,
     ""},
    {"PyReader_read", autolink_PyReader_read, METH_VARARGS, ""},

    // PyClient fun
    {"new_PyClient", autolink_new_PyClient, METH_VARARGS, ""},
    {"delete_PyClient", autolink_delete_PyClient, METH_VARARGS, ""},
    {"PyClient_send_request", autolink_PyClient_send_request, METH_VARARGS, ""},
    // PyService fun
    {"new_PyService", autolink_new_PyService, METH_VARARGS, ""},
    {"delete_PyService", autolink_delete_PyService, METH_VARARGS, ""},
    {"PyService_register_func", autolink_PyService_register_func, METH_VARARGS,
     ""},
    {"PyService_read", autolink_PyService_read, METH_VARARGS, ""},
    {"PyService_write", autolink_PyService_write, METH_VARARGS, ""},
    // PyNode fun
    {"new_PyNode", autolink_new_PyNode, METH_VARARGS, ""},
    {"delete_PyNode", autolink_delete_PyNode, METH_VARARGS, ""},
    {"PyNode_shutdown", autolink_PyNode_shutdown, METH_VARARGS, ""},
    {"PyNode_create_writer", autolink_PyNode_create_writer, METH_VARARGS, ""},
    {"PyNode_register_message", autolink_PyNode_register_message, METH_VARARGS,
     ""},
    {"PyNode_create_reader", autolink_PyNode_create_reader, METH_VARARGS, ""},
    {"PyNode_create_client", autolink_PyNode_create_client, METH_VARARGS, ""},
    {"PyNode_create_service", autolink_PyNode_create_service, METH_VARARGS, ""},

    {"PyChannelUtils_get_msg_type", autolink_PyChannelUtils_get_msg_type,
     METH_VARARGS, ""},
    {"PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata",
     autolink_PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata,
     METH_VARARGS, ""},
    {"PyChannelUtils_get_active_channels",
     autolink_PyChannelUtils_get_active_channels, METH_VARARGS, ""},
    {"PyChannelUtils_get_channels_info",
     autolink_PyChannelUtils_get_channels_info, METH_VARARGS, ""},

    {"PyNodeUtils_get_active_nodes", autolink_PyNodeUtils_get_active_nodes,
     METH_VARARGS, ""},
    {"PyNodeUtils_get_node_attr", autolink_PyNodeUtils_get_node_attr,
     METH_VARARGS, ""},
    {"PyNodeUtils_get_readersofnode", autolink_PyNodeUtils_get_readersofnode,
     METH_VARARGS, ""},
    {"PyNodeUtils_get_writersofnode", autolink_PyNodeUtils_get_writersofnode,
     METH_VARARGS, ""},

    {"PyServiceUtils_get_active_services",
     autolink_PyServiceUtils_get_active_services, METH_VARARGS, ""},
    {"PyServiceUtils_get_service_attr",
     autolink_PyServiceUtils_get_service_attr, METH_VARARGS, ""},

    {nullptr, nullptr, 0, nullptr} /* sentinel */
};

/// Init function of this module
PyMODINIT_FUNC PyInit__autolink_wrapper(void) {
    static struct PyModuleDef module_def = {
        PyModuleDef_HEAD_INIT,
        "_autolink_wrapper",  // Module name.
        "Autolink module",    // Module doc.
        -1,                   // Module size.
        _autolink_methods,    // Module methods.
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    };

    return PyModule_Create(&module_def);
}