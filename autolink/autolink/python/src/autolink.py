#!/usr/bin/env python3

#***************************************************************************
# Copyright 2025 The Openbot Authors (duyongquan)
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
#***************************************************************************

# -*- coding: utf-8 -*-
"""Module for init environment."""

import ctypes
import threading
import time

try:
    from google.protobuf.descriptor_pb2 import FileDescriptorProto
except ModuleNotFoundError:
    FileDescriptorProto = None

try:
    from ._loader import load_wrapper_module
except ImportError:
    from _loader import load_wrapper_module


PY_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
PY_CALLBACK_TYPE_T = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

_AUTOLINK = load_wrapper_module("_autolink_wrapper")


##
# @brief init autolink environment.
# @param module_name Used as the log file name.
#
# @return Success is True, otherwise False.
def init(module_name="autolink_py"):
    """
    init autolink environment.
    """
    return _AUTOLINK.py_init(module_name)


def ok():
    """
    is autolink envi ok.
    """
    return _AUTOLINK.py_ok()


def shutdown():
    """
    shutdown autolink envi.
    """
    return _AUTOLINK.py_shutdown()


def is_shutdown():
    """
    is autolink shutdown.
    """
    return _AUTOLINK.py_is_shutdown()


def waitforshutdown():
    """
    wait until the autolink is shutdown.
    """
    return _AUTOLINK.py_waitforshutdown()

# //////////////////////////////class//////////////////////////////


class Writer(object):

    """
    Class for autolink writer wrapper.
    """

    def __init__(self, name, writer, data_type):
        self.name = name
        self.writer = writer
        self.data_type = data_type

    ##
    # @brief write message.
    #
    # @param data is a message type.
    #
    # @return Success is 0, otherwise False.
    def write(self, data):
        """
        writer message string
        """
        return _AUTOLINK.PyWriter_write(self.writer, data.SerializeToString())


class Reader(object):

    """
    Class for autolink reader wrapper.
    """

    def __init__(self, name, reader, data_type):
        self.name = name
        self.reader = reader
        self.data_type = data_type


class Client(object):

    """
    Class for autolink service client wrapper.
    """

    def __init__(self, client, data_type):
        self.client = client
        self.data_type = data_type

    ##
    # @brief send request message to service.
    #
    # @param data is a message type.
    #
    # @return None or response from service.
    def send_request(self, data):
        """
        send request to service
        """
        response_str = _AUTOLINK.PyClient_send_request(
            self.client, data.SerializeToString())
        if len(response_str) == 0:
            return None

        response = self.data_type()
        response.ParseFromString(response_str)
        return response


class Node(object):

    """
    Class for autolink Node wrapper.
    """

    def __init__(self, name):
        self.node = _AUTOLINK.new_PyNode(name)
        self.list_writer = []
        self.list_reader = []
        self.subs = {}
        self.pubs = {}
        self.list_client = []
        self.list_service = []
        self.mutex = threading.Lock()
        self.callbacks = {}
        self.services = {}

    def __del__(self):
        # print("+++ node __del___")
        for writer in self.list_writer:
            _AUTOLINK.delete_PyWriter(writer)
        for reader in self.list_reader:
            _AUTOLINK.delete_PyReader(reader)
        for c in self.list_client:
            _AUTOLINK.delete_PyClient(c)
        for s in self.list_service:
            _AUTOLINK.delete_PyService(s)
        _AUTOLINK.delete_PyNode(self.node)

    ##
    # @brief register proto message by proto descriptor file.
    #
    # @param file_desc object about datatype.DESCRIPTOR.file .
    def register_message(self, file_desc):
        """
        register proto message desc file.
        """
        if FileDescriptorProto is None:
            raise RuntimeError(
                "python protobuf is required for register_message. "
                "Install python3-protobuf or pip install protobuf."
            )
        for dep in file_desc.dependencies:
            self.register_message(dep)
        proto = FileDescriptorProto()
        file_desc.CopyToProto(proto)
        proto.name = file_desc.name
        desc_str = proto.SerializeToString()
        _AUTOLINK.PyNode_register_message(self.node, desc_str)

    ##
    # @brief create a channel writer for send message to another channel.
    #
    # @param name is the channel name.
    # @param data_type is message class for serialization
    # @param qos_depth is a queue size, which defines the size of the cache.
    #
    # @return return the writer object.
    def create_writer(self, name, data_type, qos_depth=1):
        """
        create a channel writer for send message to another channel.
        """
        self.register_message(data_type.DESCRIPTOR.file)
        datatype = data_type.DESCRIPTOR.full_name
        writer = _AUTOLINK.PyNode_create_writer(self.node, name,
                                             datatype, qos_depth)
        self.list_writer.append(writer)
        return Writer(name, writer, datatype)

    def reader_callback(self, name):
        sub = self.subs[name.decode('utf8')]
        msg_str = _AUTOLINK.PyReader_read(sub[0], False)
        if len(msg_str) > 0:
            if sub[3] != "RawData":
                proto = sub[3]()
                proto.ParseFromString(msg_str)
            else:
                # print "read rawdata-> ",sub[3]
                proto = msg_str

            if sub[2] is None:
                sub[1](proto)
            else:
                sub[1](proto, sub[2])
        return 0

    ##
    # @brief create a channel reader for receive message from another channel.
    #
    # @param name the channel name to read.
    # @param data_type  message class for serialization
    # @param callback function to call (fn(data)) when data is received. If
    # args is set, the function must accept the args as a second argument,
    # i.e. fn(data, args)
    # @param args additional arguments to pass to the callback
    #
    # @return return the writer object.
    def create_reader(self, name, data_type, callback, args=None):
        """
        create a channel reader for receive message from another channel.
        """
        self.mutex.acquire()
        if name in self.subs.keys():
            self.mutex.release()
            return None
        self.mutex.release()

        # Register message type and get full name
        if data_type != "RawData":
            self.register_message(data_type.DESCRIPTOR.file)
            datatype = data_type.DESCRIPTOR.full_name
        else:
            datatype = "RawData"
        reader = _AUTOLINK.PyNode_create_reader(
            self.node, name, datatype)
        if reader is None:
            return None
        self.list_reader.append(reader)
        sub = (reader, callback, args, data_type, False)

        self.mutex.acquire()
        self.subs[name] = sub
        self.mutex.release()
        fun_reader_cb = PY_CALLBACK_TYPE(self.reader_callback)
        self.callbacks[name] = fun_reader_cb
        f_ptr = ctypes.cast(self.callbacks[name], ctypes.c_void_p).value
        _AUTOLINK.PyReader_register_func(reader, f_ptr)

        return Reader(name, reader, data_type)

    def create_rawdata_reader(self, name, callback, args=None):
        """
        Create RawData reader:listener RawMessage
        """
        return self.create_reader(name, "RawData", callback, args)

    ##
    # @brief create client for the c/s.
    #
    # @param name the service name.
    # @param request_data_type the request message type.
    # @param response_data_type the response message type.
    #
    # @return the client object.
    def create_client(self, name, request_data_type, response_data_type):
        datatype = request_data_type.DESCRIPTOR.full_name
        c = _AUTOLINK.PyNode_create_client(self.node, name,
                                        str(datatype))
        self.list_client.append(c)
        return Client(c, response_data_type)

    def service_callback(self, name):
        # Temporary workaround for cyber_py3 examples: service & client
        v = self.services[name.decode("utf-8")]

        msg_str = _AUTOLINK.PyService_read(v[0])
        if (len(msg_str) > 0):
            proto = v[3]()
            proto.ParseFromString(msg_str)
            response = None
            if v[2] is None:
                response = v[1](proto)
            else:
                response = v[1](proto, v[2])
            _AUTOLINK.PyService_write(v[0], response.SerializeToString())
        return 0

    ##
    # @brief create client for the c/s.
    #
    # @param name the service name.
    # @param req_data_type the request message type.
    # @param res_data_type the response message type.
    # @param callback function to call (fn(data)) when data is received. If
    # args is set, the function must accept the args as a second argument,
    # i.e. fn(data, args)
    # @param args additional arguments to pass to the callback.
    #
    # @return return the service object.
    def create_service(self, name, req_data_type, res_data_type, callback,
                       args=None):
        self.mutex.acquire()
        if name in self.services.keys():
            self.mutex.release()
            return None
        self.mutex.release()
        datatype = req_data_type.DESCRIPTOR.full_name
        s = _AUTOLINK.PyNode_create_service(self.node, name, str(datatype))
        self.list_service.append(s)
        v = (s, callback, args, req_data_type, False)
        self.mutex.acquire()
        self.services[name] = v
        self.mutex.release()
        f = PY_CALLBACK_TYPE(self.service_callback)
        self.callbacks[name] = f
        f_ptr = ctypes.cast(f, ctypes.c_void_p).value
        _AUTOLINK.PyService_register_func(s, f_ptr)
        return s

    def spin(self):
        """
        spin for every 0.002s.
        """
        while not _AUTOLINK.py_is_shutdown():
            time.sleep(0.002)


class ChannelUtils(object):

    @staticmethod
    ##
    # @brief Parse rawmsg from rawmsg data by message type.
    #
    # @param msg_type message type.
    # @param rawmsgdata rawmsg data.
    #
    # @return a human readable form of this message. For debugging and
    # other purposes.
    def get_debugstring_rawmsgdata(msg_type, rawmsgdata):
        return _AUTOLINK.PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata(msg_type, rawmsgdata)

    @staticmethod
    ##
    # @brief Parse rawmsg from channel name.
    #
    # @param channel_name channel name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return return the messsage type of this channel.
    def get_msgtype(channel_name, sleep_s=2):
        return _AUTOLINK.PyChannelUtils_get_msg_type(channel_name, sleep_s)

    @staticmethod
    ##
    # @brief Get all active channel names
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active channel names.
    def get_channels(sleep_s=2):
        return _AUTOLINK.PyChannelUtils_get_active_channels(sleep_s)

    @staticmethod
    ##
    # @brief Get the active channel info.
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active channels info. {'channel1':[], 'channel2':[]} .
    def get_channels_info(sleep_s=2):
        return _AUTOLINK.PyChannelUtils_get_channels_info(sleep_s)


class NodeUtils(object):

    @staticmethod
    ##
    # @brief Get all active node names.
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active node names.
    def get_nodes(sleep_s=2):
        return _AUTOLINK.PyNodeUtils_get_active_nodes(sleep_s)

    @staticmethod
    ##
    # @brief Get node attribute by the node name.
    #
    # @param node_name node name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return the node's attribute.
    def get_node_attr(node_name, sleep_s=2):
        return _AUTOLINK.PyNodeUtils_get_node_attr(node_name, sleep_s)

    @staticmethod
    ##
    # @brief Get node's reader channel names
    #
    # @param node_name the node name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return node's reader channel names.
    def get_readersofnode(node_name, sleep_s=2):
        return _AUTOLINK.PyNodeUtils_get_readersofnode(node_name, sleep_s)

    @staticmethod
    ##
    # @brief Get node's writer channel names.
    #
    # @param node_name the node name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return node's writer channel names.
    def get_writersofnode(node_name, sleep_s=2):
        return _AUTOLINK.PyNodeUtils_get_writersofnode(node_name, sleep_s)


class ServiceUtils(object):

    @staticmethod
    ##
    # @brief Get all active service names.
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active service names.
    def get_services(sleep_s=2):
        return _AUTOLINK.PyServiceUtils_get_active_services(sleep_s)

    @staticmethod
    ##
    # @brief Get service attribute by the service name.
    #
    # @param service_name service name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return the service's attribute.
    def get_service_attr(service_name, sleep_s=2):
        return _AUTOLINK.PyServiceUtils_get_service_attr(service_name, sleep_s)
