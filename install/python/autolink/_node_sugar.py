class _Writer:
    def __init__(self, writer, message_class):
        self._writer = writer
        self._message_class = message_class

    def write(self, message):
        if self._message_class is not None:
            message = message.SerializeToString()
        if not isinstance(message, bytes):
            raise TypeError("message must be bytes or an instance of data_type")
        return self._writer.write(message)


class _Reader:
    def __init__(self, reader):
        self._reader = reader


def _resolve_message_type(node, data_type):
    if data_type is None or data_type == "RawData":
        return "RawData", None
    if isinstance(data_type, str):
        return data_type, None

    descriptor = getattr(data_type, "DESCRIPTOR", None)
    if descriptor is None:
        raise TypeError("data_type must be a protobuf class, str, or None")

    _register_descriptor_tree(node, descriptor.file)
    return descriptor.full_name, data_type


def _register_descriptor_tree(node, file_descriptor, registered=None):
    if registered is None:
        registered = set()
    if file_descriptor.name in registered:
        return
    for dependency in file_descriptor.dependencies:
        _register_descriptor_tree(node, dependency, registered)
    registered.add(file_descriptor.name)
    node.register_message(file_descriptor.serialized_pb)


def create_writer(node, name, data_type, qos_depth=1):
    message_type, message_class = _resolve_message_type(node, data_type)
    return _Writer(node._create_writer(name, message_type, qos_depth),
                   message_class)


def create_reader(node, name, callback, data_type=None):
    message_type, message_class = _resolve_message_type(node, data_type)
    if message_class is not None:
        original_callback = callback

        def callback(data):
            message = message_class()
            message.ParseFromString(data)
            original_callback(message)

    handle = _Reader(node._create_reader(name, callback, message_type))
    # Keep the reader alive for the lifetime of the node.
    readers = getattr(node, "_autolink_readers", None)
    if readers is None:
        readers = []
        setattr(node, "_autolink_readers", readers)
    readers.append(handle)
    return handle


def create_service(node, name, callback, req_type="RawData", res_type=None):
    # res_type kept for API symmetry; wire type follows req_type for RawData path.
    del res_type
    message_type, message_class = _resolve_message_type(node, req_type)
    if message_class is not None:
        original = callback

        def callback(data):
            req = message_class()
            req.ParseFromString(data)
            out = original(req)
            if hasattr(out, "SerializeToString"):
                return out.SerializeToString()
            return out

    handle = node._create_service(name, callback, message_type)
    # Keep the service alive for the lifetime of the node (discarding the
    # return value must not destroy the server).
    services = getattr(node, "_autolink_services", None)
    if services is None:
        services = []
        setattr(node, "_autolink_services", services)
    services.append(handle)
    return handle


def create_client(node, name, req_type="RawData", res_type=None):
    del res_type
    message_type, message_class = _resolve_message_type(node, req_type)
    client = node._create_client(name, message_type)
    if message_class is None:
        return client

    class _PbClient:
        def __init__(self, raw, cls):
            self._raw = raw
            self._cls = cls

        def send_request(self, message, timeout_sec=5):
            if hasattr(message, "SerializeToString"):
                message = message.SerializeToString()
            data = self._raw.send_request(message, timeout_sec)
            out = self._cls()
            out.ParseFromString(data)
            return out

    return _PbClient(client, message_class)


def spin():
    import time

    from ._core import is_shutdown

    while not is_shutdown():
        time.sleep(0.002)
