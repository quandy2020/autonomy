# automsgs Python bindings

Python message bindings are generated from the same `.proto` files as the C++ library. The generated `*_pb2.py` modules are installed under the `automsgs` package.

## Install layout

- **C++**: Headers and `libautomsgs_proto.so` are installed as usual.
- **Python**: Generated modules are installed to `${AUTOMSGS_PYTHON_INSTALL_PATH}/automsgs/` (default: `lib/python` under the install prefix). The `python/src/__init__.py` is installed as `automsgs/__init__.py` so that `import automsgs` works.

To use the messages in Python, ensure the install directory is on `PYTHONPATH`, then:

```python
from automsgs.msgs.geometry_msgs.Vector3_pb2 import Vector3
from automsgs.msgs.std_msgs.String_pb2 import String
```

## Tests

The `basic_TEST.py` test checks serialization of a `Vector3` message. Run with:

```bash
ctest -R basic_TEST -V
```

Requires the Python `protobuf` package (e.g. `pip install protobuf`) so that `import google.protobuf` works when loading the generated `_pb2` modules.
