# autonomy
autonomy open robot for everyone





## 1 Q1 No module named 'cyber

```bash
source /usr/local/setup.bash
```



## 2 Q2 No module named 'google'

```bash
Traceback (most recent call last):
  File "/usr/local/bin/cyber_node", line 23, in <module>
    from cyber.python.cyber_py3 import cyber
  File "/usr/local/lib/python3.12/site-packages/cyber/python/cyber_py3/cyber.py", line 28, in <module>
    from google.protobuf.descriptor_pb2 import FileDescriptorProto
ModuleNotFoundError: No module named 'google'
```

sovle

```bash
python3 -m pip install protobuf==3.14.0 --break-system-packages
```

## Q3 cannot allocate memory in static TLS block

```bash
Traceback (most recent call last):
  File "/usr/local/bin/cyber_node", line 23, in <module>
    from cyber.python.cyber_py3 import cyber
  File "/usr/local/lib/python3.12/site-packages/cyber/python/cyber_py3/cyber.py", line 44, in <module>
    _CYBER = importlib.import_module('_cyber_wrapper')
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/importlib/__init__.py", line 90, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ImportError: /usr/local/lib/libtcmalloc.so.4: cannot allocate memory in static TLS block
```

sovle

```
export LD_PRELOAD=/usr/local/lib/libtcmalloc.so.4
```

