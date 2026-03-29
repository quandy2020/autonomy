# Autolink Python Examples

This directory contains runnable Python examples for Autolink.

## Prerequisites

- Build and install Autolink Python wrappers (`AUTOLINK_BUILD_PYTHON=ON`).
- Export runtime paths (or source your setup script):

```bash
export AUTOLINK_DISTRIBUTION_HOME=/usr/local
export AUTOLINK_PATH=/usr/local/share/autolink
export PYTHONPATH=/usr/local/python:/usr/local/lib/autolink/python/internal:${PYTHONPATH}
```

For source-tree runs, adjust paths to your build/install prefix.

## Run Examples

### Pub/Sub

```bash
# terminal 1
python3 py_listener.py

# terminal 2
python3 py_talker.py
```

### Service/Client

```bash
# terminal 1
python3 py_service.py

# terminal 2
python3 py_client.py
```

### Parameter

```bash
python3 py_parameter.py
```

### Time/Timer

```bash
python3 py_time.py
python3 py_timer.py
```

### Record

```bash
python3 py_record.py
python3 py_record_channel_info.py <record_file>
python3 py_record_trans.py <record_file>
```

## Notes

- `py_action_client.py` and `py_action_server.py` are placeholders and not implemented yet.
- If discovery fails across hosts/containers, ensure `AUTOLINK_DOMAIN_ID` and network settings are aligned.
