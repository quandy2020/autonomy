# Autolink C++ Examples

## Build

```bash
cmake -S src/autonomy/autolink -B build/autolink -DAUTOLINK_BUILD_EXAMPLES=ON
cmake --build build/autolink -j8
```

Example binaries are generated under `build/autolink/bin/examples`.

## Runtime Environment

Set configuration path (choose one):

- Install tree:
  - `export AUTOLINK_PATH=/usr/local/share/autolink`
- Source/build tree:
  - `export AUTOLINK_PATH=<path-to-autolink-root>`

Your `AUTOLINK_PATH` should contain `conf/autolink.pb.conf`.

## Available Binaries

- `autolink_example_talker`
- `autolink_example_listener`
- `autolink_example_talker_listener`
- `autolink_example_service`
- `autolink_example_paramserver`
- `autolink_example_record`

## Quick Start: Talker + Listener

```bash
# terminal 1
cd build/autolink/bin/examples
./autolink_example_listener

# terminal 2
cd build/autolink/bin/examples
./autolink_example_talker
```

Listener should print received `seq` and `content` after discovery.

## If Data Is Not Received

- Ensure both processes use the same `AUTOLINK_PATH`.
- Ensure both sides use the same `AUTOLINK_DOMAIN_ID` (default `80`).
- For cross-host/container runs, verify network and RTPS discovery connectivity.
