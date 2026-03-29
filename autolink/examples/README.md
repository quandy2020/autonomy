# Autolink Examples

This directory contains Autolink examples in multiple languages.

- C++ examples: `examples/cpp/README.md`
- Python examples: `examples/python/README.md`

## Build Enable Switch

Examples are built when `AUTOLINK_BUILD_EXAMPLES=ON`.

```bash
cmake -S src/autonomy/autolink -B build/autolink -DAUTOLINK_BUILD_EXAMPLES=ON
cmake --build build/autolink -j8
```
