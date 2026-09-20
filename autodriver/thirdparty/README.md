# Third-Party Dependencies

This directory contains third-party libraries and headers required by the quadrupedal robot bridge adapters.

## ZSL-1W SDK

**Location**: `zsl1w/`

**Description**: SDK for ZSongLing ZSL-1W quadrupedal robot

**Structure**:
```
zsl1w/
├── include/
│   └── zsl-1w/
│       └── highlevel.h
└── lib/
    ├── x86_64/
    │   └── libmc_sdk_zsl_1w_x86_64.so
    └── aarch64/
        └── libmc_sdk_zsl_1w_aarch64.so
```

**Source**: Copied from `/home/lichengmin/Project/zsibot_sdk`

**Version**: Check with vendor SDK documentation

**License**: Refer to original SDK license

**Usage**: The CMakeLists.txt will automatically detect and link the appropriate library based on the system architecture.

## Adding New Third-Party Libraries

When adding a new third-party dependency:

1. Create a subdirectory under `third-party/`
2. Organize headers in `<vendor>/include/`
3. Organize libraries in `<vendor>/lib/<arch>/`
4. Update this README
5. Update CMakeLists.txt to find and link the library
6. Document the source and version

## Notes

- These libraries are **not** part of the ROS2 package dependencies
- Libraries are architecture-specific (x86_64, aarch64)
- Ensure proper licensing compliance when distributing
- Consider using git-lfs for large binary files
