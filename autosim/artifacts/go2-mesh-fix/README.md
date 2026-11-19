# Go2 mesh rendering fix

All seven Go2 DAE meshes were converted to GLB at full resolution, preserving triangles and material primitives. Previously, decimation reduced the main shell area from 0.3291 to 0.1083 square metres. Go2 now skips decimation.

Five viewpoints were visually inspected. These retain the initial black-background inspection; artifacts/go2-full-gray.png shows the final grey background with black calves and feet visible.
