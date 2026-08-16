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

# Install generate script (same pattern as gz-msgs: PROJECT_NAME_generate.py)
install(PROGRAMS automsgs_msgs_generate.py
  RENAME automsgs_generate.py
  DESTINATION ${CMAKE_INSTALL_BINDIR})

# Install factory generator script (generates MessageTypes.hh from .pb_index files)
install(PROGRAMS automsgs_generate_factory.py
  RENAME automsgs_generate_factory.py
  DESTINATION ${CMAKE_INSTALL_BINDIR})

# RPC CLI (Python wrapper around grpcurl for proto/rpcs)
install(PROGRAMS cli/rpc-cli.py
  RENAME rpc-cli
  DESTINATION ${CMAKE_INSTALL_BINDIR})
