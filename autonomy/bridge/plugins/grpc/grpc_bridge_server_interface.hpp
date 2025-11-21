/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge { 
namespace plugins {
namespace grpc {

class GrpcBridgeServerInterface 
{
public:
     virtual ~GrpcBridgeServerInterface() {}
   
     // Starts the gRPC server.
     virtual void Start() = 0;
   
     // Waits for the 'GrpcBridgeServerI' to shut down. Note: The server must be
     // either shutting down or some other thread must call 'Shutdown()' for
     // this function to ever return.
     virtual void WaitForShutdown() = 0;
   
     // Waits until all computation is finished (for testing).
     virtual void WaitUntilIdle() = 0;
   
     // Shuts down the gRPC server thread.
     virtual void Shutdown() = 0;
};

  
}   // namespace grpc
}   // namespace plugins 
}   // namespace bridge
}   // namespace autonomy