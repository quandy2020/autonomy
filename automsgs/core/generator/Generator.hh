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

#ifndef AUTOMSGS_GENERATOR_HH_
#define AUTOMSGS_GENERATOR_HH_

#include <google/protobuf/compiler/code_generator.h>
#include <string>

namespace google {
namespace protobuf {
namespace compiler {
namespace cpp {

/// \brief Protobuf compiler plugin for automsgs: generates .pb_index and
/// a wrapper header with UniquePtr/SharedPtr typedefs.
class Generator : public CodeGenerator
{
 public:
  explicit Generator(const std::string& name);
  ~Generator() override;

  bool Generate(const FileDescriptor* file,
                const std::string& parameter,
                GeneratorContext* generator_context,
                std::string* error) const override;
};

}  // namespace cpp
}  // namespace compiler
}  // namespace protobuf
}  // namespace google

#endif  // AUTOMSGS_GENERATOR_HH_
