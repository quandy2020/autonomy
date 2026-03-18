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

#include <string>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/io/zero_copy_stream.h>

#include "Generator.hh"

namespace google {
namespace protobuf {
namespace compiler {
namespace cpp {

Generator::Generator(const std::string& /*name*/) {}

Generator::~Generator() = default;

bool Generator::Generate(const FileDescriptor* file,
                         const std::string& /*parameter*/,
                         GeneratorContext* generator_context,
                         std::string* /*error*/) const {
  const std::string delim = ".proto";
  std::string header_filename(file->name());
  std::string::size_type pos = header_filename.rfind(delim);
  if (pos == std::string::npos)
    return true;
  header_filename.replace(pos, delim.size(), ".pb.h");

  // Insert #include <memory> at the "includes" insertion point
  io::ZeroCopyOutputStream* includes_out =
      generator_context->OpenForInsert(header_filename, "includes");
  if (includes_out) {
    io::Printer includes_printer(includes_out, '$');
    includes_printer.Print("#include <memory>\n");
  }

  // Insert UniquePtr/SharedPtr typedefs at the "namespace_scope" insertion point
  io::ZeroCopyOutputStream* scope_out =
      generator_context->OpenForInsert(header_filename, "namespace_scope");
  if (scope_out) {
    io::Printer scope_printer(scope_out, '$');
    for (int i = 0; i < file->message_type_count(); ++i) {
      const Descriptor* desc = file->message_type(i);
      const std::string& name = desc->name();
      scope_printer.PrintRaw("typedef std::unique_ptr<" + name + "> " + name + "UniquePtr;\n");
      scope_printer.PrintRaw("typedef std::unique_ptr<const " + name + "> Const" + name + "UniquePtr;\n");
      scope_printer.PrintRaw("typedef std::shared_ptr<" + name + "> " + name + "SharedPtr;\n");
      scope_printer.PrintRaw("typedef std::shared_ptr<const " + name + "> Const" + name + "SharedPtr;\n");
    }
  }

  return true;
}

}  // namespace cpp
}  // namespace compiler
}  // namespace protobuf
}  // namespace google
