/**
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

#include "autolink/class_loader/test/base.hpp"

#include <iostream>
#include <string>

#include "autolink/class_loader/class_loader_register_macro.hpp"

namespace {

void Print(const std::string& name) {
    std::cout << name << "::DoSomething" << std::endl;
}

}  // namespace

class Apple : public Base {
   public:
    void DoSomething() override { Print("Apple"); }
};

class Pear : public Base {
   public:
    void DoSomething() override { Print("Pear"); }
};

class Banana : public Base {
   public:
    void DoSomething() override { Print("Banana"); }
};

class Peach : public Base {
   public:
    void DoSomething() override { Print("Peach"); }
};

CLASS_LOADER_REGISTER_CLASS(Apple, Base)
CLASS_LOADER_REGISTER_CLASS(Pear, Base)
CLASS_LOADER_REGISTER_CLASS(Banana, Base)
CLASS_LOADER_REGISTER_CLASS(Peach, Base)


