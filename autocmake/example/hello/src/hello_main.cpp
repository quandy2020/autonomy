#include "hello/hello.hpp"

#include <iostream>

int main() {
  std::cout << hello_value() << '\n';
  return hello_value() == 7 ? 0 : 1;
}
