#include "base/base.hpp"

#include <iostream>

int main() {
  std::cout << base_value() << '\n';
  return base_value() == 3 ? 0 : 1;
}
