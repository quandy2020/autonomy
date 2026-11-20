#include <iostream>
#include <string>
#include <vector>
int main() {
  const std::vector<std::string> words{"cross-build", "C++", "smoke"};
  for (const auto& word : words) {
    std::cout << word << ' ';
  }
  std::cout << '\n';
}
