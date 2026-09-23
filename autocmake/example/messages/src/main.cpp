#include "proto/greeting.pb.h"

#include <iostream>
#include <string>

int main() {
  messages::Greeting greeting;
  greeting.set_text("ok");
  greeting.set_value(3);
  greeting.mutable_note()->set_text("proto3");
  if (greeting.note().text() != std::string("proto3")) {
    return 1;
  }
  std::cout << greeting.text() << ' ' << greeting.value() << '\n';
  return 0;
}
