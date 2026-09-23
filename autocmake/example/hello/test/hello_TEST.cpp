#include "hello/greet/greet.hpp"
#include "hello/hello.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(Hello, Value) { EXPECT_EQ(hello_value(), 7); }

TEST(Hello, Greet) { EXPECT_EQ(greet(), std::string("hello")); }
