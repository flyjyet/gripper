#pragma once

#include <iostream>
#include <string>

// Lightweight test assertion macros for framework-level contract tests.
// No external test framework dependency — sufficient for the current
// three-test-file scope.

#define TEST_ASSERT(cond, msg)                                         \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::cerr << "  FAIL [" << __FILE__ << ":" << __LINE__ << "] "  \
                << (msg) << std::endl;                                 \
      return 1;                                                        \
    }                                                                  \
  } while (0)

#define TEST_ASSERT_EQ(lhs, rhs, msg)                                  \
  do {                                                                 \
    if ((lhs) != (rhs)) {                                              \
      std::cerr << "  FAIL [" << __FILE__ << ":" << __LINE__ << "] "  \
                << (msg) << ": expected " << (rhs) << " got "          \
                << (lhs) << std::endl;                                 \
      return 1;                                                        \
    }                                                                  \
  } while (0)

#define RUN_TEST(name)                                                 \
  do {                                                                 \
    std::cout << "  " << #name << " ..." << std::endl;                \
    int rc = test_##name();                                            \
    if (rc != 0) {                                                     \
      std::cerr << "FAILED: " << #name << std::endl;                  \
      return rc;                                                       \
    }                                                                  \
  } while (0)
