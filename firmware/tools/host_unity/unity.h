// HOST-ONLY stand-in for the subset of Unity (ThrowTheSwitch) that firmware/test/*/test_main.cpp uses.
// It exists so the SAME test sources can be built with a plain g++ when PlatformIO's native platform
// cannot be installed (tools/run_host_tests.sh). PlatformIO never sees this file: `pio test -e native`
// uses the real Unity 2.6.1 from platformio.ini. Keep the macros below a strict subset of Unity's:
// same names, same argument order (expected first), same effect.
#ifndef HOST_UNITY_SHIM_H
#define HOST_UNITY_SHIM_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

void setUp(void);     // defined by each test file, as with the real Unity
void tearDown(void);  // defined by each test file

namespace host_unity {
struct Failure {};
inline int& tests_run() { static int n = 0; return n; }
inline int& tests_failed() { static int n = 0; return n; }
inline const char*& current_test() { static const char* name = ""; return name; }
inline bool& current_failed() { static bool failed = false; return failed; }
inline void fail(const char* file, int line, const char* what) {
  std::printf("%s:%d:%s:FAIL: %s\n", file, line, current_test(), what);
  current_failed() = true;
  throw Failure();
}
// The arguments are only used inside this call, so a temporary such as ``make_string().c_str()`` stays alive.
inline void assert_equal_string(const char* expected, const char* actual, const char* file, int line) {
  if (expected == nullptr || actual == nullptr || std::strcmp(expected, actual) != 0) {
    char message[400];
    std::snprintf(message, sizeof(message), "Expected '%s' Was '%s'", expected ? expected : "(null)", actual ? actual : "(null)");
    fail(file, line, message);
  }
}
}  // namespace host_unity

#define UNITY_BEGIN() (std::printf("-- host unity shim --\n"), 0)

#define UNITY_END() \
  (std::printf("\n-----------------------\n%d Tests %d Failures 0 Ignored\n%s\n", host_unity::tests_run(), host_unity::tests_failed(), \
               host_unity::tests_failed() == 0 ? "OK" : "FAIL"), \
   host_unity::tests_failed() == 0 ? 0 : 1)

#define RUN_TEST(func)                                           \
  do {                                                           \
    host_unity::current_test() = #func;                          \
    host_unity::current_failed() = false;                        \
    ++host_unity::tests_run();                                   \
    try {                                                        \
      setUp();                                                   \
      func();                                                    \
    } catch (const host_unity::Failure&) {                       \
    }                                                            \
    try {                                                        \
      tearDown();                                                \
    } catch (const host_unity::Failure&) {                       \
    }                                                            \
    if (host_unity::current_failed()) {                          \
      ++host_unity::tests_failed();                              \
    } else {                                                     \
      std::printf("%s:%d:%s:PASS\n", __FILE__, __LINE__, #func); \
    }                                                            \
  } while (0)

#define TEST_FAIL_MESSAGE(message) host_unity::fail(__FILE__, __LINE__, (message))
#define TEST_ASSERT_TRUE(condition) \
  do { if (!(condition)) host_unity::fail(__FILE__, __LINE__, "Expected TRUE Was FALSE: " #condition); } while (0)
#define TEST_ASSERT_FALSE(condition) \
  do { if ((condition)) host_unity::fail(__FILE__, __LINE__, "Expected FALSE Was TRUE: " #condition); } while (0)
#define TEST_ASSERT_EQUAL_INT(expected, actual)                                                        \
  do {                                                                                                 \
    const long long e_ = static_cast<long long>(expected), a_ = static_cast<long long>(actual);        \
    if (e_ != a_) {                                                                                    \
      char m_[160];                                                                                    \
      std::snprintf(m_, sizeof(m_), "Expected %lld Was %lld (" #actual ")", e_, a_);                   \
      host_unity::fail(__FILE__, __LINE__, m_);                                                        \
    }                                                                                                  \
  } while (0)
#define TEST_ASSERT_EQUAL_UINT8(expected, actual) TEST_ASSERT_EQUAL_INT(expected, actual)
#define TEST_ASSERT_EQUAL_UINT32(expected, actual) TEST_ASSERT_EQUAL_INT(expected, actual)
#define TEST_ASSERT_EQUAL_UINT16(expected, actual) TEST_ASSERT_EQUAL_INT(expected, actual)
#define TEST_ASSERT_EQUAL_INT32(expected, actual) TEST_ASSERT_EQUAL_INT(expected, actual)
#define TEST_ASSERT_EQUAL(expected, actual) TEST_ASSERT_EQUAL_INT(expected, actual)
#define TEST_ASSERT_UINT32_WITHIN(delta, expected, actual)                                                        \
  do {                                                                                                            \
    const long long e_ = static_cast<long long>(expected), a_ = static_cast<long long>(actual);                   \
    const long long d_ = e_ > a_ ? e_ - a_ : a_ - e_;                                                             \
    if (d_ > static_cast<long long>(delta)) {                                                                     \
      char m_[160];                                                                                               \
      std::snprintf(m_, sizeof(m_), "Expected %lld Was %lld (within %lld) (" #actual ")", e_, a_, static_cast<long long>(delta)); \
      host_unity::fail(__FILE__, __LINE__, m_);                                                                   \
    }                                                                                                             \
  } while (0)
#define TEST_ASSERT_EQUAL_STRING(expected, actual) host_unity::assert_equal_string((expected), (actual), __FILE__, __LINE__)
#define TEST_ASSERT_DOUBLE_WITHIN(delta, expected, actual)                                             \
  do { if (!(std::fabs(static_cast<double>(expected) - static_cast<double>(actual)) <= static_cast<double>(delta))) \
         host_unity::fail(__FILE__, __LINE__, "Expected a double within delta of " #expected); } while (0)

#endif  // HOST_UNITY_SHIM_H
