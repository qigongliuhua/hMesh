#pragma once

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace tf {

// 测试结果枚举
enum class TestResult { PASSED, FAILED, ERRORED };

// 单个测试用例信息
struct TestCase {
    std::string name;
    std::function<void()> test_func;
    TestResult result;
    std::string message;
    double duration_ms;
};

// 测试套件
class TestSuite {
   public:
    explicit TestSuite(const std::string& name) : name_(name) {}

    void addTestCase(const std::string& name, std::function<void()> test_func) {
        test_cases_.push_back({name, test_func, TestResult::PASSED, "", 0.0});
    }

    void run() {
        std::cout << "Running test suite: " << name_ << "\n";
        std::cout << "================================\n";

        int passed = 0;
        int failed = 0;
        int errored = 0;

        for (auto& test_case : test_cases_) {
            auto start = std::chrono::high_resolution_clock::now();

            try {
                test_case.test_func();
                test_case.result = TestResult::PASSED;
                passed++;
            } catch (const std::exception& e) {
                test_case.result = TestResult::ERRORED;
                test_case.message = e.what();
                errored++;
            } catch (const std::string& s) {
                test_case.result = TestResult::FAILED;
                test_case.message = s;
                failed++;
            } catch (...) {
                test_case.result = TestResult::ERRORED;
                test_case.message = "Unknown error occurred";
                errored++;
            }

            auto end = std::chrono::high_resolution_clock::now();
            test_case.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();

            printTestCaseResult(test_case);
        }

        std::cout << "================================\n";
        std::cout << "Summary: " << passed << " passed, " << failed << " failed, " << errored
                  << " errored\n";
        std::cout << "Total: " << test_cases_.size() << " tests\n\n";
    }

   private:
    void printTestCaseResult(const TestCase& test_case) {
        std::cout << "[";
        switch (test_case.result) {
            case TestResult::PASSED:
                std::cout << "PASS";
                break;
            case TestResult::FAILED:
                std::cout << "FAIL";
                break;
            case TestResult::ERRORED:
                std::cout << "ERROR";
                break;
        }
        std::cout << "] " << test_case.name << " (" << test_case.duration_ms << " ms)";

        if (test_case.result != TestResult::PASSED) {
            std::cout << " - " << test_case.message;
        }
        std::cout << "\n";
    }

    std::string name_;
    std::vector<TestCase> test_cases_;
};

// 浮点数比较工具
namespace detail {
template <typename T>
constexpr bool is_floating_point_v = std::is_floating_point<T>::value;

template <typename T>
typename std::enable_if_t<is_floating_point_v<T>, bool> almost_equal(
    T a, T b, T epsilon = std::numeric_limits<T>::epsilon()) {
    if (a == b) return true;

    auto diff = std::abs(a - b);
    auto norm = std::min((std::abs(a) + std::abs(b)), std::numeric_limits<T>::max());

    return diff < epsilon * norm;
}

template <typename T>
typename std::enable_if_t<!is_floating_point_v<T>, bool> almost_equal(T a, T b, T = T{}) {
    return a == b;
}
}  // namespace detail

// 断言宏
#define ASSERT_TRUE(expr)                                       \
    if (!(expr)) {                                              \
        std::ostringstream oss;                                 \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "       \
            << "Assertion failed: " << #expr << " is not true"; \
        throw oss.str();                                        \
    }

#define ASSERT_FALSE(expr)                                       \
    if (expr) {                                                  \
        std::ostringstream oss;                                  \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "        \
            << "Assertion failed: " << #expr << " is not false"; \
        throw oss.str();                                         \
    }

#define ASSERT_EQUAL(a, b)                                                                       \
    if ((a) != (b)) {                                                                            \
        std::ostringstream oss;                                                                  \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "                                        \
            << "Assertion failed: " << #a << " (" << (a) << ") != " << #b << " (" << (b) << ")"; \
        throw oss.str();                                                                         \
    }

#define ASSERT_NOT_EQUAL(a, b)                                                                   \
    if ((a) == (b)) {                                                                            \
        std::ostringstream oss;                                                                  \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "                                        \
            << "Assertion failed: " << #a << " (" << (a) << ") == " << #b << " (" << (b) << ")"; \
        throw oss.str();                                                                         \
    }

#define ASSERT_ALMOST_EQUAL(a, b, epsilon)                                               \
    if (!tf::detail::almost_equal((a), (b), (epsilon))) {                                \
        std::ostringstream oss;                                                          \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "                                \
            << "Assertion failed: " << #a << " (" << (a) << ") != " << #b << " (" << (b) \
            << ") within epsilon " << (epsilon);                                         \
        throw oss.str();                                                                 \
    }

#define ASSERT_ALMOST_EQUAL_DEFAULT(a, b)                                                \
    if (!tf::detail::almost_equal((a), (b))) {                                           \
        std::ostringstream oss;                                                          \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "                                \
            << "Assertion failed: " << #a << " (" << (a) << ") != " << #b << " (" << (b) \
            << ") within default epsilon";                                               \
        throw oss.str();                                                                 \
    }

#define ASSERT_THROWS(expr, exception_type)                                           \
    try {                                                                             \
        expr;                                                                         \
        std::ostringstream oss;                                                       \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "                             \
            << "Assertion failed: " << #expr << " did not throw " << #exception_type; \
        throw oss.str();                                                              \
    } catch (const exception_type&) {                                                 \
    } catch (...) {                                                                   \
        std::ostringstream oss;                                                       \
        oss << "[" << __FILE__ << ":" << __LINE__ << "] "                             \
            << "Assertion failed: " << #expr << " threw unexpected exception";        \
        throw oss.str();                                                              \
    }

class Timer {
   public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Nanoseconds = std::chrono::nanoseconds;
    using Microseconds = std::chrono::microseconds;
    using Milliseconds = std::chrono::milliseconds;
    using Seconds = std::chrono::seconds;
    using Minutes = std::chrono::minutes;
    using Hours = std::chrono::hours;
    Timer() : start_time_(Clock::now()) {}
    // 重置计时器
    void reset() { start_time_ = Clock::now(); }
    // 获取经过的时间（纳秒）
    auto elapsed_nanoseconds() const {
        return std::chrono::duration_cast<Nanoseconds>(Clock::now() - start_time_).count();
    }
    // 获取经过的时间（微秒）
    auto elapsed_microseconds() const {
        return std::chrono::duration_cast<Microseconds>(Clock::now() - start_time_).count();
    }
    // 获取经过的时间（毫秒）
    auto elapsed_milliseconds() const {
        return std::chrono::duration_cast<Milliseconds>(Clock::now() - start_time_).count();
    }
    // 获取经过的时间（秒）
    auto elapsed_seconds() const {
        return std::chrono::duration_cast<Seconds>(Clock::now() - start_time_).count();
    }
    // 获取经过的时间（分钟）
    auto elapsed_minutes() const {
        return std::chrono::duration_cast<Minutes>(Clock::now() - start_time_).count();
    }
    // 获取经过的时间（小时）
    auto elapsed_hours() const {
        return std::chrono::duration_cast<Hours>(Clock::now() - start_time_).count();
    }
    // 获取原始duration对象（用于自定义时间单位）
    auto elapsed() const { return Clock::now() - start_time_; }
    // 自动选择合适的时间单位打印
    std::string format_auto() const {
        auto duration = elapsed();
        auto ns = std::chrono::duration_cast<Nanoseconds>(duration).count();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        if (ns < 1000) {
            oss << ns << " ns";
        } else if (ns < 1000000) {
            oss << (ns / 1000.0) << " μs";
        } else if (ns < 1000000000) {
            oss << (ns / 1000000.0) << " ms";
        } else if (ns < 60000000000) {
            oss << (ns / 1000000000.0) << " s";
        } else if (ns < 3600000000000) {
            oss << (ns / 60000000000.0) << " min";
        } else {
            oss << (ns / 3600000000000.0) << " h";
        }
        return oss.str();
    }
    // 打印时间（自动选择单位）
    void print_auto(const std::string& prefix = "") const {
        std::cout << prefix << format_auto() << std::endl;
    }
    // 打印时间（指定单位）
    template <typename Duration>
    void print(const std::string& prefix = "") const {
        auto count = std::chrono::duration_cast<Duration>(elapsed()).count();
        std::cout << prefix << count << " " << unit_name<Duration>() << std::endl;
    }

   private:
    TimePoint start_time_;
    // 获取时间单位的名称
    template <typename Duration>
    static constexpr const char* unit_name() {
        if constexpr (std::is_same_v<Duration, Nanoseconds>) {
            return "ns";
        } else if constexpr (std::is_same_v<Duration, Microseconds>) {
            return "μs";
        } else if constexpr (std::is_same_v<Duration, Milliseconds>) {
            return "ms";
        } else if constexpr (std::is_same_v<Duration, Seconds>) {
            return "s";
        } else if constexpr (std::is_same_v<Duration, Minutes>) {
            return "min";
        } else if constexpr (std::is_same_v<Duration, Hours>) {
            return "h";
        } else {
            return "units";
        }
    }
};
}  // namespace tf
