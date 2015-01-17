#ifndef STITCHTRON9000_FEATURE_TIMER_HPP_
#define STITCHTRON9000_FEATURE_TIMER_HPP_

#include <chrono>
#include <string>
#include <cassert>
#include <ostream>
#include <iomanip>
#include <iostream>
#include <thread>

namespace s9000 {
namespace feature {

typedef std::chrono::seconds sec;
typedef std::chrono::milliseconds ms;
typedef std::chrono::microseconds us;
typedef std::chrono::nanoseconds ns;

/**
 * @brief The is_duration struct
 */
template <typename>
struct is_duration : std::false_type {};

template <typename T, typename U>
struct is_duration<std::chrono::duration<T, U>> : std::true_type {};

/**
 * @brief Unit
 * @return unit as a string
 */
template <typename T>
std::string durationUnit() {
  return "unknown unit";
}

template <>
std::string durationUnit<sec>() {
  return "sec";
}

template <>
std::string durationUnit<ms>() {
  return "ms";
}

template <>
std::string durationUnit<us>() {
  return "us";
}

template <>
std::string durationUnit<ns>() {
  return "ns";
}

template <typename NumType, typename DenType>
double ratio() {
  typedef typename NumType::period NumPeriod;
  typedef typename DenType::period DenPeriod;
  typedef typename std::ratio_divide<NumPeriod, DenPeriod>::type RatioType;
  return static_cast<double>(RatioType::num) / RatioType::den;
}

/**
 * @brief The Timer class
 */
template <typename DurationType>
class Timer {
  static_assert(is_duration<DurationType>::value, "Not a valid duration type");

 public:
  explicit Timer(const std::string& name, int report_every_n_iter = 0)
      : name_(name), report_every_n_iter_(report_every_n_iter) {}

  int iteration() const { return iteration_; }
  const std::string& name() const { return name_; }

  void start() {
    assert(!running_);
    running_ = true;
    start_ = std::chrono::high_resolution_clock::now();
  }

  void stop() {
    elapsed_ = std::chrono::duration_cast<DurationType>(
        std::chrono::high_resolution_clock::now() - start_);
    assert(running_);
    total_ += elapsed_;
    ++iteration_;
    min_ = std::min(elapsed_, min_);
    max_ = std::max(elapsed_, max_);
    running_ = false;
    if (report_every_n_iter_ == 0) return;
    if (!(iteration_ % report_every_n_iter_)) report();
  }

  template <typename T = DurationType>
  double elapsed() const {
    return elapsed_.count() * ratio<DurationType, T>();
  }

  template <typename T = DurationType>
  double min() const {
    return min_.count() * ratio<DurationType, T>();
  }

  template <typename T = DurationType>
  double max() const {
    return max_.count() * ratio<DurationType, T>();
  }

  template <typename T = DurationType>
  double average() const {
    return total_.count() * ratio<DurationType, T>() / iteration_;
  }

  void reset() {
    iteration_ = 0;
    running_ = false;
  }

  template <typename T = DurationType>
  void sleep(int tick) {
    T duration(tick);
    std::this_thread::sleep_for(duration);
  }

  std::string baseUnit() { return durationUnit<DurationType>(); }

  template <typename T = DurationType>
  std::string unit() {
    return durationUnit<T>();
  }

  template <typename T = DurationType>
  void report(std::ostream& os = std::cout) const {
    os << name_ << " - iterations: " << iteration_
       << ", unit: " << durationUnit<T>() << ", average: " << average<T>()
       << " "
       << ", min: " << min<T>() << ", max: " << max<T>() << std::endl;
  }

 private:
  std::string name_{"timer"};
  int iteration_{0};
  int report_every_n_iter_{0};
  bool running_{false};
  std::chrono::high_resolution_clock::time_point start_;
  DurationType min_{DurationType::max()};
  DurationType max_{DurationType::min()};
  DurationType elapsed_{0};
  DurationType total_{0};
};

typedef Timer<sec> TimerSec;
typedef Timer<ms> TimerMs;
typedef Timer<us> TimerUs;
typedef Timer<ns> TimerNs;

template <typename ClockType>
void printClockInfo() {
  std::cout << "- precision: ";
  // if time unit is less or equal one millisecond
  typedef typename ClockType::period RatioType;  // type of time unit
  if (std::ratio_less_equal<RatioType, std::milli>::value) {
    // convert to and print as milliseconds
    typedef typename std::ratio_multiply<RatioType, std::kilo>::type TT;
    std::cout << std::fixed << static_cast<double>(TT::num) / TT::den << " ms"
              << std::endl;
  } else {
    // print as seconds
    std::cout << std::fixed
              << static_cast<double>(RatioType::num) / RatioType::den << " sec"
              << std::endl;
  }
  std::cout << "- is_steady: " << std::boolalpha << ClockType::is_steady
            << std::endl;
}

}  // namespace feature
}  // namespace s9000

#endif  // STITCHTRON9000_FEATURE_TIMER_HPP_
