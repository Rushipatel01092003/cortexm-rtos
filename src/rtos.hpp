#pragma once
// Minimal preemptive RTOS kernel for ARM Cortex-M3.
//  - Fixed-priority scheduling, round-robin among equal priorities
//  - Context switch in PendSV, 1 kHz SysTick tick, tick-based sleep
//  - Counting semaphores with blocking and a blocking bounded queue built on them
#include <cstddef>
#include <cstdint>

namespace rtos {

constexpr std::size_t kMaxTasks = 6;
constexpr std::uint32_t kStackCanary = 0xDEADBEEF;
constexpr std::uint32_t kWaitForever = 0xFFFFFFFFu;

enum class State : std::uint8_t { Unused, Ready, Sleeping, Blocked };

struct Tcb {
  std::uint32_t* sp;  // saved stack pointer; MUST stay the first member (used by assembly)
  const char* name;
  State state;
  std::uint8_t priority;  // higher number = higher priority
  const void* wait_obj;   // object a Blocked task is waiting on
  std::uint32_t wake_tick;
  std::uint32_t* stack_base;
  std::size_t stack_words;
  bool has_timeout = false;  // Blocked task also wakes at wake_tick if nobody signals it
};

using TaskFn = void (*)(void*);

// Masks interrupts for the lifetime of the object (saves/restores PRIMASK, so it nests).
class CriticalSection {
 public:
  CriticalSection() {
    asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask_) : : "memory");
  }
  ~CriticalSection() { asm volatile("msr primask, %0" : : "r"(primask_) : "memory"); }
  CriticalSection(const CriticalSection&) = delete;
  CriticalSection& operator=(const CriticalSection&) = delete;

 private:
  std::uint32_t primask_;
};

// `stack` must be 8-byte aligned.
void create_task(const char* name, TaskFn fn, void* arg, std::uint8_t priority,
                 std::uint32_t* stack, std::size_t words);
[[noreturn]] void start(std::uint32_t cpu_hz, std::uint32_t tick_hz);

Tcb* g_current_task();
std::uint32_t ticks();
void yield();
void sleep_ms(std::uint32_t ms);
// Drift-free periodic delay: wakes at last_wake + period, then advances last_wake.
void delay_until(std::uint32_t& last_wake, std::uint32_t period_ms);

// Diagnostics
std::uint32_t context_switches();
std::size_t task_count();
const Tcb& task_info(std::size_t i);
std::size_t stack_free_words(const Tcb& t);  // high-water mark via stack canary

class Semaphore {
 public:
  constexpr explicit Semaphore(std::uint32_t initial = 0) : count_(initial) {}
  // Blocks until a token is available or timeout_ms elapses. Returns false on timeout.
  // timeout_ms == 0 is a non-blocking try-take.
  bool take(std::uint32_t timeout_ms = kWaitForever);
  void give();  // releases one token, wakes one waiter
 private:
  volatile std::uint32_t count_;
};

// Blocking bounded FIFO, safe for multiple producers/consumers.
template <typename T, std::size_t N>
class Queue {
 public:
  constexpr Queue() : items_(0), space_(N) {}
  // Both return false if the timeout expires first (queue still full / still empty).
  bool push(const T& v, std::uint32_t timeout_ms = kWaitForever) {
    if (!space_.take(timeout_ms)) return false;
    {
      CriticalSection cs;
      buf_[tail_] = v;
      tail_ = (tail_ + 1) % N;
      ++depth_;
      if (depth_ > max_depth_) max_depth_ = depth_;
    }
    items_.give();
    return true;
  }
  bool pop(T& out, std::uint32_t timeout_ms) {
    if (!items_.take(timeout_ms)) return false;
    {
      CriticalSection cs;
      out = buf_[head_];
      head_ = (head_ + 1) % N;
      --depth_;
    }
    space_.give();
    return true;
  }
  T pop() {  // blocks forever
    T v{};
    pop(v, kWaitForever);
    return v;
  }
  std::size_t max_depth() const { return max_depth_; }

 private:
  T buf_[N]{};
  std::size_t head_ = 0, tail_ = 0, depth_ = 0, max_depth_ = 0;
  Semaphore items_;
  Semaphore space_;
};

}  // namespace rtos
