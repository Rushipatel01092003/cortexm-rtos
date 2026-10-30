#include "rtos.hpp"
#include "board.hpp"

namespace {
constexpr std::uint32_t kIcsr = 0xE000ED04;       // Interrupt Control and State Register
constexpr std::uint32_t kIcsrPendSvSet = 1u << 28;
constexpr std::uint32_t kShpr3 = 0xE000ED20;      // System Handler Priority Register 3
constexpr std::uint32_t kSysTickCtrl = 0xE000E010;
constexpr std::uint32_t kSysTickLoad = 0xE000E014;
constexpr std::uint32_t kSysTickVal = 0xE000E018;

rtos::Tcb g_tasks[rtos::kMaxTasks];
std::size_t g_count = 0;
volatile std::uint32_t g_ticks = 0;
std::uint32_t g_switches = 0;

constexpr std::size_t kIdleStackWords = 96;
alignas(8) std::uint32_t g_idle_stack[kIdleStackWords];

inline void request_switch() {
  board::reg(kIcsr) = kIcsrPendSvSet;
  asm volatile("dsb\n isb" ::: "memory");
}

void idle_task(void*) {
  for (;;) asm volatile("wfi");
}

[[noreturn]] void task_exit() {
  {
    rtos::CriticalSection cs;
    rtos::g_current_task()->state = rtos::State::Unused;
  }
  for (;;) request_switch();
}
}  // namespace

extern "C" {
rtos::Tcb* volatile g_current = nullptr;  // read/written by the PendSV/SVC assembly
// Set by SVC_Handler once the first task is running. PendSV ignores requests before that,
// because a SysTick can fire between enabling the timer and launching the first task.
volatile std::uint32_t g_started = 0;

// Picks the next task: highest priority Ready task, round-robin among equals.
void rtos_select_next(void) {
  rtos::Tcb* cur = g_current;
  std::size_t idx = static_cast<std::size_t>(cur - g_tasks);
  rtos::Tcb* best = nullptr;
  for (std::size_t k = 1; k <= g_count; ++k) {
    rtos::Tcb* t = &g_tasks[(idx + k) % g_count];
    if (t->state == rtos::State::Ready && (best == nullptr || t->priority > best->priority)) best = t;
  }
  if (best != cur) ++g_switches;
  g_current = best;  // idle task is always Ready, so best is never null
}

void SysTick_Handler(void) {
  std::uint32_t now = g_ticks + 1;
  g_ticks = now;
  for (std::size_t i = 0; i < g_count; ++i) {
    rtos::Tcb& t = g_tasks[i];
    if (t.state == rtos::State::Sleeping && static_cast<std::int32_t>(now - t.wake_tick) >= 0)
      t.state = rtos::State::Ready;
  }
  request_switch();
}
}

// SVC: launch the first task. Loads its saved context and returns to thread mode on PSP.
// PendSV: save r4-r11 of the running task, pick the next task, restore its context.
// (r0-r3, r12, lr, pc, xpsr are saved/restored by the hardware on exception entry/return.)
asm(R"(
  .syntax unified
  .section .text.rtos_asm, "ax", %progbits
  .global SVC_Handler
  .type SVC_Handler, %function
  .thumb_func
SVC_Handler:
  ldr   r3, =g_current
  ldr   r2, [r3]
  ldr   r0, [r2]
  ldmia r0!, {r4-r11}
  msr   psp, r0
  mov   r1, #1
  ldr   r2, =g_started
  str   r1, [r2]
  ldr   lr, =0xFFFFFFFD
  bx    lr

  .global PendSV_Handler
  .type PendSV_Handler, %function
  .thumb_func
PendSV_Handler:
  ldr   r3, =g_started
  ldr   r3, [r3]
  cbz   r3, 1f
  mrs   r0, psp
  stmdb r0!, {r4-r11}
  ldr   r3, =g_current
  ldr   r2, [r3]
  str   r0, [r2]
  push  {r3, lr}
  cpsid i
  bl    rtos_select_next
  cpsie i
  pop   {r3, lr}
  ldr   r2, [r3]
  ldr   r0, [r2]
  ldmia r0!, {r4-r11}
  msr   psp, r0
  bx    lr
1:
  bx    lr
  .ltorg
)");

namespace rtos {

Tcb* g_current_task() { return g_current; }

void create_task(const char* name, TaskFn fn, void* arg, std::uint8_t priority,
                 std::uint32_t* stack, std::size_t words) {
  CriticalSection cs;
  if (g_count >= kMaxTasks) return;
  for (std::size_t i = 0; i < words; ++i) stack[i] = kStackCanary;

  // Build the initial frame so the first context switch "returns" into fn(arg).
  std::uint32_t* top = stack + words;
  top = reinterpret_cast<std::uint32_t*>(reinterpret_cast<std::uintptr_t>(top) & ~7u);
  top -= 16;
  for (int i = 0; i < 16; ++i) top[i] = 0;               // r4-r11 then r0-r3, r12 = 0
  top[8] = reinterpret_cast<std::uintptr_t>(arg);        // r0 = argument
  top[13] = reinterpret_cast<std::uintptr_t>(&task_exit);  // lr: returning from fn ends the task
  top[14] = reinterpret_cast<std::uintptr_t>(fn);        // pc
  top[15] = 0x01000000;                                  // xPSR: Thumb bit set

  Tcb& t = g_tasks[g_count++];
  t = Tcb{top, name, State::Ready, priority, nullptr, 0, stack, words};
}

void start(std::uint32_t cpu_hz, std::uint32_t tick_hz) {
  create_task("idle", idle_task, nullptr, 0, g_idle_stack, kIdleStackWords);

  // PendSV and SysTick at lowest priority so they never preempt other ISRs.
  board::reg(kShpr3) |= (0xFFu << 16) | (0xFFu << 24);
  board::reg(kSysTickLoad) = cpu_hz / tick_hz - 1;
  board::reg(kSysTickVal) = 0;
  board::reg(kSysTickCtrl) = 0x7;  // core clock, interrupt enabled, counter enabled

  Tcb* first = &g_tasks[0];
  for (std::size_t i = 0; i < g_count; ++i)
    if (g_tasks[i].priority > first->priority) first = &g_tasks[i];
  g_current = first;
  asm volatile("cpsie i\n svc 0" ::: "memory");
  for (;;) {
  }
}

std::uint32_t ticks() { return g_ticks; }
void yield() { request_switch(); }

void sleep_ms(std::uint32_t ms) {
  {
    CriticalSection cs;
    g_current->wake_tick = g_ticks + ms;
    g_current->state = State::Sleeping;
  }
  request_switch();
}

void delay_until(std::uint32_t& last_wake, std::uint32_t period_ms) {
  last_wake += period_ms;
  {
    CriticalSection cs;
    if (static_cast<std::int32_t>(last_wake - g_ticks) <= 0) return;  // already late: don't sleep
    g_current->wake_tick = last_wake;
    g_current->state = State::Sleeping;
  }
  request_switch();
}

std::uint32_t context_switches() { return g_switches; }
std::size_t task_count() { return g_count; }
const Tcb& task_info(std::size_t i) { return g_tasks[i]; }

std::size_t stack_free_words(const Tcb& t) {
  std::size_t n = 0;
  while (n < t.stack_words && t.stack_base[n] == kStackCanary) ++n;
  return n;
}

void Semaphore::take() {
  for (;;) {
    {
      CriticalSection cs;
      if (count_ > 0) {
        --count_;
        return;
      }
      g_current->state = State::Blocked;
      g_current->wait_obj = this;
    }
    request_switch();  // resumes here once give() makes this task Ready; then re-check
  }
}

void Semaphore::give() {
  CriticalSection cs;
  ++count_;
  for (std::size_t i = 0; i < g_count; ++i) {
    Tcb& t = g_tasks[i];
    if (t.state == State::Blocked && t.wait_obj == this) {
      t.state = State::Ready;
      t.wait_obj = nullptr;
      break;
    }
  }
  request_switch();  // pended now, taken as soon as the critical section ends
}

}  // namespace rtos
