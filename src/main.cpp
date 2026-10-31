// Sensor-processing pipeline on a custom Cortex-M3 RTOS:
//   sensor_task (100 Hz, prio 3) -> raw_q -> filter_task (prio 2) -> log_q -> logger_task (prio 1)
// The sensor is simulated (triangle wave + uniform noise) so the project runs without hardware.
#include "board.hpp"
#include "kalman.hpp"
#include "rtos.hpp"
#include "uart.hpp"

namespace {

constexpr std::uint32_t kSamples = 500;
constexpr std::uint32_t kSamplePeriodMs = 10;  // 100 Hz
constexpr std::uint32_t kPrintEvery = 50;
constexpr std::int32_t kNoiseAmp = 300;
// Fault injection: the sensor stalls once; the filter's watchdog must notice and recover.
constexpr std::uint32_t kDropoutAtSample = 250;
constexpr std::uint32_t kDropoutMs = 300;
constexpr std::uint32_t kWatchdogMs = 50;

struct Sample {
  std::uint32_t tick;
  std::int32_t truth;
  std::int32_t raw;
  std::int32_t filtered;
  std::uint32_t latency_ms;
};

struct Stats {
  std::uint64_t sq_err_raw = 0;
  std::uint64_t sq_err_filt = 0;
  std::uint32_t max_latency_ms = 0;
  std::uint32_t timeouts = 0;  // watchdog expirations (no sample within kWatchdogMs)
};

rtos::Queue<Sample, 8> g_raw_q;
rtos::Queue<Sample, 8> g_log_q;
Stats g_stats;

constexpr std::size_t kStackWords = 256;
alignas(8) std::uint32_t g_sensor_stack[kStackWords];
alignas(8) std::uint32_t g_filter_stack[kStackWords];
alignas(8) std::uint32_t g_logger_stack[kStackWords];

std::uint32_t xorshift32() {
  static std::uint32_t s = 2463534242u;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

// Triangle wave, +/-1000 units, period 400 samples.
std::int32_t true_signal(std::uint32_t n) {
  std::int32_t p = static_cast<std::int32_t>(n % 400);
  std::int32_t v = p < 200 ? p : 400 - p;
  return (v - 100) * 10;
}

std::uint32_t isqrt(std::uint64_t v) {
  std::uint64_t r = 0, bit = 1ull << 62;
  while (bit > v) bit >>= 2;
  while (bit) {
    if (v >= r + bit) {
      v -= r + bit;
      r = (r >> 1) + bit;
    } else {
      r >>= 1;
    }
    bit >>= 2;
  }
  return static_cast<std::uint32_t>(r);
}

void sensor_task(void*) {
  std::uint32_t last_wake = rtos::ticks();
  for (std::uint32_t n = 0; n < kSamples; ++n) {
    if (n == kDropoutAtSample) rtos::sleep_ms(kDropoutMs);  // simulated sensor stall
    rtos::delay_until(last_wake, kSamplePeriodMs);
    std::int32_t truth = true_signal(n);
    std::int32_t noise = static_cast<std::int32_t>(xorshift32() % (2 * kNoiseAmp + 1)) - kNoiseAmp;
    g_raw_q.push(Sample{rtos::ticks(), truth, truth + noise, 0, 0});
  }
}

void filter_task(void*) {
  // q: process variance per step (tuned by sweeping 5..3000, best RMS at ~1200),
  // r: measurement variance (uniform +/-300 -> var = 30000)
  KalmanQ16 kf(/*q=*/1200, /*r=*/30000, /*p0=*/30000);
  for (std::uint32_t n = 0; n < kSamples;) {
    Sample s{};
    if (!g_raw_q.pop(s, kWatchdogMs)) {  // watchdog: no data for kWatchdogMs
      ++g_stats.timeouts;
      continue;
    }
    ++n;
    s.filtered = kf.update(s.raw);
    s.latency_ms = rtos::ticks() - s.tick;
    if (s.latency_ms > g_stats.max_latency_ms) g_stats.max_latency_ms = s.latency_ms;
    std::int64_t er = s.raw - s.truth, ef = s.filtered - s.truth;
    g_stats.sq_err_raw += static_cast<std::uint64_t>(er * er);
    g_stats.sq_err_filt += static_cast<std::uint64_t>(ef * ef);
    g_log_q.push(s);
  }
}

void print_padded(const char* str, std::size_t width) {
  std::size_t n = 0;
  while (str[n]) uart::putc(str[n++]);
  for (; n < width; ++n) uart::putc(' ');
}

void print_report(std::uint32_t received) {
  std::uint32_t rms_raw = isqrt(g_stats.sq_err_raw / kSamples);
  std::uint32_t rms_filt = isqrt(g_stats.sq_err_filt / kSamples);
  std::int32_t reduction =
      rms_raw ? 100 - static_cast<std::int32_t>((100 * rms_filt) / rms_raw) : 0;

  uart::puts("\n===== Summary =====\n");
  uart::puts("samples processed   : ");
  uart::print_u(received);
  uart::puts("\nRMS error raw       : ");
  uart::print_u(rms_raw);
  uart::puts("\nRMS error filtered  : ");
  uart::print_u(rms_filt);
  uart::puts("\nnoise reduction     : ");
  uart::print_i(reduction);
  uart::puts("%\nwatchdog timeouts   : ");
  uart::print_u(g_stats.timeouts);
  uart::puts("\nmax pipeline latency: ");
  uart::print_u(g_stats.max_latency_ms);
  uart::puts(" ms\ncontext switches    : ");
  uart::print_u(rtos::context_switches());
  uart::puts("\nmax queue depth     : raw=");
  uart::print_u(static_cast<std::uint32_t>(g_raw_q.max_depth()));
  uart::puts(" log=");
  uart::print_u(static_cast<std::uint32_t>(g_log_q.max_depth()));

  uart::puts("\n\n task    prio  stack-free(words)\n");
  for (std::size_t i = 0; i < rtos::task_count(); ++i) {
    const rtos::Tcb& t = rtos::task_info(i);
    uart::puts(" ");
    print_padded(t.name, 8);
    uart::print_u(t.priority);
    uart::puts("     ");
    uart::print_u(static_cast<std::uint32_t>(rtos::stack_free_words(t)));
    uart::putc('/');
    uart::print_u(static_cast<std::uint32_t>(t.stack_words));
    uart::putc('\n');
  }

  // The injected sensor stall must have been detected by the watchdog.
  bool pass = received == kSamples && rms_filt < rms_raw && g_stats.timeouts >= 1;
  uart::puts(pass ? "\nRESULT: PASS\n" : "\nRESULT: FAIL\n");
  board::semihost_exit(pass);
}

void logger_task(void*) {
  uart::puts("t(ms)   truth    raw  filtered\n");
  std::uint32_t received = 0;
  for (std::uint32_t n = 0; n < kSamples; ++n) {
    Sample s = g_log_q.pop();
    ++received;
    if (n % kPrintEvery == 0) {
      uart::print_u(s.tick);
      uart::puts("\t");
      uart::print_i(s.truth);
      uart::puts("\t");
      uart::print_i(s.raw);
      uart::puts("\t");
      uart::print_i(s.filtered);
      uart::putc('\n');
    }
  }
#ifdef FAULT_TEST
  // Deliberate bus fault (read from unmapped memory) to demonstrate the HardFault handler.
  volatile std::uint32_t bad = *reinterpret_cast<volatile std::uint32_t*>(0xA0000000);
  (void)bad;
#endif
  print_report(received);
}

}  // namespace

int main() {
  uart::init();
  uart::puts("Cortex-M3 RTOS sensor pipeline (QEMU mps2-an385)\n");
  rtos::create_task("sensor", sensor_task, nullptr, 3, g_sensor_stack, kStackWords);
  rtos::create_task("filter", filter_task, nullptr, 2, g_filter_stack, kStackWords);
  rtos::create_task("logger", logger_task, nullptr, 1, g_logger_stack, kStackWords);
  rtos::start(board::kCpuHz, board::kTickHz);
}
