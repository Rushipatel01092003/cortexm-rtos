#pragma once
#include <cstdint>

// Board constants for QEMU mps2-an385 (ARM Cortex-M3 @ 25 MHz, CMSDK UART).
namespace board {
constexpr std::uint32_t kCpuHz = 25'000'000;
constexpr std::uint32_t kTickHz = 1000;

// Reads/writes a memory-mapped 32-bit register.
inline volatile std::uint32_t& reg(std::uint32_t addr) {
  return *reinterpret_cast<volatile std::uint32_t*>(addr);
}

// ARM semihosting exit: lets QEMU terminate with a host exit code (used by `make test`).
[[noreturn]] inline void semihost_exit(bool ok) {
  register std::uint32_t r0 asm("r0") = 0x18;                      // SYS_EXIT
  register std::uint32_t r1 asm("r1") = ok ? 0x20026u : 0x20023u;  // ApplicationExit / RunTimeError
  asm volatile("bkpt 0xAB" : : "r"(r0), "r"(r1) : "memory");
  for (;;) asm volatile("wfi");
}
}  // namespace board
