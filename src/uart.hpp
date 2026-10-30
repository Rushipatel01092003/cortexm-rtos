#pragma once
#include <cstdint>

// Polled driver for the CMSDK APB UART, written against its memory-mapped registers.
namespace uart {
void init();
void putc(char c);
void puts(const char* s);
void print_u(std::uint32_t v);
void print_i(std::int32_t v);
void print_hex(std::uint32_t v);
}  // namespace uart
