#include "uart.hpp"
#include "board.hpp"

namespace uart {
namespace {
constexpr std::uint32_t kBase = 0x40004000;
constexpr std::uint32_t kData = kBase + 0x00;
constexpr std::uint32_t kState = kBase + 0x04;
constexpr std::uint32_t kCtrl = kBase + 0x08;
constexpr std::uint32_t kBaudDiv = kBase + 0x10;
constexpr std::uint32_t kStateTxFull = 1u << 0;
constexpr std::uint32_t kCtrlTxEn = 1u << 0;
}  // namespace

void init() {
  board::reg(kBaudDiv) = board::kCpuHz / 115200;
  board::reg(kCtrl) = kCtrlTxEn;
}

void putc(char c) {
  if (c == '\n') putc('\r');
  while (board::reg(kState) & kStateTxFull) {
  }
  board::reg(kData) = static_cast<std::uint32_t>(c);
}

void puts(const char* s) {
  while (*s) putc(*s++);
}

void print_u(std::uint32_t v) {
  char buf[11];
  int i = 0;
  do {
    buf[i++] = static_cast<char>('0' + v % 10);
    v /= 10;
  } while (v);
  while (i) putc(buf[--i]);
}

void print_i(std::int32_t v) {
  if (v < 0) {
    putc('-');
    print_u(0u - static_cast<std::uint32_t>(v));
  } else {
    print_u(static_cast<std::uint32_t>(v));
  }
}

void print_hex(std::uint32_t v) {
  puts("0x");
  for (int s = 28; s >= 0; s -= 4) putc("0123456789ABCDEF"[(v >> s) & 0xF]);
}
}  // namespace uart
