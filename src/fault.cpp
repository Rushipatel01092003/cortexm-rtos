// HardFault handler: recovers the stacked exception frame and reports PC/LR/fault status.
#include "board.hpp"
#include "uart.hpp"

extern "C" void hardfault_c(std::uint32_t* frame, std::uint32_t exc_return) {
  uart::puts("\n*** HardFault ***\n  PC   = ");
  uart::print_hex(frame[6]);
  uart::puts("\n  LR   = ");
  uart::print_hex(frame[5]);
  uart::puts("\n  CFSR = ");
  uart::print_hex(board::reg(0xE000ED28));
  uart::puts("\n  HFSR = ");
  uart::print_hex(board::reg(0xE000ED2C));
  uart::puts("\n  BFAR = ");
  uart::print_hex(board::reg(0xE000ED38));
  uart::puts("\n  EXC_RETURN = ");
  uart::print_hex(exc_return);
  uart::putc('\n');
  board::semihost_exit(false);
}

// Pick the stack (MSP or PSP) that was active at the fault, then call the C handler.
asm(R"(
  .syntax unified
  .section .text.HardFault_Handler, "ax", %progbits
  .global HardFault_Handler
  .type HardFault_Handler, %function
  .thumb_func
HardFault_Handler:
  tst   lr, #4
  ite   eq
  mrseq r0, msp
  mrsne r0, psp
  mov   r1, lr
  b     hardfault_c
)");
