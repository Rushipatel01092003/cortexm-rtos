// Bare-metal startup: vector table, .data/.bss init, static constructors, entry to main().
#include <cstdint>

extern "C" {
extern std::uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

int main();
void Reset_Handler(void);
void Default_Handler(void) {
  for (;;) {
  }
}

#define WEAK_DEFAULT __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_DEFAULT;
void MemManage_Handler(void) WEAK_DEFAULT;
void BusFault_Handler(void) WEAK_DEFAULT;
void UsageFault_Handler(void) WEAK_DEFAULT;
void DebugMon_Handler(void) WEAK_DEFAULT;
// Defined elsewhere: HardFault_Handler (fault.cpp); SVC/PendSV/SysTick (rtos.cpp).
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void Reset_Handler(void) {
  for (std::uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) *dst++ = *src++;
  for (std::uint32_t* dst = &_sbss; dst < &_ebss;) *dst++ = 0;
  for (auto fn = __init_array_start; fn < __init_array_end; ++fn) (*fn)();
  main();
  for (;;) {
  }
}
}

// Cortex-M3 vector table (system exceptions only; no external IRQs are used).
asm(R"(
  .syntax unified
  .section .isr_vector, "a", %progbits
  .align 2
  .global g_vectors
g_vectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0, 0, 0, 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .text
)");
