#include <attributes.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

_Static_assert(sizeof(u8) == 1, "u8 is not 8 bits");
_Static_assert(sizeof(u16) == 2, "u16 is not 16 bits");
_Static_assert(sizeof(u32) == 4, "u32 is not 32 bits");

typedef void (*ISR_Handler)(void);

#define STACK_SIZE 1024

extern void _start(void) INTERRUPT;
extern void _nmi(void) INTERRUPT;
extern void _hardFault(void) INTERRUPT;
extern void _mpuFault(void) INTERRUPT;
extern void _busFault(void) INTERRUPT;
extern void _usageFault(void) INTERRUPT;
extern void _unhandledInterrupt(void) INTERRUPT;

// Stack einrichten
static u32 stack[STACK_SIZE] SECTION(".stackarea");

#define VECTOR_TABLE_LENGTH 51

ISR_Handler const fixed_vector_table[VECTOR_TABLE_LENGTH] SECTION(".isr_vector") USED  =
{
    (ISR_Handler)((u32)stack + sizeof(stack)), // The initial stack pointer
    _start,                                    // The reset handler
    _nmi,                                      // The NMI handler
    _hardFault,                                // The hard fault handler
    _mpuFault,                                 // The MPU fault handler
    _busFault,                                 // The bus fault handler
    _usageFault,                               // The usage fault handler
    (ISR_Handler)0xEFFFE782,                   // Reserved
    0,                                         // Reserved
    0,                                         // Reserved
    0,                                         // Reserved
    _unhandledInterrupt,                       // SVCall handler
    _unhandledInterrupt,                       // Debug monitor handler
    0,                                         // Reserved
    _unhandledInterrupt,                       // The PendSV handler
    _unhandledInterrupt,                       // The SysTick handler

    // IRQs:
    _unhandledInterrupt,                       // 16 Watchdog
    _unhandledInterrupt,                       // 17 Timer0
    _unhandledInterrupt,                       // 18 Timer1
    _unhandledInterrupt,                       // 19 Timer2
    _unhandledInterrupt,                       // 20 Timer3
    _unhandledInterrupt,                       // 21 UART0
    _unhandledInterrupt,                       // 22 UART1
    _unhandledInterrupt,                       // 23 UART2
    _unhandledInterrupt,                       // 24 UART3
    _unhandledInterrupt,                       // 25 PWM 1
    _unhandledInterrupt,                       // 26 I2C0
    _unhandledInterrupt,                       // 27 I2C1
    _unhandledInterrupt,                       // 28 I2C2
    _unhandledInterrupt,                       // 29 SPI
    _unhandledInterrupt,                       // 30 SSP0
    _unhandledInterrupt,                       // 31 SSP1
    _unhandledInterrupt,                       // 32 PLL0
    _unhandledInterrupt,                       // 33 RTC
    _unhandledInterrupt,                       // 34 Ext0
    _unhandledInterrupt,                       // 35 Ext1
    _unhandledInterrupt,                       // 36 Ext2
    _unhandledInterrupt,                       // 37 Ext3 / GPIO
    _unhandledInterrupt,                       // 38 ADC
    _unhandledInterrupt,                       // 39 BOD
    _unhandledInterrupt,                       // 40 USB
    _unhandledInterrupt,                       // 41 CAN
    _unhandledInterrupt,                       // 42 GPDMA
    _unhandledInterrupt,                       // 43 I2S
    _unhandledInterrupt,                       // 44 Ethernet
    _unhandledInterrupt,                       // 45 RITINT
    _unhandledInterrupt,                       // 46 Motor PWM
    _unhandledInterrupt,                       // 47 QuadEncoder
    _unhandledInterrupt,                       // 48 PLL1 (USB)
    _unhandledInterrupt,                       // 49 USB Activity
    _unhandledInterrupt                        // 50 CAN Activity
};


