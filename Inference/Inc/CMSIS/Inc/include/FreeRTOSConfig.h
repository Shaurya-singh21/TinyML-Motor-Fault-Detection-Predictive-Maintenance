#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* -------------------------------------------------------------
   1. CORE SYSTEM INITIALIZATION SETTINGS
   ------------------------------------------------------------- */
#define configUSE_PREEMPTION                    1  // 1 = Preemptive scheduling (tasks interrupt each other), 0 = Cooperative
#define configCPU_CLOCK_HZ                      ( 16000000UL ) // Default HSI clock speed out of reset (16 MHz)
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 ) // 1000 Hz = 1ms system heartbeat tick
#define configMAX_PRIORITIES                    ( 5 )  // Task priorities can range from 0 (lowest) to 4 (highest)
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 ) // Minimum allocation pool size per task (in words)
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 20 * 1024 ) ) // Allocate 20KB of RAM to heap_4.c task pool
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0  // 0 = 32-bit tick counter (standard for ARM Cortex)
#define configIDLE_SHOULD_YIELD                 1

/* -------------------------------------------------------------
   2. KERNEL API INCLUSION SWITCHES (1 = Enabled, 0 = Compiled Out)
   ------------------------------------------------------------- */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1  // MANDATORY: Enables the use of vTaskDelay()

/* -------------------------------------------------------------
   3. CORTEX-M HARDWARE INTERRUPT PRIORITY MANAGEMENT
   ------------------------------------------------------------- */
#define configPRIO_BITS                         4  // STM32F446RE utilizes 4 priority bits (16 levels)
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15 // Lowest hardware interrupt priority rank
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5 // Safe threshold for calling FreeRTOS APIs from ISRs

/* -------------------------------------------------------------
   4. THE HARDWARE VECTOR PATCH PANEL (The Critical Link)
   ------------------------------------------------------------- */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
