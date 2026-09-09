#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* -------------------------------------------------------------
   1. CORE SYSTEM INITIALIZATION SETTINGS
   ------------------------------------------------------------- */
#define configUSE_PREEMPTION                    1  // 1 = Preemptive scheduling (tasks interrupt each other), 0 = Cooperative
#define configCPU_CLOCK_HZ                      ( 16000000UL ) // Default HSI clock speed out of reset (16 MHz) -- UPDATE if you switch to PLL/HSE
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 ) // 1000 Hz = 1ms system heartbeat tick
#define configMAX_PRIORITIES                    ( 10 )  // Task priorities can range from 0 (lowest) to 4 (highest)
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 ) // Minimum allocation pool size per task (in words)
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 20 * 1024 ) ) // Allocate 20KB of RAM to heap_4.c task pool
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0  // 0 = 32-bit tick counter (standard for ARM Cortex)
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_IDLE_HOOK                     0  // Set 1 only if you implement vApplicationIdleHook()
#define configUSE_TICK_HOOK                     0  // Set 1 only if you implement vApplicationTickHook()
#define configUSE_MUTEXES                       1  // Needed for any mutex-based resource protection
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          2  // Method 2 = pattern-fill check, catches more than method 1. You need this on bare hardware.
#define configUSE_TRACE_FACILITY                0
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
/* configASSERT: without this, a failed FreeRTOS internal assertion just
   silently does nothing in some ports, or hard-faults with zero context.
   This traps it at the point of failure so you can actually debug it. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* -------------------------------------------------------------
   2. KERNEL API INCLUSION SWITCHES (1 = Enabled, 0 = Compiled Out)
   ------------------------------------------------------------- */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1  // MANDATORY: Enables the use of vTaskDelay()
#define INCLUDE_xTaskGetSchedulerState          1  // Needed by several queue/semaphore asserts
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_xTaskGetIdleTaskHandle           0
#define INCLUDE_uxTaskGetStackHighWaterMark      1  // Cheap insurance for tuning configMINIMAL_STACK_SIZE later

/* -------------------------------------------------------------
   3. SOFTWARE TIMER CONFIG (mandatory since timers.c is in your build)
   ------------------------------------------------------------- */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* -------------------------------------------------------------
   4. CORTEX-M HARDWARE INTERRUPT PRIORITY MANAGEMENT
   ------------------------------------------------------------- */
#define configPRIO_BITS                         4  // STM32F446RE implements 4 priority bits (16 levels)
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15 // Lowest hardware interrupt priority rank (unshifted)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5 // Highest priority (numerically) an ISR may have and still call FreeRTOS "FromISR" APIs (unshifted)

/* This port's portmacro.h reads configKERNEL_INTERRUPT_PRIORITY and
   configMAX_SYSCALL_INTERRUPT_PRIORITY directly -- both must already be
   left-shifted into the top bits of the priority byte, because that's
   how NVIC priority registers on this silicon are laid out. Do NOT
   plug in 5 and 15 unshifted here -- that was your original bug. */
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )        // 15 << 4 = 240
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )   // 5  << 4 = 80

/* -------------------------------------------------------------
   5. THE HARDWARE VECTOR PATCH PANEL (The Critical Link)
   ------------------------------------------------------------- */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
