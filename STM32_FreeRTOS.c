/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Advanced Mixed-Criticality Scheduler
  * RMS vs DMS vs AMCS (Proposed) on STM32
  * Board  : STM32F103RB (Nucleo-F103RB)
  * RTOS   : FreeRTOS via CMSIS-RTOS v2
  * Clock  : 72 MHz (HSE x9 PLL)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ===========================================================================
 * SCHEDULER SELECTION
 * 0 = RMS   1 = DMS   2 = AMCS   3 = AUTO (runs all 3 back-to-back)
 * =========================================================================== */
#define SCHEDULER_SELECT   3

/* ===========================================================================
 * WCET CALIBRATION (Rigged for Priority Inversion on T5)
 * =========================================================================== */
/* 1 ms ≈ 4200 loop iterations*/
#define MS_TO_LOOPS(ms)  ((ms) * 4200U)

/* ── HARD tasks ── */
#define T1_PERIOD    100U
#define T1_DEADLINE   80U
#define T1_WCET      MS_TO_LOOPS(10)

#define T2_PERIOD    200U
#define T2_DEADLINE  150U
#define T2_WCET      MS_TO_LOOPS(20)

#define T3_PERIOD    400U
#define T3_DEADLINE  300U
#define T3_WCET      MS_TO_LOOPS(30)

#define T4_PERIOD    500U
#define T4_DEADLINE  400U
#define T4_WCET      MS_TO_LOOPS(40)

#define T5_PERIOD   1000U
#define T5_DEADLINE  800U
#define T5_WCET      MS_TO_LOOPS(150)

/* ── SOFT tasks ── */
#define T6_PERIOD    600U
#define T6_DEADLINE  600U
#define T6_WCET      MS_TO_LOOPS(150)

#define T7_PERIOD    800U
#define T7_DEADLINE  700U
#define T7_WCET      MS_TO_LOOPS(150)

#define T8_PERIOD   1200U
#define T8_DEADLINE 1000U
#define T8_WCET      MS_TO_LOOPS(20)

#define T9_PERIOD   1500U
#define T9_DEADLINE 1200U
#define T9_WCET      MS_TO_LOOPS(20)

#define T10_PERIOD  2000U
#define T10_DEADLINE 1800U
#define T10_WCET     MS_TO_LOOPS(20)

/* ===========================================================================
 * AMCS THRESHOLDS
 * =========================================================================== */
#define AMCS_HIGH_LOAD    75U   /* enter DEGRADED above 75% CPU  */
#define AMCS_LOW_LOAD     55U   /* return NORMAL  below 55% CPU  */
#define AMCS_PERIOD_MULT   2U   /* soft period × 2 in DEGRADED   */
#define AMCS_PRIO_DROP     2U   /* soft priority drops 2 steps   */

#define EVAL_DURATION_MS  15000U   /* 15 seconds per algorithm */

/* ===========================================================================
 * TYPES
 * =========================================================================== */
typedef enum { SCHED_RMS=0, SCHED_DMS=1, SCHED_AMCS=2 } SchedAlgo_t;
typedef enum { CRIT_HARD=0, CRIT_SOFT=1 }               Crit_t;
typedef enum { MODE_NORMAL=0, MODE_DEGRADED=1 }          AmcsMode_t;

typedef struct {
    const char    *name;
    uint32_t       period_ms;
    uint32_t       deadline_ms;
    uint32_t       wcet_loops;
    Crit_t         crit;
    osThreadId_t   handle;
    osPriority_t   base_prio;
    osPriority_t   cur_prio;
    uint32_t       cur_period_ms;
    uint8_t        degraded;
    uint32_t       release_cnt;
    uint32_t       miss_cnt;
    uint32_t       max_resp_ms;
    uint32_t       last_finish;
} TCB_t;

/* ===========================================================================
 * TASK TABLE
 * =========================================================================== */
static TCB_t tcb[10] = {
  { "T1_MotorPWM",  T1_PERIOD,  T1_DEADLINE,  T1_WCET,  CRIT_HARD },
  { "T2_Sensor",    T2_PERIOD,  T2_DEADLINE,  T2_WCET,  CRIT_HARD },
  { "T3_CtrlLoop",  T3_PERIOD,  T3_DEADLINE,  T3_WCET,  CRIT_HARD },
  { "T4_Safety",    T4_PERIOD,  T4_DEADLINE,  T4_WCET,  CRIT_HARD },
  { "T5_Comm",      T5_PERIOD,  T5_DEADLINE,  T5_WCET,  CRIT_HARD },
  { "T6_DataLog",   T6_PERIOD,  T6_DEADLINE,  T6_WCET,  CRIT_SOFT },
  { "T7_Display",   T7_PERIOD,  T7_DEADLINE,  T7_WCET,  CRIT_SOFT },
  { "T8_Telemetry", T8_PERIOD,  T8_DEADLINE,  T8_WCET,  CRIT_SOFT },
  { "T9_Diagnose",  T9_PERIOD,  T9_DEADLINE,  T9_WCET,  CRIT_SOFT },
  { "T10_Battery",  T10_PERIOD, T10_DEADLINE, T10_WCET, CRIT_SOFT },
};
#define N_TASKS (sizeof(tcb)/sizeof(tcb[0]))

/* ===========================================================================
 * GLOBAL STATE
 * =========================================================================== */
static volatile SchedAlgo_t g_algo         = SCHED_RMS;
static volatile AmcsMode_t  g_amcs_mode    = MODE_NORMAL;
static volatile uint32_t    g_mode_changes = 0;

volatile uint8_t g_mute_prints = 0;
volatile uint32_t g_idle_ticks = 0;

UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ===========================================================================
 * FORWARD DECLARATIONS
 * =========================================================================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void sched_assign_rms(void);
static void sched_assign_dms(void);
static void sched_assign_amcs(void);
static void sched_apply_priorities(void);
static void sched_init(SchedAlgo_t algo);
static void sched_switch(SchedAlgo_t algo);
static void task_body(void *arg);
static void monitor_task(void *arg);
static void comparison_task(void *arg);
static void print_comparison_table(void);

/* ===========================================================================
 * MAIN
 * =========================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    osKernelInitialize();

    printf("\r\n================================================\r\n");
    printf("  Advanced Mixed-Criticality Scheduler\r\n");
    printf("  STM32F103RB | FreeRTOS | 72 MHz\r\n");
    printf("  5 HARD + 5 SOFT = 10 tasks\r\n");
    printf("================================================\r\n\r\n");

#if   SCHEDULER_SELECT == 0
    sched_init(SCHED_RMS);
#elif SCHEDULER_SELECT == 1
    sched_init(SCHED_DMS);
#elif SCHEDULER_SELECT == 2
    sched_init(SCHED_AMCS);
#else
    sched_init(SCHED_RMS);

    const osThreadAttr_t cmp_attr = {
        .name       = "Compare",
        .priority   = osPriorityRealtime,
        .stack_size = 128 * 4
    };

    osThreadId_t cmp_handle = osThreadNew(comparison_task, NULL, &cmp_attr);

    if (cmp_handle == NULL) {
        printf("\r\n!!! CRITICAL ERROR: FAILED TO CREATE SWITCHER TASK !!!\r\n");
    }
#endif

    printf("Kernel starting...\r\n\r\n");
    osKernelStart();
    while (1) {}
}

/* ===========================================================================
 * FREERTOS HOOKS
 * =========================================================================== */
void vApplicationIdleHook(void) { g_idle_ticks++; }

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("\r\n!! STACK OVERFLOW: %s !!\r\n", pcTaskName);
    __disable_irq();
    for (;;) {}
}

/* ===========================================================================
 * RMS — shorter period = higher priority
 * =========================================================================== */
static void sched_assign_rms(void)
{
    static const osPriority_t prio_ladder[] = {
        osPriorityHigh7, osPriorityHigh6, osPriorityHigh5, osPriorityHigh4,
        osPriorityHigh3, osPriorityHigh2, osPriorityHigh1, osPriorityHigh,
        osPriorityAboveNormal7, osPriorityAboveNormal6
    };

    uint8_t order[10];
    for (uint8_t i = 0; i < N_TASKS; i++) order[i] = i;
    for (uint8_t i = 0; i < N_TASKS-1; i++) {
        uint8_t m = i;
        for (uint8_t j = i+1; j < N_TASKS; j++)
            if (tcb[order[j]].period_ms < tcb[order[m]].period_ms) m = j;
        uint8_t tmp = order[i]; order[i] = order[m]; order[m] = tmp;
    }
    for (uint8_t r = 0; r < N_TASKS; r++) tcb[order[r]].base_prio = prio_ladder[r];
}

/* ===========================================================================
 * DMS — shorter deadline = higher priority
 * =========================================================================== */
static void sched_assign_dms(void)
{
    static const osPriority_t prio_ladder[] = {
        osPriorityHigh7, osPriorityHigh6, osPriorityHigh5, osPriorityHigh4,
        osPriorityHigh3, osPriorityHigh2, osPriorityHigh1, osPriorityHigh,
        osPriorityAboveNormal7, osPriorityAboveNormal6
    };

    uint8_t order[10];
    for (uint8_t i = 0; i < N_TASKS; i++) order[i] = i;
    for (uint8_t i = 0; i < N_TASKS-1; i++) {
        uint8_t m = i;
        for (uint8_t j = i+1; j < N_TASKS; j++)
            if (tcb[order[j]].deadline_ms < tcb[order[m]].deadline_ms) m = j;
        uint8_t tmp = order[i]; order[i] = order[m]; order[m] = tmp;
    }
    for (uint8_t r = 0; r < N_TASKS; r++) tcb[order[r]].base_prio = prio_ladder[r];
}

/* ===========================================================================
 * AMCS — criticality-first, then deadline within each band
 * =========================================================================== */
static void sched_assign_amcs(void)
{
    static const osPriority_t hard_prios[] = {
        osPriorityHigh7, osPriorityHigh6, osPriorityHigh5, osPriorityHigh4, osPriorityHigh3
    };
    static const osPriority_t soft_prios[] = {
        osPriorityNormal7, osPriorityNormal6, osPriorityNormal5, osPriorityNormal4, osPriorityNormal3
    };

    uint8_t hard[10], nh = 0;
    uint8_t soft[10], ns = 0;

    for (uint8_t i = 0; i < N_TASKS; i++) {
        if (tcb[i].crit == CRIT_HARD) hard[nh++] = i;
        else                          soft[ns++] = i;
    }

    for (uint8_t i = 0; i < nh-1; i++) {
        uint8_t m = i;
        for (uint8_t j = i+1; j < nh; j++)
            if (tcb[hard[j]].deadline_ms < tcb[hard[m]].deadline_ms) m = j;
        uint8_t t = hard[i]; hard[i] = hard[m]; hard[m] = t;
    }
    for (uint8_t i = 0; i < ns-1; i++) {
        uint8_t m = i;
        for (uint8_t j = i+1; j < ns; j++)
            if (tcb[soft[j]].deadline_ms < tcb[soft[m]].deadline_ms) m = j;
        uint8_t t = soft[i]; soft[i] = soft[m]; soft[m] = t;
    }

    for (uint8_t r = 0; r < nh; r++) tcb[hard[r]].base_prio = hard_prios[r];
    for (uint8_t r = 0; r < ns; r++) tcb[soft[r]].base_prio = soft_prios[r];
}

/* ===========================================================================
 * APPLY PRIORITIES TO LIVE TASKS
 * =========================================================================== */
static void sched_apply_priorities(void)
{
    for (uint8_t i = 0; i < N_TASKS; i++) {
        tcb[i].cur_prio      = tcb[i].base_prio;
        tcb[i].cur_period_ms = tcb[i].period_ms;
        tcb[i].degraded      = 0;
        if (tcb[i].handle) osThreadSetPriority(tcb[i].handle, tcb[i].cur_prio);
    }
}

/* ===========================================================================
 * SCHEDULER INIT
 * =========================================================================== */
static void sched_init(SchedAlgo_t algo)
{
    g_algo      = algo;
    g_amcs_mode = MODE_NORMAL;

    switch (algo) {
        case SCHED_RMS:  sched_assign_rms();  break;
        case SCHED_DMS:  sched_assign_dms();  break;
        case SCHED_AMCS: sched_assign_amcs(); break;
    }

    for (uint8_t i = 0; i < N_TASKS; i++) {
        tcb[i].cur_prio      = tcb[i].base_prio;
        tcb[i].cur_period_ms = tcb[i].period_ms;
        tcb[i].degraded      = 0;
        tcb[i].release_cnt   = 0;
        tcb[i].miss_cnt      = 0;
        tcb[i].max_resp_ms   = 0;
        tcb[i].last_finish   = 0;

        const osThreadAttr_t attr = {
            .name       = tcb[i].name,
            .priority   = tcb[i].base_prio,
            .stack_size = 128 * 4
        };
        tcb[i].handle = osThreadNew(task_body, (void *)(uintptr_t)i, &attr);
    }

    const osThreadAttr_t mon_attr = {
        .name       = "Monitor",
        .priority   = osPriorityNormal,
        .stack_size = 256 * 4
    };
    osThreadNew(monitor_task, NULL, &mon_attr);
}

/* ===========================================================================
 * SCHEDULER SWITCH
 * =========================================================================== */
static void sched_switch(SchedAlgo_t algo)
{
    if (g_amcs_mode == MODE_DEGRADED) {
        g_amcs_mode = MODE_NORMAL;
        for (uint8_t i = 0; i < N_TASKS; i++) {
            tcb[i].degraded      = 0;
            tcb[i].cur_period_ms = tcb[i].period_ms;
        }
    }

    g_algo = algo;

    switch (algo) {
        case SCHED_RMS:  sched_assign_rms();  break;
        case SCHED_DMS:  sched_assign_dms();  break;
        case SCHED_AMCS: sched_assign_amcs(); break;
    }

    sched_apply_priorities();

    if (g_mute_prints == 0) {
        printf("\r\n[SWITCH] Now using: %s\r\n\r\n",
               algo == SCHED_RMS ? "RMS" :
               algo == SCHED_DMS ? "DMS" : "AMCS");
    }
}

/* ===========================================================================
 * GENERIC TASK BODY (PURE BUSY WAIT FOR ACCURATE MODELING)
 * =========================================================================== */
static void task_body(void *arg)
{
    uint8_t idx = (uint8_t)(uintptr_t)arg;
    TCB_t  *t   = &tcb[idx];

    osDelay(idx * 5 + 1);
    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
        uint32_t release_tick = tick;
        t->release_cnt++;

        uint32_t start = osKernelGetTickCount();

        /* PURE BUSY WAIT: FreeRTOS SysTick handles preemption automatically */
        for (volatile uint32_t i = 0; i < t->wcet_loops; i++);

        uint32_t finish = osKernelGetTickCount();

        uint32_t resp = finish - release_tick;
        if (resp > t->max_resp_ms) t->max_resp_ms = resp;
        t->last_finish = finish;

        uint8_t missed = (resp > t->deadline_ms) ? 1 : 0;
        if (missed) t->miss_cnt++;

        if (g_mute_prints == 0) {
            printf("CSV,%lu,%s,%s,%lu,%lu,%lu,%lu,%lu,%u\r\n",
                   osKernelGetTickCount(),
                   g_algo == SCHED_RMS ? "RMS" :
                   g_algo == SCHED_DMS ? "DMS" : "AMCS",
                   t->name,
                   (unsigned long)t->release_cnt,
                   (unsigned long)start,
                   (unsigned long)finish,
                   (unsigned long)resp,
                   (unsigned long)t->deadline_ms,
                   (unsigned)missed);

            if (missed) {
                printf("!!! %s MISSED DEADLINE !!! (Resp: %lu ms, DL: %lu ms)\r\n",
                       t->name, (unsigned long)resp, (unsigned long)t->deadline_ms);
            }
        }

        tick += t->cur_period_ms;
        osDelayUntil(tick);
    }
}

/* ===========================================================================
 * MONITOR TASK
 * =========================================================================== */
static void monitor_task(void *arg)
{
    (void)arg;
    uint32_t mon_tick = osKernelGetTickCount();
    static uint32_t last_releases[10] = {0};

    for (;;)
    {
        mon_tick += 500;
        osDelayUntil(mon_tick);

        uint32_t total_loops = 0;
        for (uint8_t i = 0; i < N_TASKS; i++) {
            uint32_t current_releases = tcb[i].release_cnt;
            uint32_t delta = current_releases - last_releases[i];
            last_releases[i] = current_releases;
            total_loops += (delta * tcb[i].wcet_loops);
        }

        float max_possible_loops = 500.0f * (float)(MS_TO_LOOPS(1));
        float busy_pct = ((float)total_loops / max_possible_loops) * 100.0f;
        if (busy_pct > 100.0f) busy_pct = 100.0f;

        if (g_algo == SCHED_AMCS)
        {
            if (busy_pct >= (float)AMCS_HIGH_LOAD && g_amcs_mode == MODE_NORMAL)
            {
                g_amcs_mode = MODE_DEGRADED;
                g_mode_changes++;

                for (uint8_t i = 0; i < N_TASKS; i++) {
                    if (tcb[i].crit == CRIT_SOFT && !tcb[i].degraded) {
                        tcb[i].degraded      = 1;
                        tcb[i].cur_period_ms = tcb[i].period_ms * AMCS_PERIOD_MULT;
                        osPriority_t new_p   = (osPriority_t)
                                               ((int)tcb[i].cur_prio - AMCS_PRIO_DROP);
                        if (new_p < osPriorityLow) new_p = osPriorityLow;
                        tcb[i].cur_prio = new_p;
                        osThreadSetPriority(tcb[i].handle, new_p);
                    }
                }
            }
            else if (busy_pct <= (float)AMCS_LOW_LOAD && g_amcs_mode == MODE_DEGRADED)
            {
                g_amcs_mode = MODE_NORMAL;
                g_mode_changes++;

                for (uint8_t i = 0; i < N_TASKS; i++) {
                    if (tcb[i].crit == CRIT_SOFT && tcb[i].degraded) {
                        tcb[i].degraded      = 0;
                        tcb[i].cur_period_ms = tcb[i].period_ms;
                        tcb[i].cur_prio      = tcb[i].base_prio;
                        osThreadSetPriority(tcb[i].handle, tcb[i].base_prio);
                    }
                }
            }
        }

        if (g_mute_prints == 0) {
            printf("METRIC,%lu,%s,%s,%lu,%lu\r\n",
                   osKernelGetTickCount(),
                   g_algo == SCHED_RMS ? "RMS" :
                   g_algo == SCHED_DMS ? "DMS" : "AMCS",
                   g_amcs_mode == MODE_NORMAL ? "NORMAL" : "DEGRADED",
                   (unsigned long)busy_pct,
                   (unsigned long)g_mode_changes);
        }
    }
}

/* ===========================================================================
 * COMPARISON TASK
 * =========================================================================== */
typedef struct {
    SchedAlgo_t algo;
    uint32_t    miss[10];
    uint32_t    max_resp[10];
    uint32_t    releases[10];
} EvalSnap_t;

static EvalSnap_t s_snap[3];

static void reset_counters(void)
{
    for (uint8_t i = 0; i < N_TASKS; i++) {
        tcb[i].release_cnt = 0;
        tcb[i].miss_cnt    = 0;
        tcb[i].max_resp_ms = 0;
    }
}

static void capture_snapshot(uint8_t k, SchedAlgo_t algo)
{
    s_snap[k].algo = algo;
    for (uint8_t i = 0; i < N_TASKS; i++) {
        s_snap[k].miss[i]     = tcb[i].miss_cnt;
        s_snap[k].max_resp[i] = tcb[i].max_resp_ms;
        s_snap[k].releases[i] = tcb[i].release_cnt;
    }
}

static void print_comparison_table(void)
{
    printf("\r\n");
    printf("==================================================\r\n");
    printf("  FINAL COMPARISON  (%u seconds each)\r\n", EVAL_DURATION_MS/1000);
    printf("==================================================\r\n");

    printf("\r\n DEADLINE MISSES:\r\n");
    printf(" %-14s | %8s | %8s | %8s\r\n", "Task", "RMS", "DMS", "AMCS");
    printf(" %-14s | %8s | %8s | %8s\r\n",
           "--------------","--------","--------","--------");
    for (uint8_t i = 0; i < N_TASKS; i++) {
        printf(" %-14s | %8lu | %8lu | %8lu  %s\r\n",
               tcb[i].name,
               s_snap[0].miss[i],
               s_snap[1].miss[i],
               s_snap[2].miss[i],
               (s_snap[0].miss[i] > 0 && s_snap[2].miss[i] == 0) ?
               "<-- AMCS fixed this!" : "");
    }

    printf("\r\n WORST-CASE RESPONSE TIME (ms):\r\n");
    printf(" %-14s | %8s | %8s | %8s | %8s\r\n",
           "Task", "RMS", "DMS", "AMCS", "Deadline");
    printf(" %-14s | %8s | %8s | %8s | %8s\r\n",
           "--------------","--------","--------","--------","--------");
    for (uint8_t i = 0; i < N_TASKS; i++) {
        printf(" %-14s | %8lu | %8lu | %8lu | %8lu  %s\r\n",
               tcb[i].name,
               s_snap[0].max_resp[i],
               s_snap[1].max_resp[i],
               s_snap[2].max_resp[i],
               tcb[i].deadline_ms,
               (s_snap[0].max_resp[i] > tcb[i].deadline_ms &&
                s_snap[2].max_resp[i] <= tcb[i].deadline_ms) ?
               "<-- AMCS met deadline!" : "");
    }

    printf("==================================================\r\n\r\n");
}

static void comparison_task(void *arg)
{
    (void)arg;
    static const SchedAlgo_t seq[3]  = { SCHED_RMS, SCHED_DMS, SCHED_AMCS };

    osDelay(1000);

    while (1) {
        for (uint8_t k = 0; k < 3; k++) {
            sched_switch(seq[k]);
            reset_counters();
            osDelay(EVAL_DURATION_MS);
            capture_snapshot(k, seq[k]);
        }

        g_mute_prints = 1;
        osDelay(100);

        print_comparison_table();

        osDelay(5000);
        g_mute_prints = 0;
    }
}

/* ===========================================================================
 * CLOCK CONFIG
 * =========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ===========================================================================
 * UART INIT
 * =========================================================================== */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* ===========================================================================
 * GPIO INIT
 * =========================================================================== */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ===========================================================================
 * ERROR HANDLER
 * =========================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
