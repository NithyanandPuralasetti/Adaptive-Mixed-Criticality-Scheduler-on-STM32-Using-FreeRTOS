Adaptive-Mixed-Criticality-Scheduler-on-STM32-Using-FreeRTOS

"""""README""""

IOC Specifications of the project for FreeRTOS :::

System core: GPIO-->USART2(PA2,3)
             NVIC-->As it is
             RCC-->HSE(BYPASS CLK Source)-->PLLCLK(72MHZ) in clock configuration
             Sys-->Serial wire , Tim2
Connnectivity: USART2--->115200,Asynchronous

Middle Ware & Software Packs: FreeRTOS
            API-->CMSIS_V2
            USE_PREEMPTION-->ENABLED
            TICK RATE-->1000

AFTER GIVING THE SPECIFICATIONS IN THE CUBEIDE THEN WE GENERATE CODE AND NEXT WE GO TO PRINTING THE OUTPUT IN TERMINAL

CREATE THE PYTHON FILE TO PRINT THE OUTPUT AND RUN THAT PYTHON SCRIPT AFTER RUNNING THE MAIN CODE THEN WE WILL SEE THE OUTPUT 



## 📊 System Evaluation Results (110% CPU Overload)

The system was stressed to a 110% utilization state for 15 seconds. The tables below demonstrate how standard static schedulers (RMS/DMS) suffer catastrophic priority inversion, while our **Advanced Mixed-Criticality Scheduler (AMCS)** actively protects hard-critical tasks.

### 🛑 Deadline Misses
Notice how the AMCS dynamically sacrifices the soft-critical `T7_Display` task to rescue the hard-critical communication and telemetry tasks.

| Task | Criticality | RMS | DMS | AMCS | Result |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **T1_MotorPWM** | Hard | 0 | 0 | 0 | - |
| **T2_Sensor** | Hard | 0 | 0 | 0 | - |
| **T3_CtrlLoop** | Hard | 0 | 0 | 0 | - |
| **T4_Safety** | Hard | 0 | 0 | 0 | - |
| **T5_Comm** | Hard | 4 | 4 | **0** | ✅ *AMCS Fixed* |
| **T6_DataLog** | Soft | 0 | 0 | 0 | - |
| **T7_Display** | Soft | 0 | 0 | **4** | ⚠️ *Controlled Sacrifice* |
| **T8_Telemetry** | Soft | 2 | 2 | **0** | ✅ *AMCS Fixed* |
| **T9_Diagnose** | Soft | 2 | 2 | **0** | ✅ *AMCS Fixed* |
| **T10_Battery** | Soft | 1 | 0 | **0** | ✅ *AMCS Fixed* |

### ⏱️ Worst-Case Response Time (WCRT)
Under static scheduling, `T5_Comm` experiences massive preemption interference. Under AMCS, its response time is successfully pulled back under the 800ms deadline.

| Task | Deadline (ms) | RMS (ms) | DMS (ms) | AMCS (ms) | AMCS Status |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **T1_MotorPWM** | 80 | 10 | 12 | 12 | Safe |
| **T2_Sensor** | 150 | 21 | 22 | 23 | Safe |
| **T3_CtrlLoop** | 300 | 29 | 32 | 29 | Safe |
| **T4_Safety** | 400 | 103 | 103 | 104 | Safe |
| **T5_Comm** | **800** | ❌ 1164 | ❌ 1163 | **✅ 614** | **Met Deadline** |
| **T6_DataLog** | 600 | 255 | 255 | 444 | Safe |
| **T7_Display** | **700** | 454 | 452 | **⚠️ 752** | *Sacrificed* |
| **T8_Telemetry**| **1000** | ❌ 2118 | ❌ 2115 | **✅ 733** | **Met Deadline** |
| **T9_Diagnose** | **1200** | ❌ 2900 | ❌ 2897 | **✅ 315** | **Met Deadline** |
| **T10_Battery** | **1800** | ❌ 2422 | 1471 | **✅ 1235** | **Met Deadline** |
