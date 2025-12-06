# Concurrency Diagram - LidarSafe Technologies

## System Architecture Overview

This diagram shows the complete task/ISR structure with synchronization primitives.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    LidarSafe Real-Time System Architecture                  │
│                         ESP32 + FreeRTOS Implementation                     │
└─────────────────────────────────────────────────────────────────────────────┘

════════════════════════════════════════════════════════════════════════════════
                               HARDWARE LAYER
════════════════════════════════════════════════════════════════════════════════

    ┌───────────────┐     ┌───────────────┐     ┌───────────────────────────┐
    │   HC-SR04     │     │   EMERGENCY   │     │         LEDs              │
    │  Ultrasonic   │     │    BUTTON     │     │  RED    YELLOW   GREEN    │
    │               │     │               │     │ GPIO2   GPIO15   GPIO13   │
    │ TRIG: GPIO5   │     │    GPIO4      │     │  (R)     (Y)      (G)     │
    │ ECHO: GPIO18  │     │ INPUT_PULLUP  │     │                           │
    └───────┬───────┘     └───────┬───────┘     └─────────────▲─────────────┘
            │                     │                           │
            │ Analog              │ Interrupt                 │ Digital
            │ Distance            │ (FALLING)                 │ Output
            │                     │                           │
════════════│═════════════════════│═══════════════════════════│════════════════
            │         INTERRUPT SERVICE ROUTINE               │
════════════│═════════════════════│═══════════════════════════│════════════════
            │                     │                           │
            │                     ▼                           │
            │          ┌─────────────────────┐                │
            │          │   Emergency_ISR     │                │
            │          │   ━━━━━━━━━━━━━━━   │                │
            │          │   Trigger: FALLING  │                │
            │          │   Priority: HIGHEST │                │
            │          │   Deadline: <1ms    │                │
            │          │   Type: ███ HARD ███│                │
            │          └──────────┬──────────┘                │
            │                     │                           │
            │                     │ xSemaphoreGiveFromISR()   │
            │                     ▼                           │
════════════│═════════════════════════════════════════════════│════════════════
            │         SYNCHRONIZATION PRIMITIVES              │
════════════│═════════════════════════════════════════════════│════════════════
            │                     │                           │
            │    ┌────────────────┼────────────────┐          │
            │    │                │                │          │
            │    ▼                ▼                ▼          │
            │  ┌─────────┐  ┌──────────┐  ┌─────────────┐     │
            │  │ MUTEX   │  │ BINARY   │  │   QUEUE     │     │
            │  │         │  │SEMAPHORE │  │             │     │
            │  │xSensor  │  │xEmergency│  │xProcessed   │     │
            │  │DataMutex│  │Semaphore │  │DataQueue    │     │
            │  │         │  │          │  │ (Size: 5)   │     │
            │  └────┬────┘  └────┬─────┘  └──────┬──────┘     │
            │       │            │               │            │
════════════│═══════│════════════│═══════════════│════════════│════════════════
            │       │    FREERTOS TASKS          │            │
════════════│═══════│════════════│═══════════════│════════════│════════════════
            │       │            │               │            │
            ▼       ▼            │               │            │
   ┌────────────────────┐        │               │            │
   │    SensorTask      │        │               │            │
   │    ━━━━━━━━━━━━    │        │               │            │
   │  Period:  50ms     │        │               │            │
   │  Priority: 5       │        │               │            │
   │  Deadline: 50ms    │        │               │            │
   │  Type: ███ HARD ███│        │               │            │
   │                    │        │               │            │
   │  [measureDistance] │        │               │            │
   │  VARIABLE TIME     │        │               │            │
   │  (depends on dist) │        │               │            │
   └─────────┬──────────┘        │               │            │
             │                   │               │            │
             │ xSemaphoreTake    │               │            │
             │ (write data)      │               │            │
             │ xSemaphoreGive    │               │            │
             ▼                   │               │            │
   ┌─────────────────────────────┴───────────────┐            │
   │              ProcessingTask                 │            │
   │              ━━━━━━━━━━━━━━━                 │            │
   │  Period:  100ms                             │            │
   │  Priority: 4                                │            │
   │  Deadline: 100ms                            │            │
   │  Type: ███ HARD ███                         │            │
   │                                             │            │
   │  [classifyObstacle]                         │            │
   │  VARIABLE TIME: 1-10ms based on threat      │            │
   │                                             │            │
   │  ◄── xSemaphoreTake (read data via mutex)   │            │
   │  ◄── xSemaphoreTake (check emergency sem)   │            │
   │  ──► xQueueSend (processed data)            │            │
   └──────────────────────┬──────────────────────┘            │
                          │                                   │
                          │ ProcessedData_t                   │
                          │ {distance, level, timestamp,      │
                          │  emergency_active}                │
                          ▼                                   │
             ┌────────────┴────────────┐                      │
             │                         │                      │
             ▼                         ▼                      │
   ┌──────────────────┐     ┌──────────────────┐              │
   │   WarningTask    │     │  TelemetryTask   │              │
   │   ━━━━━━━━━━━    │     │  ━━━━━━━━━━━━━   │              │
   │ Period: 200ms    │     │ Period: 500ms    │              │
   │ Priority: 3      │     │ Priority: 2      │              │
   │ Deadline: 200ms  │     │ Deadline: 500ms  │              │
   │ Type: ░░ SOFT ░░ │     │ Type: ░░ SOFT ░░ │              │
   │                  │     │                  │              │
   │ xQueueReceive    │     │ xQueuePeek       │              │
   │ (consumes data)  │     │ (reads, keeps)   │              │
   └────────┬─────────┘     └────────┬─────────┘              │
            │                        │                        │
            │                        │ Serial.printf()        │
            │                        ▼                        │
            │               ┌──────────────────┐              │
            │               │  UART TX (GPIO1) │              │
            │               │  Serial Monitor  │              │
            │               │  @ 115200 baud   │              │
            │               │                  │              │
            │               │  EXTERNAL COMM   │              │
            │               └──────────────────┘              │
            │                                                 │
            └─────────────────────────────────────────────────┘
                          LED Control (R, Y)
                                   │
   ┌───────────────────────────────┘
   │
   │     ┌──────────────────┐
   │     │  HeartbeatTask   │
   │     │  ━━━━━━━━━━━━━   │
   │     │ Period: 1000ms   │
   │     │ Priority: 1      │
   │     │ Deadline: 1000ms │
   │     │ Type: ░░ SOFT ░░ │
   │     │                  │
   │     │ digitalWrite(G)  │
   │     └────────┬─────────┘
   │              │
   │              ▼
   │     Green LED (GPIO13)
   │
   └──► Red LED (GPIO2) + Yellow LED (GPIO15)


════════════════════════════════════════════════════════════════════════════════
                                  LEGEND
════════════════════════════════════════════════════════════════════════════════

    ███ HARD ███  = Hard real-time deadline (failure = system failure)
    ░░░ SOFT ░░░  = Soft real-time deadline (failure = degraded performance)

    ────────►     = Data flow direction
    ◄──────►      = Bidirectional access (via mutex)

    ┌─────────┐
    │  TASK   │   = FreeRTOS Task
    └─────────┘

    ┌─────────┐
    │  SYNC   │   = Synchronization Primitive (Mutex/Semaphore/Queue)
    └─────────┘


════════════════════════════════════════════════════════════════════════════════
                           TIMING SUMMARY
════════════════════════════════════════════════════════════════════════════════

    Task            Period    WCET     Utilization    Deadline Type
    ──────────────  ────────  ───────  ─────────────  ─────────────
    Emergency_ISR   Event     <1ms     N/A            HARD
    SensorTask      50ms      ~8ms     16%            HARD
    ProcessingTask  100ms     ~12ms    12%            HARD
    WarningTask     200ms     ~2ms     1%             SOFT
    TelemetryTask   500ms     ~10ms    2%             SOFT
    HeartbeatTask   1000ms    ~1ms     0.1%           SOFT
    ──────────────────────────────────────────────────────────────
    TOTAL CPU UTILIZATION:             ~31%

    RMS Utilization Bound (5 tasks): 5 * (2^(1/5) - 1) ≈ 74%

    Schedulability: ✅ GUARANTEED (31% << 74%)


════════════════════════════════════════════════════════════════════════════════
                        DATA STRUCTURE FLOW
════════════════════════════════════════════════════════════════════════════════

    SensorData_t (Protected by Mutex):
    ┌─────────────────────────────────┐
    │  float distance_cm              │  ◄── Written by SensorTask
    │  uint32_t timestamp_ms          │  ◄── Read by ProcessingTask
    │  bool valid                     │
    └─────────────────────────────────┘

    ProcessedData_t (Passed via Queue):
    ┌─────────────────────────────────┐
    │  float distance_cm              │
    │  ObstacleLevel_t level          │  ──► WarningTask (LED control)
    │  uint32_t timestamp_ms          │  ──► TelemetryTask (logging)
    │  bool emergency_active          │
    └─────────────────────────────────┘

    ObstacleLevel_t:
    ┌─────────────────────────────────┐
    │  OBSTACLE_NONE      (>200cm)    │  ──► LEDs OFF
    │  OBSTACLE_FAR       (100-200cm) │  ──► Yellow BLINK
    │  OBSTACLE_PROXIMITY (30-100cm)  │  ──► Yellow ON
    │  OBSTACLE_COLLISION (<30cm)     │  ──► Red + Yellow ON
    └─────────────────────────────────┘

```

## Quick Reference

| Component      | Type | Period | Priority | H/S  | GPIO |
| -------------- | ---- | ------ | -------- | ---- | ---- |
| Emergency_ISR  | ISR  | Event  | Highest  | HARD | 4    |
| SensorTask     | Task | 50ms   | 5        | HARD | 5,18 |
| ProcessingTask | Task | 100ms  | 4        | HARD | -    |
| WarningTask    | Task | 200ms  | 3        | SOFT | 2,15 |
| TelemetryTask  | Task | 500ms  | 2        | SOFT | TX   |
| HeartbeatTask  | Task | 1000ms | 1        | SOFT | 13   |
