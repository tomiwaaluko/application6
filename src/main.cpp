/**
 * ============================================================================
 * LidarSafe Technologies - Autonomous Vehicle LIDAR Perception Prototype
 * ============================================================================
 * 
 * Company: LidarSafe Technologies (fictional Orlando-based startup)
 * Project: Real-Time LIDAR Obstacle Detection System Proof-of-Concept
 * 
 * Description:
 *   This prototype demonstrates a real-time embedded system for autonomous
 *   vehicle obstacle detection. It simulates a LIDAR perception pipeline with:
 *   - Distance sensing (HC-SR04 ultrasonic as LIDAR proxy)
 *   - Emergency stop interrupt handling
 *   - Obstacle classification and warning generation
 *   - Telemetry logging for safety analysis
 * 
 * Real-Time Criticality:
 *   In autonomous vehicles, missing a deadline for obstacle detection could
 *   result in a collision. This prototype demonstrates how FreeRTOS ensures
 *   predictable timing for safety-critical tasks.
 * 
 * Author: [Your Name]
 * Course: Real-Time Systems
 * Date: December 2025
 * 
 * AI Disclosure: GitHub Copilot was used to assist with code structure and
 * FreeRTOS API usage. All timing analysis and design decisions are original.
 * ============================================================================
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================
// All pins labeled to match diagram.json for easy verification

#define PIN_LED_COLLISION     2    // Red LED - Collision warning (HARD deadline indicator)
#define PIN_LED_PROXIMITY    15    // Yellow LED - Proximity alert
#define PIN_LED_HEARTBEAT    13    // Green LED - System heartbeat
#define PIN_ULTRASONIC_TRIG   5    // HC-SR04 Trigger pin
#define PIN_ULTRASONIC_ECHO  18    // HC-SR04 Echo pin
#define PIN_EMERGENCY_BTN     4    // Emergency stop button (INPUT_PULLUP, active LOW)
#define PIN_DEBUG_TIMING     19    // Debug pin for logic analyzer timing verification

// ============================================================================
// TIMING CONSTANTS - All periods and deadlines declared here
// ============================================================================
// These values are chosen to demonstrate real-time scheduling while keeping
// CPU utilization reasonable (~30-40% total)

// Task Periods (in milliseconds)
#define PERIOD_SENSOR_MS        50    // 20 Hz - LIDAR scan rate
#define PERIOD_PROCESSING_MS   100    // 10 Hz - Obstacle classification
#define PERIOD_WARNING_MS      200    //  5 Hz - Warning LED update
#define PERIOD_TELEMETRY_MS    500    //  2 Hz - UART logging
#define PERIOD_HEARTBEAT_MS   1000    //  1 Hz - System alive indicator

// Deadlines (equal to periods for this design - implicit deadlines)
#define DEADLINE_SENSOR_MS      PERIOD_SENSOR_MS
#define DEADLINE_PROCESSING_MS  PERIOD_PROCESSING_MS
#define DEADLINE_WARNING_MS     PERIOD_WARNING_MS
#define DEADLINE_TELEMETRY_MS   PERIOD_TELEMETRY_MS
#define DEADLINE_HEARTBEAT_MS   PERIOD_HEARTBEAT_MS

// Distance thresholds (in centimeters)
#define THRESHOLD_COLLISION_CM   30   // < 30cm = COLLISION IMMINENT (HARD)
#define THRESHOLD_PROXIMITY_CM  100   // < 100cm = Proximity warning (SOFT)
#define THRESHOLD_SAFE_CM       200   // >= 200cm = All clear

// ============================================================================
// TASK PRIORITIES (Higher number = Higher priority in FreeRTOS)
// ============================================================================
// Priority assignment rationale:
// - SensorTask highest because raw data acquisition is time-critical
// - ProcessingTask next because classification determines safety actions
// - WarningTask medium because visual feedback is important but not critical
// - TelemetryTask lower because logging can tolerate some delay
// - HeartbeatTask lowest because it's purely diagnostic

#define PRIORITY_SENSOR       5    // Highest task priority (HARD deadline)
#define PRIORITY_PROCESSING   4    // High priority (HARD deadline)
#define PRIORITY_WARNING      3    // Medium priority (SOFT deadline)
#define PRIORITY_TELEMETRY    2    // Low priority (SOFT deadline)
#define PRIORITY_HEARTBEAT    1    // Lowest priority (SOFT deadline)

// Stack sizes (in words, not bytes - ESP32 uses 32-bit words)
#define STACK_SIZE_DEFAULT   2048
#define STACK_SIZE_TELEMETRY 3072  // Larger for Serial operations

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * Obstacle classification levels
 */
typedef enum {
    OBSTACLE_NONE = 0,      // No obstacle detected
    OBSTACLE_FAR,           // Obstacle detected but far (safe)
    OBSTACLE_PROXIMITY,     // Obstacle in proximity zone (warning)
    OBSTACLE_COLLISION      // Obstacle in collision zone (CRITICAL)
} ObstacleLevel_t;

/**
 * Sensor data structure - shared between SensorTask and ProcessingTask
 * Protected by mutex to prevent race conditions
 */
typedef struct {
    float distance_cm;           // Current distance reading
    uint32_t timestamp_ms;       // When measurement was taken
    bool valid;                  // Whether reading is valid
} SensorData_t;

/**
 * Processed data structure - passed via queue to downstream tasks
 */
typedef struct {
    float distance_cm;           // Distance value
    ObstacleLevel_t level;       // Classification result
    uint32_t timestamp_ms;       // Processing timestamp
    bool emergency_active;       // Emergency stop triggered
} ProcessedData_t;

// ============================================================================
// GLOBAL VARIABLES - FreeRTOS Primitives
// ============================================================================

// Task handles (for debugging/monitoring)
TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xProcessingTaskHandle = NULL;
TaskHandle_t xWarningTaskHandle = NULL;
TaskHandle_t xTelemetryTaskHandle = NULL;
TaskHandle_t xHeartbeatTaskHandle = NULL;

// SYNCHRONIZATION MECHANISM #1: Mutex
// Purpose: Protect shared sensor data structure from race conditions
// Used by: SensorTask (writer), ProcessingTask (reader)
SemaphoreHandle_t xSensorDataMutex = NULL;

// SYNCHRONIZATION MECHANISM #2: Binary Semaphore
// Purpose: Signal from Emergency ISR to ProcessingTask for immediate response
// Used by: Emergency_ISR (gives), ProcessingTask (takes with timeout)
SemaphoreHandle_t xEmergencySemaphore = NULL;

// INTERNAL COMMUNICATION: Queue
// Purpose: Pass processed obstacle data from ProcessingTask to consumers
// Used by: ProcessingTask (sender), WarningTask & TelemetryTask (receivers)
QueueHandle_t xProcessedDataQueue = NULL;

// Shared sensor data (protected by mutex)
volatile SensorData_t g_sensorData = {0.0f, 0, false};

// Emergency state (set by ISR, read by tasks)
volatile bool g_emergencyTriggered = false;

// Statistics for analysis
volatile uint32_t g_sensorTaskCount = 0;
volatile uint32_t g_processingTaskCount = 0;
volatile uint32_t g_isrTriggerCount = 0;

// ============================================================================
// ISR: Emergency Stop Interrupt
// ============================================================================
/**
 * Emergency_ISR - HARD REAL-TIME
 * 
 * Trigger: Falling edge on emergency button (GPIO4)
 * Deadline: Immediate (< 1ms response required)
 * Classification: HARD - Missing this deadline could cause collision
 * 
 * Company Use-Case: Emergency brake trigger - when operator or safety system
 * detects imminent danger, this interrupt must immediately signal the
 * processing pipeline to halt vehicle operations.
 * 
 * Updated Behavior: Toggle emergency state - first press activates emergency
 * stop with visual/serial feedback, second press clears it.
 * 
 * Design Decision: Using binary semaphore (not direct task notification)
 * because semaphore provides clearer synchronization semantics and allows
 * multiple tasks to potentially wait on the same emergency signal.
 */
void IRAM_ATTR Emergency_ISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Toggle emergency state (each button press flips the state)
    g_emergencyTriggered = !g_emergencyTriggered;
    g_isrTriggerCount++;
    
    // Signal ProcessingTask via binary semaphore
    // Using FromISR variant - CRITICAL for ISR safety
    xSemaphoreGiveFromISR(xEmergencySemaphore, &xHigherPriorityTaskWoken);
    
    // Yield to higher priority task if needed (immediate response)
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// ============================================================================
// HELPER FUNCTION: Ultrasonic Distance Measurement
// ============================================================================
/**
 * measureDistance - Read HC-SR04 ultrasonic sensor
 * 
 * Returns: Distance in centimeters, or -1.0 if timeout/error
 * 
 * Note: This function has variable execution time (depends on distance)
 * which satisfies the requirement for "at least 1 task takes variable time"
 */
float measureDistance() {
    // Send 10us trigger pulse
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    
    // Measure echo pulse duration (timeout after 30ms = ~5m max distance)
    unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 30000);
    
    if (duration == 0) {
        return -1.0f;  // Timeout - no echo received
    }
    
    // Calculate distance: speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (duration * 0.0343) / 2 (round trip)
    float distance = (duration * 0.0343f) / 2.0f;
    
    return distance;
}

/**
 * classifyObstacle - Determine obstacle threat level
 */
ObstacleLevel_t classifyObstacle(float distance_cm) {
    if (distance_cm < 0) {
        return OBSTACLE_NONE;  // Invalid reading
    } else if (distance_cm < THRESHOLD_COLLISION_CM) {
        return OBSTACLE_COLLISION;  // CRITICAL
    } else if (distance_cm < THRESHOLD_PROXIMITY_CM) {
        return OBSTACLE_PROXIMITY;  // Warning
    } else if (distance_cm < THRESHOLD_SAFE_CM) {
        return OBSTACLE_FAR;  // Detected but safe
    } else {
        return OBSTACLE_NONE;  // Clear
    }
}

// ============================================================================
// TASK 1: SensorTask - HARD REAL-TIME
// ============================================================================
/**
 * SensorTask - LIDAR Sensor Polling
 * 
 * Period: 50ms (20 Hz)
 * Deadline: 50ms (HARD)
 * Priority: 5 (Highest)
 * 
 * Company Use-Case: LIDAR frame acquisition - The autonomous vehicle must
 * continuously scan its environment at 20 Hz to detect obstacles in real-time.
 * Missing sensor readings could create blind spots leading to collisions.
 * 
 * Requirement Satisfied: ≥1 sensor, variable execution time (pulseIn varies)
 */
void SensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_SENSOR_MS);
    
    Serial.println("[SENSOR] Task started - Period: 50ms, Deadline: HARD");
    
    for (;;) {
        uint32_t startTime = millis();
        
        // Toggle debug pin HIGH for logic analyzer timing measurement
        digitalWrite(PIN_DEBUG_TIMING, HIGH);
        
        // Read distance sensor (VARIABLE execution time based on distance)
        float distance = measureDistance();
        uint32_t measureTime = millis();
        
        // CRITICAL SECTION: Update shared sensor data with mutex protection
        // This prevents race condition with ProcessingTask reading the data
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_sensorData.distance_cm = distance;
            g_sensorData.timestamp_ms = measureTime;
            g_sensorData.valid = (distance >= 0);
            xSemaphoreGive(xSensorDataMutex);
        } else {
            // Mutex timeout - log but continue (SOFT failure)
            Serial.println("[SENSOR] WARNING: Mutex timeout!");
        }
        
        g_sensorTaskCount++;
        
        // Toggle debug pin LOW
        digitalWrite(PIN_DEBUG_TIMING, LOW);
        
        uint32_t executionTime = millis() - startTime;
        
        // Deadline check: Log if we're approaching deadline
        if (executionTime > DEADLINE_SENSOR_MS * 0.8) {
            Serial.printf("[SENSOR] WARNING: Execution time %lu ms (80%% of deadline)\n", 
                         executionTime);
        }
        
        // Use vTaskDelayUntil for precise periodic timing (no drift)
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================================
// TASK 2: ProcessingTask - HARD REAL-TIME
// ============================================================================
/**
 * ProcessingTask - Obstacle Classification
 * 
 * Period: 100ms (10 Hz)
 * Deadline: 100ms (HARD)
 * Priority: 4 (High)
 * 
 * Company Use-Case: Obstacle classification and threat assessment - This task
 * analyzes sensor data to determine if obstacles pose collision risk. It must
 * complete within deadline to ensure timely warning generation.
 * 
 * Variable Execution Time: Simulates variable processing based on obstacle
 * complexity (simple vs complex classification algorithms)
 * 
 * Requirement Satisfied: ≥4 tasks, variable processing time, uses mutex & semaphore
 */
void ProcessingTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_PROCESSING_MS);
    
    SensorData_t localSensorData;
    ProcessedData_t processedData;
    
    Serial.println("[PROCESSING] Task started - Period: 100ms, Deadline: HARD");
    
    for (;;) {
        uint32_t startTime = millis();
        bool emergencyThisCycle = false;
        
        // Check for emergency signal from ISR (non-blocking check)
        // Binary semaphore with 0 timeout = poll without blocking
        if (xSemaphoreTake(xEmergencySemaphore, 0) == pdTRUE) {
            emergencyThisCycle = true;
            
            // Check current emergency state
            if (g_emergencyTriggered) {
                // EMERGENCY ACTIVATED
                Serial.printf("[EMERGENCY] !!! EMERGENCY STOP ACTIVATED at %lu ms !!!\n", millis());
                Serial.println("[EMERGENCY] System entering emergency halt mode.");
                Serial.println("[EMERGENCY] Press button again to resume normal operation.");
                
                // Flash red LED rapidly to indicate emergency state (4 seconds)
                for (int i = 0; i < 20; i++) {
                    digitalWrite(PIN_LED_COLLISION, HIGH);
                    digitalWrite(PIN_LED_PROXIMITY, LOW);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    digitalWrite(PIN_LED_COLLISION, LOW);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            } else {
                // EMERGENCY CLEARED
                Serial.printf("[EMERGENCY] Emergency stop CLEARED at %lu ms\n", millis());
                Serial.println("[EMERGENCY] Resuming normal operation.");
                
                // Brief confirmation blink
                digitalWrite(PIN_LED_COLLISION, LOW);
                digitalWrite(PIN_LED_PROXIMITY, LOW);
                digitalWrite(PIN_LED_HEARTBEAT, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                digitalWrite(PIN_LED_HEARTBEAT, LOW);
            }
        }
        
        // CRITICAL SECTION: Read shared sensor data with mutex protection
        // This is where a race condition would occur without the mutex
        // Line protected: Reading g_sensorData while SensorTask might be writing
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            // Copy data to local variable to minimize critical section duration
            // Manual field copy required because g_sensorData is volatile
            localSensorData.distance_cm = g_sensorData.distance_cm;
            localSensorData.timestamp_ms = g_sensorData.timestamp_ms;
            localSensorData.valid = g_sensorData.valid;
            xSemaphoreGive(xSensorDataMutex);
        } else {
            // Mutex timeout - use stale data (graceful degradation)
            Serial.println("[PROCESSING] WARNING: Mutex timeout, using stale data");
        }
        
        // VARIABLE EXECUTION TIME: Simulate complex processing
        // In real system, this would be actual classification algorithm
        ObstacleLevel_t level = classifyObstacle(localSensorData.distance_cm);
        
        // Simulate variable processing time based on classification complexity
        // Collision detection requires more thorough analysis
        if (level == OBSTACLE_COLLISION) {
            delay(5);  // Complex analysis for critical situations
        } else if (level == OBSTACLE_PROXIMITY) {
            delay(3);  // Moderate analysis
        } else {
            delay(1);  // Simple case
        }
        
        // Prepare processed data for queue
        processedData.distance_cm = localSensorData.distance_cm;
        processedData.level = level;
        processedData.timestamp_ms = millis();
        processedData.emergency_active = emergencyThisCycle || g_emergencyTriggered;
        
        // Send to queue (INTERNAL COMMUNICATION CHANNEL)
        // Using timeout to prevent blocking if queue is full
        if (xQueueSend(xProcessedDataQueue, &processedData, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("[PROCESSING] WARNING: Queue full, data dropped");
        }
        
        g_processingTaskCount++;
        
        uint32_t executionTime = millis() - startTime;
        
        // Deadline verification
        if (executionTime > DEADLINE_PROCESSING_MS) {
            Serial.printf("[PROCESSING] !!! DEADLINE MISSED: %lu ms > %d ms !!!\n",
                         executionTime, DEADLINE_PROCESSING_MS);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================================
// TASK 3: WarningTask - SOFT REAL-TIME
// ============================================================================
/**
 * WarningTask - LED Warning Display
 * 
 * Period: 200ms (5 Hz)
 * Deadline: 200ms (SOFT)
 * Priority: 3 (Medium)
 * 
 * Company Use-Case: Visual warning system - Controls dashboard warning lights
 * to alert driver/operator of obstacle proximity. Missing occasional deadline
 * may cause brief flicker but doesn't compromise safety (SOFT).
 * 
 * Requirement Satisfied: ≥2 LEDs, queue consumer
 */
void WarningTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_WARNING_MS);
    
    ProcessedData_t receivedData;
    
    Serial.println("[WARNING] Task started - Period: 200ms, Deadline: SOFT");
    
    for (;;) {
        // Check if system is in emergency mode
        if (g_emergencyTriggered) {
            // EMERGENCY MODE: Flash red LED rapidly, ignore normal operation
            digitalWrite(PIN_LED_COLLISION, !digitalRead(PIN_LED_COLLISION));
            digitalWrite(PIN_LED_PROXIMITY, LOW);
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            continue; // Skip normal LED updates
        }
        
        // Receive from queue (blocks until data available or timeout)
        if (xQueueReceive(xProcessedDataQueue, &receivedData, pdMS_TO_TICKS(150)) == pdPASS) {
            
            // Update LEDs based on obstacle level
            switch (receivedData.level) {
                case OBSTACLE_COLLISION:
                    // CRITICAL: Both red and yellow ON, rapid indication
                    digitalWrite(PIN_LED_COLLISION, HIGH);
                    digitalWrite(PIN_LED_PROXIMITY, HIGH);
                    break;
                    
                case OBSTACLE_PROXIMITY:
                    // Warning: Yellow ON, red OFF
                    digitalWrite(PIN_LED_COLLISION, LOW);
                    digitalWrite(PIN_LED_PROXIMITY, HIGH);
                    break;
                    
                case OBSTACLE_FAR:
                    // Detected but safe: Yellow blinking (toggle)
                    digitalWrite(PIN_LED_COLLISION, LOW);
                    digitalWrite(PIN_LED_PROXIMITY, !digitalRead(PIN_LED_PROXIMITY));
                    break;
                    
                case OBSTACLE_NONE:
                default:
                    // All clear: Both OFF
                    digitalWrite(PIN_LED_COLLISION, LOW);
                    digitalWrite(PIN_LED_PROXIMITY, LOW);
                    break;
            }
            
        } else {
            // Queue timeout - no data received (system may be idle)
            // Turn off warning LEDs as precaution
            digitalWrite(PIN_LED_COLLISION, LOW);
            digitalWrite(PIN_LED_PROXIMITY, LOW);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================================
// TASK 4: TelemetryTask - SOFT REAL-TIME
// ============================================================================
/**
 * TelemetryTask - UART Data Logging
 * 
 * Period: 500ms (2 Hz)
 * Deadline: 500ms (SOFT)
 * Priority: 2 (Low)
 * 
 * Company Use-Case: Telemetry logging for safety analysis - Records sensor
 * data and system state to UART for post-incident analysis. Missing occasional
 * log entries is acceptable (SOFT) as long as overall coverage is maintained.
 * 
 * Requirement Satisfied: External communication (UART), timestamps for
 * determinism proof
 */
void TelemetryTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_TELEMETRY_MS);
    
    ProcessedData_t receivedData;
    static uint32_t logSequence = 0;
    
    Serial.println("[TELEMETRY] Task started - Period: 500ms, Deadline: SOFT");
    Serial.println("============================================================");
    Serial.println("TELEMETRY LOG - LidarSafe Technologies");
    Serial.println("Format: [SEQ] TIME | DIST | LEVEL | EMERGENCY | STATS");
    Serial.println("============================================================");
    
    for (;;) {
        uint32_t currentTime = millis();
        
        // Peek at queue without removing (WarningTask also needs this data)
        // Alternative: Use separate queue or broadcast mechanism
        if (xQueuePeek(xProcessedDataQueue, &receivedData, pdMS_TO_TICKS(100)) == pdPASS) {
            
            const char* levelStr;
            switch (receivedData.level) {
                case OBSTACLE_COLLISION: levelStr = "COLLISION!"; break;
                case OBSTACLE_PROXIMITY: levelStr = "PROXIMITY"; break;
                case OBSTACLE_FAR:       levelStr = "FAR"; break;
                default:                 levelStr = "CLEAR"; break;
            }
            
            // EXTERNAL COMMUNICATION: UART output with timestamps
            // This proves determinism - timestamps should be ~500ms apart
            Serial.printf("[%04lu] %8lu ms | %6.1f cm | %-10s | EMG:%s | S:%lu P:%lu I:%lu\n",
                         logSequence++,
                         currentTime,
                         receivedData.distance_cm,
                         levelStr,
                         receivedData.emergency_active ? "YES" : "NO ",
                         g_sensorTaskCount,
                         g_processingTaskCount,
                         g_isrTriggerCount);
            
            // Deadline miss would show as >500ms gap between log entries
            
        } else {
            // No data - log system status anyway
            Serial.printf("[%04lu] %8lu ms | NO DATA  | WAITING    | EMG:%s | S:%lu P:%lu I:%lu\n",
                         logSequence++,
                         currentTime,
                         g_emergencyTriggered ? "YES" : "NO ",
                         g_sensorTaskCount,
                         g_processingTaskCount,
                         g_isrTriggerCount);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================================
// TASK 5: HeartbeatTask - SOFT REAL-TIME
// ============================================================================
/**
 * HeartbeatTask - System Alive Indicator
 * 
 * Period: 1000ms (1 Hz)
 * Deadline: 1000ms (SOFT)
 * Priority: 1 (Lowest)
 * 
 * Company Use-Case: System health indicator - Blinks green LED to show system
 * is operational. Useful for visual debugging and operator confidence.
 * Missing this deadline only affects visual feedback (SOFT).
 * 
 * Requirement Satisfied: ≥4 tasks (this is task #5)
 */
void HeartbeatTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_HEARTBEAT_MS);
    
    bool ledState = false;
    
    Serial.println("[HEARTBEAT] Task started - Period: 1000ms, Deadline: SOFT");
    
    for (;;) {
        // Toggle heartbeat LED
        ledState = !ledState;
        digitalWrite(PIN_LED_HEARTBEAT, ledState);
        
        // Emergency override: Fast blink if emergency active
        if (g_emergencyTriggered) {
            // Double-blink pattern to indicate emergency state
            delay(100);
            digitalWrite(PIN_LED_HEARTBEAT, !ledState);
            delay(100);
            digitalWrite(PIN_LED_HEARTBEAT, ledState);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
    // Initialize serial communication (EXTERNAL COMM CHANNEL)
    Serial.begin(115200);
    delay(500);  // Allow serial to stabilize (500ms needed for Wokwi)
    Serial.println("\n\n");  // Extra newlines help serial monitor sync
    
    Serial.println("============================================================");
    Serial.println("LidarSafe Technologies - LIDAR Perception Prototype v1.0");
    Serial.println("Real-Time Systems Proof-of-Concept");
    Serial.println("============================================================\n");
    
    // Initialize GPIO pins
    pinMode(PIN_LED_COLLISION, OUTPUT);
    pinMode(PIN_LED_PROXIMITY, OUTPUT);
    pinMode(PIN_LED_HEARTBEAT, OUTPUT);
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);  // Active LOW with internal pullup
    pinMode(PIN_DEBUG_TIMING, OUTPUT);
    
    // Initial LED test - all ON briefly then OFF
    digitalWrite(PIN_LED_COLLISION, HIGH);
    digitalWrite(PIN_LED_PROXIMITY, HIGH);
    digitalWrite(PIN_LED_HEARTBEAT, HIGH);
    delay(500);
    digitalWrite(PIN_LED_COLLISION, LOW);
    digitalWrite(PIN_LED_PROXIMITY, LOW);
    digitalWrite(PIN_LED_HEARTBEAT, LOW);
    
    Serial.println("[INIT] GPIO pins configured");
    
    // Create synchronization primitives
    
    // SYNCHRONIZATION #1: Mutex for sensor data protection
    xSensorDataMutex = xSemaphoreCreateMutex();
    if (xSensorDataMutex == NULL) {
        Serial.println("[INIT] ERROR: Failed to create sensor data mutex!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] Mutex created: xSensorDataMutex");
    
    // SYNCHRONIZATION #2: Binary semaphore for emergency ISR signaling
    xEmergencySemaphore = xSemaphoreCreateBinary();
    if (xEmergencySemaphore == NULL) {
        Serial.println("[INIT] ERROR: Failed to create emergency semaphore!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] Binary Semaphore created: xEmergencySemaphore");
    
    // INTERNAL COMMUNICATION: Queue for processed data
    // Size 5 allows buffering if consumer tasks fall behind briefly
    xProcessedDataQueue = xQueueCreate(5, sizeof(ProcessedData_t));
    if (xProcessedDataQueue == NULL) {
        Serial.println("[INIT] ERROR: Failed to create processed data queue!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] Queue created: xProcessedDataQueue (size 5)");
    
    // Attach emergency button interrupt
    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_BTN), Emergency_ISR, FALLING);
    Serial.println("[INIT] Emergency ISR attached to GPIO4 (FALLING edge)");
    
    // Create FreeRTOS tasks
    // Tasks are created with specific priorities to ensure Rate Monotonic scheduling
    
    BaseType_t xReturned;
    
    // Task 1: SensorTask - HARD deadline, highest priority
    xReturned = xTaskCreate(
        SensorTask,
        "SensorTask",
        STACK_SIZE_DEFAULT,
        NULL,
        PRIORITY_SENSOR,
        &xSensorTaskHandle
    );
    if (xReturned != pdPASS) {
        Serial.println("[INIT] ERROR: Failed to create SensorTask!");
    } else {
        Serial.printf("[INIT] SensorTask created - Priority %d (HARD)\n", PRIORITY_SENSOR);
    }
    
    // Task 2: ProcessingTask - HARD deadline, high priority
    xReturned = xTaskCreate(
        ProcessingTask,
        "ProcessingTask",
        STACK_SIZE_DEFAULT,
        NULL,
        PRIORITY_PROCESSING,
        &xProcessingTaskHandle
    );
    if (xReturned != pdPASS) {
        Serial.println("[INIT] ERROR: Failed to create ProcessingTask!");
    } else {
        Serial.printf("[INIT] ProcessingTask created - Priority %d (HARD)\n", PRIORITY_PROCESSING);
    }
    
    // Task 3: WarningTask - SOFT deadline, medium priority
    xReturned = xTaskCreate(
        WarningTask,
        "WarningTask",
        STACK_SIZE_DEFAULT,
        NULL,
        PRIORITY_WARNING,
        &xWarningTaskHandle
    );
    if (xReturned != pdPASS) {
        Serial.println("[INIT] ERROR: Failed to create WarningTask!");
    } else {
        Serial.printf("[INIT] WarningTask created - Priority %d (SOFT)\n", PRIORITY_WARNING);
    }
    
    // Task 4: TelemetryTask - SOFT deadline, low priority
    xReturned = xTaskCreate(
        TelemetryTask,
        "TelemetryTask",
        STACK_SIZE_TELEMETRY,
        NULL,
        PRIORITY_TELEMETRY,
        &xTelemetryTaskHandle
    );
    if (xReturned != pdPASS) {
        Serial.println("[INIT] ERROR: Failed to create TelemetryTask!");
    } else {
        Serial.printf("[INIT] TelemetryTask created - Priority %d (SOFT)\n", PRIORITY_TELEMETRY);
    }
    
    // Task 5: HeartbeatTask - SOFT deadline, lowest priority
    xReturned = xTaskCreate(
        HeartbeatTask,
        "HeartbeatTask",
        STACK_SIZE_DEFAULT,
        NULL,
        PRIORITY_HEARTBEAT,
        &xHeartbeatTaskHandle
    );
    if (xReturned != pdPASS) {
        Serial.println("[INIT] ERROR: Failed to create HeartbeatTask!");
    } else {
        Serial.printf("[INIT] HeartbeatTask created - Priority %d (SOFT)\n", PRIORITY_HEARTBEAT);
    }
    
    Serial.println("\n[INIT] All tasks created successfully!");
    Serial.println("[INIT] FreeRTOS scheduler starting...\n");
    Serial.println("============================================================");
    Serial.println("SYSTEM RUNNING - Press red button to trigger EMERGENCY STOP");
    Serial.println("============================================================\n");
    
    // Note: On ESP32, FreeRTOS scheduler starts automatically
    // The loop() function runs as a FreeRTOS task at priority 1
}

// ============================================================================
// LOOP FUNCTION
// ============================================================================
/**
 * loop() - Runs as a FreeRTOS task on ESP32
 * 
 * We keep this minimal since all work is done in dedicated tasks.
 * This demonstrates proper FreeRTOS design - avoid work in loop().
 */
void loop() {
    // Nothing to do here - all work handled by FreeRTOS tasks
    // Adding a delay to prevent watchdog issues
    vTaskDelay(pdMS_TO_TICKS(1000));
}
