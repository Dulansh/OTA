#include <Wire.h>
#include <MPU6050.h>

// Configuration constants
const float VIBRATION_THRESHOLD = 0.35;    // Threshold for detecting active operation
const int READINGS_COUNT = 10;             // Number of samples for moving average
const long SAMPLE_INTERVAL_MS = 1000;      // Sampling interval in milliseconds
const float FILTER_ALPHA = 0.9;            // High-pass filter coefficient (0-1)
const unsigned long MINUTE_MS = 60000;     // Milliseconds in a minute

// MPU6050 sensor instance
MPU6050 mpu;

// Moving average variables
float readings[READINGS_COUNT] = {0};
int readIndex = 0;
float readingsTotal = 0;
float average = 0;

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long minuteTimer = 0;
int workingSeconds = 0;

// High-pass filter variables
float prevAccel[3] = {0, 0, 0};  // Previous acceleration values {x, y, z}
float filteredAccel[3] = {0, 0, 0};  // Filtered acceleration values

// Runtime statistics
unsigned long totalMinutes = 0;    // Track total running time

void setup() {
    Serial.begin(115200);
    initializeMPU();
    minuteTimer = millis();  // Initialize the minute timer
}

void loop() {
    unsigned long currentTime = millis();
    
    // Process sensor data
    processSensorData();

    // Check if sampling interval has elapsed (every second)
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        updateWorkingTime();  // Update working time based on vibration threshold
        lastSampleTime = currentTime;
    }

    // Check if a minute has elapsed
    if (currentTime - minuteTimer >= MINUTE_MS) {
        reportMinuteStats();  // Output stats for the current minute
        resetMinuteCounter(); // Reset counters for the next minute
    }
}

void initializeMPU() {
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);  // Halt if sensor not found
    }
    Serial.println("MPU6050 connected successfully.");
    Serial.println("Monitoring started - will report machine working time every minute.");
}

void processSensorData() {
    // Read raw acceleration data
    int16_t rawAccel[3];
    mpu.getAcceleration(&rawAccel[0], &rawAccel[1], &rawAccel[2]);
    
    float resultantMagnitude = 0;
    
    // Process each axis
    for (int i = 0; i < 3; i++) {
        // Normalize acceleration values
        float normalizedAccel = rawAccel[i] / 16384.0;
        
        // Apply high-pass filter
        filteredAccel[i] = FILTER_ALPHA * (filteredAccel[i] + normalizedAccel - prevAccel[i]);
        prevAccel[i] = normalizedAccel;
        
        // Add to magnitude calculation
        resultantMagnitude += filteredAccel[i] * filteredAccel[i];
    }
    
    resultantMagnitude = sqrt(resultantMagnitude);
    updateMovingAverage(resultantMagnitude);
}

void updateMovingAverage(float newValue) {
    readingsTotal -= readings[readIndex];
    readings[readIndex] = newValue;
    readingsTotal += newValue;

    readIndex = (readIndex + 1) % READINGS_COUNT;
    average = readingsTotal / READINGS_COUNT;
}

void updateWorkingTime() {
    if (average > VIBRATION_THRESHOLD) {
        workingSeconds++;
    }
}

void reportMinuteStats() {
    // Calculate percentage of working time
    float workingPercentage = (workingSeconds * 100.0) / 60.0;
    
    Serial.print("Minute ");
    Serial.print(totalMinutes + 1);  // Add 1 for human-readable counting
    Serial.print(" | Working time: ");
    Serial.print(workingSeconds);
    Serial.print(" seconds (");
    Serial.print(workingPercentage, 1);  // Display with 1 decimal place
    Serial.println("%)");
}

void resetMinuteCounter() {
    workingSeconds = 0;         // Reset working seconds counter
    minuteTimer = millis();     // Reset minute timer to current time
}
