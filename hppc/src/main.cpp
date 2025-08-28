#include <Arduino.h>
#include <SD.h>

// GPIO configuration
#define STATUS_LED 13
#define NEO_PIN 8
#define CURRENT_ON 5
#define CS 4
#define V_IN A0
#define A_OUT A1

// Application parameter
// Voltage measurement
const float lowerBatVoltage = 10.8; // 1.8V * 6 Cells
const float referenceVoltage = 3.3; // Arduino reference voltage
const int adcResolution = 4096;     // 12-bit ADC
const float voltageDividerRatio = 5.0; // Adjust if using voltage divider

// Ampere measurement
const float ampereRatio = 384.6;
const float ampere = 15;
const int ampereADC = 4095; // 4095
// 1024 = 1.76
// 2048 = 3.55
// 4095 = 7.09

// Timing parameters (in milliseconds)
unsigned long currentOnDuration = 10UL * 60UL * 1000UL;    // 10 minutes in ms
unsigned long currentOffDuration = 50UL * 60UL * 1000UL;  // 50 minutes in ms
unsigned long measurementPeriod = 100;  // 0.1 s
unsigned long startupDelay = 10000;      // 10 seconds startup delay

// State variables
unsigned long currentSwitchTime = 0;
unsigned long programStartTime = 0;
unsigned long lastMeasurementTime = 0;
int currentState = -1; // -1: startup, HIGH: current on, LOW: current off

bool statusLEDState = LOW;
bool sdCardError = false;

File dataFile;

// Convert ADC reading to voltage
float adcToVoltage(int adcValue) {
  return (adcValue * referenceVoltage * voltageDividerRatio) / adcResolution;
}


float getAverageVoltage(int samples = 3) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(V_IN);
    delay(1);
  }
  return adcToVoltage(sum / samples);
}


void updateCurrentState(unsigned long currentTime) {
  int prevState = currentState;

  unsigned long timeSinceSwitch = currentTime - currentSwitchTime;

  switch (currentState)
  {
  case -1: // on startup
    currentState = HIGH;
    break;
  case HIGH: 
    if (timeSinceSwitch >= currentOnDuration) {
      currentState = LOW;
    }
    break;

  case LOW: 
    if (timeSinceSwitch >= currentOffDuration) {
      currentState = HIGH;
    }
    break;
  }

  if (prevState != currentState) {
      currentSwitchTime = currentTime;
      digitalWrite(CURRENT_ON, currentState);
      digitalWrite(STATUS_LED, currentState);
  }
}


void toggleStatusLED() {
  statusLEDState = !statusLEDState;
  digitalWrite(STATUS_LED, statusLEDState);
}


void writeDataToSD(unsigned long timestamp, float voltage, int state) {
  if (!dataFile || sdCardError) return;
  
  dataFile.print(timestamp);
  dataFile.print(",");
  dataFile.print(voltage, 4);
  dataFile.print(",");
  dataFile.print(state);
  int bytesWritten = dataFile.println(";");
  dataFile.flush();

  if (bytesWritten <= 0) {
    sdCardError = true;
  }
}



void handleLowBattery(float voltage) {
  if (dataFile && !sdCardError) {
    unsigned long currentTime = millis();
    dataFile.print(currentTime);
    dataFile.print(",SHUTDOWN,LOW_BATTERY,");
    dataFile.print(voltage, 4);
    dataFile.print(",");
    dataFile.println(currentTime - programStartTime);
    dataFile.flush();
    dataFile.close();
  }
  
  // Turn off all outputs
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(CURRENT_ON, LOW);
  
  Serial.println("Low Battery Voltage detected. Programm end.");
  while(1) {
    toggleStatusLED();
    delay(500);
  }
}

void setup() {
    pinMode(STATUS_LED, OUTPUT);
    pinMode(CURRENT_ON, OUTPUT);
    pinMode(A_OUT, OUTPUT);
    pinMode(V_IN, INPUT);
    analogReadResolution(12);
    analogWriteResolution(12);

    digitalWrite(CURRENT_ON, LOW);
    digitalWrite(STATUS_LED, LOW);

    Serial.begin(9600);
    while (!Serial && millis() < 5000);


    analogWrite(A_OUT, ampereADC);

    if (!SD.begin(CS)) {
        Serial.println("SD Card Initialization failed.");
        sdCardError = true;
    } else {
        Serial.println("SD card initialized successfully");
        dataFile = SD.open( "batch2.csv", FILE_WRITE);
        if (dataFile) {
            // Write comprehensive header
            dataFile.println("# Battery Discharge Characterization Data - Run 1");
            dataFile.print("# Started: ");
            dataFile.println(millis());
            dataFile.print("# Discharge duration: ");
            dataFile.print(currentOnDuration / 1000);
            dataFile.println(" seconds");
            dataFile.print("# Rest duration: ");
            dataFile.print(currentOffDuration / 1000);
            dataFile.println(" seconds");
            dataFile.print("# Low voltage cutoff: ");
            dataFile.print(lowerBatVoltage, 3);
            dataFile.println(" V");
            dataFile.println("# Format:\nTimestamp(ms),Voltage(V),Raw_ADC,Current_State,Cycle_Time(ms)");
            dataFile.flush();
            Serial.println("Data logging started");
        } else {
            Serial.println("ERROR: Could not create data file");
            sdCardError = true;
        }
    }
  programStartTime = millis();
  lastMeasurementTime = programStartTime;
  
  Serial.println("Setup complete.");
  Serial.print("Low battery cutoff: ");
  Serial.print(lowerBatVoltage, 3);
  Serial.println("V");

  if (sdCardError) {
      Serial.println("SD card error! Wait for restart...");
      while (1) {
          delay(1000);
      }
  }
}

void loop() {

  unsigned long currentTime = millis();
  
  // Handle overflow 
  if (currentTime < programStartTime) {
    programStartTime = currentTime;
    currentSwitchTime = currentTime;
  }
  
  // Time-based measurement
  if (currentTime - lastMeasurementTime >= measurementPeriod) {
    
    float batteryVoltage = getAverageVoltage(5);
    
    Serial.print("Time: ");
    Serial.print(currentTime / 1000);
    Serial.print("s, Voltage: ");
    Serial.print(batteryVoltage, 3);
    Serial.print("V, State: ");
    Serial.println(currentState == HIGH ? "ON" : "OFF");
    
    writeDataToSD(currentTime, batteryVoltage, currentState);
    
    if (batteryVoltage < lowerBatVoltage || sdCardError) {
      handleLowBattery(batteryVoltage);
    }
    
    lastMeasurementTime = currentTime;

    if (currentTime - programStartTime >= startupDelay) {
        updateCurrentState(currentTime);
    }
  }
  delay(10);
}