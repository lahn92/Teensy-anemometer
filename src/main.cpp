#include <Arduino.h>
#include <ModbusRTU.h>

const int windPin = 22;
volatile unsigned long pulseCount = 0;

ModbusRTU mb;

void countPulse() {
  pulseCount++;
}

const int sampleInterval = 100; // ms
const int sampleCount = 10;     // number of samples for rolling average
unsigned long pulseSamples[sampleCount] = {0};
int currentSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(windPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(windPin), countPulse, RISING);

  Serial1.begin(115200);
  mb.begin(&Serial1, 3);
  mb.slave(1);
  mb.addHreg(0, 0, 10);
}

void loop() {
  static unsigned long lastSampleTime = 0;

  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime += sampleInterval;

    // Read and reset pulse count
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // Store current count in rolling buffer
    pulseSamples[currentSample] = count;
    currentSample = (currentSample + 1) % sampleCount;

    // Compute rolling sum and average
    unsigned long totalPulses = 0;
    for (int i = 0; i < sampleCount; i++) {
      totalPulses += pulseSamples[i];
    }

    float pulsesPerSecond = totalPulses * (1000.0 / (sampleInterval * sampleCount));
    float windSpeed = 0;
    if (pulsesPerSecond > 0) {
      windSpeed = 0.0664 * pulsesPerSecond + 0.5019;
    }

    uint16_t windScaled = (uint16_t)(windSpeed * 100);
    mb.Hreg(0, windScaled);

    Serial.print("Wind speed (m/s): ");
    Serial.println(windSpeed, 2);
  }

  mb.task();
}
