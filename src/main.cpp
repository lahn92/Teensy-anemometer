#include <Arduino.h>
#include <ModbusRTU.h>

const int windPin = 22;
volatile unsigned long pulseCount = 0;

ModbusRTU mb;

void countPulse()
{
  pulseCount++;
}

const int sampleInterval = 200; // ms
const int sampleCount = 10;     // number of samples for rolling average
unsigned long pulseSamples[sampleCount] = {0};
int currentSample = 0;

// Callback when a holding register is read
bool cbGet(uint16_t address, uint16_t &value)
{
  Serial.print("Modbus read request for Hreg address: ");
  Serial.println(address);
  return true; // Continue normal processing
}

void setup()
{
  Serial.begin(9600);
  pinMode(windPin, INPUT_PULLDOWN);
  pinMode(12, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(windPin), countPulse, RISING);

  Serial1.begin(9600);
  mb.begin(&Serial1, 12);
  mb.slave(99);

  // Add holding registers and set read callback
  mb.addHreg(0, 0, 10);
  mb.onGetHreg(0, cbGet);
}

void loop()
{
  mb.task();
  static unsigned long lastSampleTime = 0;

  if (millis() - lastSampleTime >= sampleInterval)
  {
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
    for (int i = 0; i < sampleCount; i++)
    {
      totalPulses += pulseSamples[i];
    }

    float pulsesPerSecond = totalPulses * (1000.0 / (sampleInterval * sampleCount));
    float windSpeed = 0;
    if (pulsesPerSecond > 0)
    {
      windSpeed = 0.0664 * pulsesPerSecond + 0.5019;
    }

    uint16_t windScaled = (uint16_t)(windSpeed * 100);
    mb.Hreg(0, windScaled);

    Serial.print("Wind speed (m/s): ");
    Serial.println(windSpeed, 2);
  }
}
