#include <Arduino.h>
#include <ModbusRtu.h>

const int windPin = 2;
const int ledPin = 13;           // Built-in LED for Teensy
const int rs485DirPin = 10;      // DE+RE control pin for RS485 transceiver

volatile unsigned long pulseCount = 0;
float windSpeed = 0;

// Modbus holding registers
uint16_t modbusRegs[2];          // Only one register used for wind speed

Modbus slave(1, Serial1, rs485DirPin); // Slave ID 1

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // RS485 at high baud rate
  pinMode(windPin, INPUT_PULLDOWN);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(windPin), countPulse, RISING);

  slave.start();             // Initialize Modbus slave
  slave.setTimeOut(2000);    // Optional timeout
}

void loop() {
  pulseCount = 0;
  delay(1000); // 1-second measurement window

  // Calculate wind speed from pulse count
  if (pulseCount == 0) {
    windSpeed = 0;
  } else {
    windSpeed = 0.0664 * pulseCount + 0.5019;
  }

  // Debug output
  Serial.print("Pulses per second: ");
  Serial.println(pulseCount);
  Serial.print("Wind speed (m/s): ");
  Serial.println(windSpeed);

  // Store wind speed as int * 100 in Modbus register
  modbusRegs[0] = (uint16_t)(windSpeed * 100.0);

  // Check if any Modbus request is handled
  if (slave.poll(modbusRegs, 1)) {
    // Blink LED to indicate Modbus activity
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
  }
}
