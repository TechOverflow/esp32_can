#include <Arduino.h>
#include <SmartLeds.h>
#include <esp32_can.h>

const int LED_COUNT = 1;
const int DATA_PIN = 8;
const int CHANNEL = 0;

SmartLed leds(LED_WS2812B, LED_COUNT, DATA_PIN, CHANNEL, DoubleBuffer);

#define CAN0_SHUTDOWN 3
#define CAN0_STANDBY 0

#define CAN1_SHUTDOWN 2
#define CAN1_STANDBY 1

void setup() {
  Serial.begin(115200);

  // setup status LED
  leds[0] = Rgb { 255, 0, 0 };
  leds.show();
  leds.wait();

  // setup CAN0 pins
  pinMode(CAN0_SHUTDOWN, OUTPUT);
  pinMode(CAN0_STANDBY, OUTPUT);

  // enable CAN0 transceiver
  digitalWrite(CAN0_SHUTDOWN, LOW);
  digitalWrite(CAN0_STANDBY, LOW);

  // setup CAN1 pins
  pinMode(CAN1_SHUTDOWN, OUTPUT);
  pinMode(CAN1_STANDBY, OUTPUT);

  // enable CAN1 transceiver
  digitalWrite(CAN1_SHUTDOWN, LOW);
  digitalWrite(CAN1_STANDBY, LOW);

  // setup CAN pins and speed
  CAN0.setCANPins(GPIO_NUM_16, GPIO_NUM_17);
  CAN0.begin(500000);
  CAN0.watchFor();
  CAN1.setCANPins(GPIO_NUM_18, GPIO_NUM_19);
  CAN1.begin(500000);
  CAN1.watchFor();

  // CAN0.debuggingMode = true;
  // CAN1.debuggingMode = true;  
}

auto lastCan0Frame = 0;
auto lastCan1Frame = 0;

void loop() {
  // check for can messages on each bus
  CAN_FRAME frame0;
  
  if (CAN0.available() && CAN0.read(frame0)) {
    // print out bus and frame
    printFrame(&frame0, 0);
    CAN1.sendFrame(frame0);
    Serial.println("Frame sent from CAN0 to CAN1");
    lastCan0Frame = millis();
  }

  CAN_FRAME frame1;

  if (CAN1.available() && CAN1.read(frame1)) {
    // print out bus and frame
    printFrame(&frame1, 1);
    CAN0.sendFrame(frame1);
    Serial.println("Frame sent from CAN1 to CAN0");
    lastCan1Frame = millis();
  }

  // show green led when receiving CAN0 and blue led when receiving CAN1
  auto now = millis();
  bool showGreen = now - lastCan0Frame < 1000;
  bool showBlue = now - lastCan1Frame < 1000;

  leds[0] = Rgb { showGreen || showGreen ? 0 : 5, showGreen ? 5 : 0, showBlue ? 5 : 0 };
  leds.show();
  leds.wait();
}

void printFrame(CAN_FRAME *message, int busNum) {
  Serial.printf("CAN %d - ID: %08X, DLC: %d, Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
    busNum,
    message->id, 
    message->length, 
    message->data.byte[0], 
    message->data.byte[1], 
    message->data.byte[2], 
    message->data.byte[3], 
    message->data.byte[4], 
    message->data.byte[5], 
    message->data.byte[6], 
    message->data.byte[7]
  );
}