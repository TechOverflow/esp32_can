/**
 * CANipulator_test_loopback.ino
 *
 * Dual-transceiver loopback test for ESP32-C6 (two TWAI controllers).
 *
 * Purpose
 * ───────
 * Verifies that both on-chip CAN controllers AND both external transceivers
 * are working, and that they can communicate with each other over a shared
 * physical bus.
 *
 * Wiring
 * ──────
 * Both transceivers must be connected to the SAME physical CAN bus:
 *   Transceiver 0  CANH ── CANH  Transceiver 1
 *   Transceiver 0  CANL ── CANL  Transceiver 1
 *   Transceiver 0  GND  ── GND   Transceiver 1
 *
 * A 120 Ω termination resistor between CANH and CANL at each end of the bus
 * is recommended (two resistors total). Without termination, short test
 * jumpers may still work, but a real bus needs them.
 *
 * CANipulator pins:
 *   CAN0 RX = GPIO16, TX = GPIO17   → Transceiver 0
 *   CAN1 RX = GPIO18, TX = GPIO19   → Transceiver 1
 *
 * Expected serial output (alternating, ~1 second apart):
 *   CAN0 sending message ID 001, data 00 00 00 00
 *   CAN1 received message ID 001, data 00 00 00 00
 *   CAN1 sending message ID 002, data 00 00 00 00
 *   CAN0 received message ID 002, data 00 00 00 00
 *   CAN0 sending message ID 001, data 00 00 00 01
 *   CAN1 received message ID 001, data 00 00 00 01
 *   CAN1 sending message ID 002, data 01 00 00 00
 *   CAN0 received message ID 002, data 01 00 00 00
 *   CAN0 sending message ID 001, data 00 00 00 02
 *   CAN1 received message ID 001, data 00 00 00 02
 *   CAN1 sending message ID 002, data 02 00 00 00
 *   CAN0 received message ID 002, data 02 00 00 00
 *
 * If you see "sending" lines but never "received" lines, check transceiver
 * wiring, termination, and that both transceivers share a common ground.
 */

#include <Arduino.h>
#include <esp32_can.h>

// ── Pin assignments (adjust to your board) ────────────────────────────────────
#define CAN0_RX_PIN   GPIO_NUM_16
#define CAN0_TX_PIN   GPIO_NUM_17
#define CAN1_RX_PIN   GPIO_NUM_18
#define CAN1_TX_PIN   GPIO_NUM_19
#define CAN0_SHUTDOWN 3
#define CAN0_STANDBY  0
#define CAN1_SHUTDOWN 2
#define CAN1_STANDBY  1

#define CAN_SPEED     500000    // 500 kbps

// ── Test message timing ───────────────────────────────────────────────────────
#define SEND_INTERVAL 1000      // ms between sends (slow, for easy reading)

// ── Blocking receipt confirmation ──────────────────────────────────────────────
// When set to 1, each send blocks until the partner bus receives the exact
// frame (matching ID + data). If RECEIPT_TIMEOUT_MS elapses with no match,
// a failure message is printed and the test continues.
#define WAIT_FOR_RECEIPT    1      // 1 = enabled, 0 = disabled
#define RECEIPT_TIMEOUT_MS  3000   // ms to wait for confirmation

// ── State ─────────────────────────────────────────────────────────────────────
unsigned long lastSendTime = 0;
bool          sendFromCan0  = true;   // alternate which bus initiates
uint8_t       counter       = 0;      // shared incrementing test value

// Print a frame in the requested format
void printFrame(const char *prefix, int busNum, const char *verb, CAN_FRAME &f)
{
    Serial.printf("%s%d %s message ID %03X, data ", prefix, busNum, verb, f.id);
    for (int i = 0; i < f.length; i++) {
        Serial.printf("%02X ", f.data.byte[i]);
    }
    Serial.println();
}

// Drain and print anything waiting on a given bus
void checkReceive(CAN_COMMON &bus, int busNum)
{
    CAN_FRAME rx;
    while (bus.available() && bus.read(rx)) {
        printFrame("CAN", busNum, "received", rx);
    }
}

// Block until `bus` receives a frame matching the expected ID and data,
// or until RECEIPT_TIMEOUT_MS elapses. Returns true if the frame arrived.
bool waitForReceipt(CAN_COMMON &bus, int busNum, CAN_FRAME &expected)
{
    unsigned long start = millis();

    while (millis() - start < RECEIPT_TIMEOUT_MS) {
        CAN_FRAME rx;
        if (bus.available() && bus.read(rx)) {
            // Check it matches what we sent
            bool match = (rx.id == expected.id) && (rx.length == expected.length);
            for (int i = 0; match && i < rx.length; i++) {
                if (rx.data.byte[i] != expected.data.byte[i]) match = false;
            }
            if (match) {
                printFrame("CAN", busNum, "received", rx);
                return true;
            }
            // Not a match — print it anyway so stray frames are visible
            printFrame("CAN", busNum, "received (unexpected)", rx);
        }
        delay(1);   // yield so the RX task and watchdog can run
    }

    Serial.printf("CAN%d NO RECEIPT within %d ms for ID %03X — retrying\n",
                  busNum, RECEIPT_TIMEOUT_MS, expected.id);
    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);
    Serial.println();
    Serial.println("=== ESP32-C6 Dual Transceiver Loopback Test ===");

    // setup CAN0 pins
    pinMode(CAN0_SHUTDOWN, OUTPUT);
    pinMode(CAN0_STANDBY, OUTPUT);
    digitalWrite(CAN0_SHUTDOWN, LOW);
    digitalWrite(CAN0_STANDBY, LOW);

    // setup CAN1 pins
    pinMode(CAN1_SHUTDOWN, OUTPUT);
    pinMode(CAN1_STANDBY, OUTPUT);
    digitalWrite(CAN1_SHUTDOWN, LOW);
    digitalWrite(CAN1_STANDBY, LOW);

    // ── CAN0 ──────────────────────────────────────────────────────────────
    CAN0.setCANPins(CAN0_RX_PIN, CAN0_TX_PIN);
    CAN0.begin(CAN_SPEED);
    CAN0.watchFor();   // accept all IDs

    // ── CAN1 ──────────────────────────────────────────────────────────────
    CAN1.setCANPins(CAN1_RX_PIN, CAN1_TX_PIN);
    CAN1.begin(CAN_SPEED);
    CAN1.watchFor();

    Serial.printf("Both controllers started at %d bps.\n", CAN_SPEED);
    Serial.printf("CAN0: RX=GPIO%d TX=GPIO%d   CAN1: RX=GPIO%d TX=GPIO%d\n",
              CAN0_RX_PIN, CAN0_TX_PIN, CAN1_RX_PIN, CAN1_TX_PIN);
    Serial.println("Waiting for traffic...\n");
}

void loop()
{
    // Always check both buses for incoming frames
    checkReceive(CAN0, 0);
    checkReceive(CAN1, 1);

    // Send a test frame on a timer, alternating direction each time
    unsigned long now = millis();
    if (now - lastSendTime >= SEND_INTERVAL) {
        lastSendTime = now;

        CAN_FRAME tx;
        tx.extended = false;
        tx.rtr      = 0;
        tx.length   = 4;
        tx.data.byte[0] = 0x00;
        tx.data.byte[1] = 0x00;
        tx.data.byte[2] = 0x00;
        tx.data.byte[3] = 0x00;

        if (sendFromCan0) {
            // CAN0: counter in the LAST byte (increments from the right)
            tx.id           = 0x001;
            tx.data.byte[3] = counter;
            printFrame("CAN", 0, "sending", tx);
            CAN0.sendFrame(tx);
            #if WAIT_FOR_RECEIPT
                waitForReceipt(CAN1, 1, tx);   // block until CAN1 sees it
            #endif
        } else {
            // CAN1: counter in the FIRST byte (increments from the left)
            tx.id           = 0x002;
            tx.data.byte[0] = counter;
            printFrame("CAN", 1, "sending", tx);
            CAN1.sendFrame(tx);
            #if WAIT_FOR_RECEIPT
                waitForReceipt(CAN0, 0, tx);   // block until CAN0 sees it
            #endif
            counter++;
        }

        sendFromCan0 = !sendFromCan0;  // swap direction for next send
    }
}
