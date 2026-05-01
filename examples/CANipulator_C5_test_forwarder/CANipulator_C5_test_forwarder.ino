/**
 * CANipulator_C5_test_forwarder.ino
 *
 * Two-channel CAN-FD forwarder for the CANipulator V2 (ESP32-C5).
 *
 * Listens on both CAN0 and CAN1 and forwards every received frame to the
 * other bus. Supports both classic CAN frames (≤ 8 bytes) and CAN-FD frames
 * (up to 64 bytes) transparently.
 *
 * Board:    CANipulator V2 or ESP32-C5 Dev Board (SOC_TWAI_SUPPORT_FD, two TWAI controllers)
 * Library:  esp32_can  (IDF 5.5+ tier, TWAI_TIER_NEW)
 *           can_common (collin80)
 *
 * Pin assignment:
 *   CAN0  RX = GPIO4,  TX = GPIO5
 *   CAN1  RX = GPIO6,  TX = GPIO7
 *
 * Bit rates used in this example:
 *   Arbitration phase : 500 kbps  (classic CAN compatible)
 *   Data phase (FD)   : 2 Mbps
 *
 * Note: The ESP32-C5 TWAI controller supports CAN-FD. Classic CAN nodes on
 * the same bus will still work at 500 kbps; they will not participate in the
 * FD data phase but will correctly acknowledge FD frames they see.
 *
 * No SmartLed dependency – add your own LED or remove those lines if needed.
 */

#include <Arduino.h>
#include <esp32_can.h>

// ── Bit rates ─────────────────────────────────────────────────────────────────
#define ARBITRATION_BAUD  500000UL   // 500 kbps nominal / arbitration phase
#define DATA_BAUD        2000000UL   // 2 Mbps FD data phase

// ── Forward declarations ──────────────────────────────────────────────────────
void printFrameClassic(CAN_FRAME    *msg, int busNum);
void printFrameFD     (CAN_FRAME_FD *msg, int busNum);
bool forwardClassic(CAN_COMMON &srcBus, CAN_COMMON &dstBus, int srcNum);
bool forwardFD     (CAN_COMMON &srcBus, CAN_COMMON &dstBus, int srcNum);

// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("[CANipulator-C5] CAN-FD forwarder starting");

    // ── CAN0 ──────────────────────────────────────────────────────────────
    // Set the FD data-phase baud rate BEFORE calling begin/init.
    // begin() internally calls init() which calls enable(), which builds the
    // twai_onchip_node_config_t; _dataBaudrate must be set first.
    CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);   // RX, TX
    CAN0.setDataBaudrate(DATA_BAUD);
    CAN0.begin(ARBITRATION_BAUD);
    CAN0.watchFor();   // accept all IDs

    // ── CAN1 ──────────────────────────────────────────────────────────────
    CAN1.setCANPins(GPIO_NUM_6, GPIO_NUM_7);   // RX, TX
    CAN1.setDataBaudrate(DATA_BAUD);
    CAN1.begin(ARBITRATION_BAUD);
    CAN1.watchFor();

    // Uncomment for verbose driver output:
    // CAN0.debuggingMode = true;
    // CAN1.debuggingMode = true;

    Serial.printf(
        "[CANipulator-C5] Ready. Arb: %lu bps  Data: %lu bps\n",
        ARBITRATION_BAUD, DATA_BAUD
    );
}

// Track last frame timestamps for each bus
unsigned long lastCan0Frame = 0;
unsigned long lastCan1Frame = 0;

void loop()
{
    // ── CAN0 → CAN1 ───────────────────────────────────────────────────────
    // Try FD frames first; fall back to classic frames.
    if (forwardFD(CAN0, CAN1, 0))      lastCan0Frame = millis();
    else if (forwardClassic(CAN0, CAN1, 0)) lastCan0Frame = millis();

    // ── CAN1 → CAN0 ───────────────────────────────────────────────────────
    if (forwardFD(CAN1, CAN0, 1))      lastCan1Frame = millis();
    else if (forwardClassic(CAN1, CAN0, 1)) lastCan1Frame = millis();

    // ── Optional: idle / activity print every 5 s ─────────────────────────
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 5000) {
        lastStatusPrint = millis();
        Serial.printf(
            "[CANipulator-C5] Last CAN0 frame: %lu ms ago  |  "
            "Last CAN1 frame: %lu ms ago\n",
            millis() - lastCan0Frame,
            millis() - lastCan1Frame
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Forwarding helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Check srcBus for a pending CAN-FD frame; if one is available, print it
 * and forward it to dstBus. Returns true if a frame was forwarded.
 */
bool forwardFD(CAN_COMMON &srcBus, CAN_COMMON &dstBus, int srcNum)
{
#if defined(SOC_TWAI_SUPPORT_FD)
    CAN_FRAME_FD frameFD;
    if (srcBus.readFD(frameFD)) {
        // Only process frames that are actually FD format
        if (frameFD.fdMode) {
            printFrameFD(&frameFD, srcNum);
            dstBus.sendFrameFD(frameFD);
            Serial.printf("  → forwarded FD to CAN%d\n", srcNum == 0 ? 1 : 0);
            return true;
        }
        // Was a classic frame retrieved via the FD path – put it back via
        // the classic path by re-packaging it.
        CAN_FRAME classic;
        classic.id       = frameFD.id;
        classic.extended = frameFD.extended;
        classic.rtr      = 0;
        classic.length   = frameFD.length > 8 ? 8 : frameFD.length;
        for (int i = 0; i < classic.length; i++)
            classic.data.byte[i] = frameFD.data.uint8[i];
        printFrameClassic(&classic, srcNum);
        dstBus.sendFrame(classic);
        Serial.printf("  → forwarded classic (via FD path) to CAN%d\n",
                      srcNum == 0 ? 1 : 0);
        return true;
    }
#endif
    return false;
}

/**
 * Check srcBus for a pending classic CAN frame (≤ 8 bytes); if one is
 * available, print it and forward it to dstBus.
 * Returns true if a frame was forwarded.
 */
bool forwardClassic(CAN_COMMON &srcBus, CAN_COMMON &dstBus, int srcNum)
{
    CAN_FRAME frame;
    if (srcBus.available() && srcBus.read(frame)) {
        printFrameClassic(&frame, srcNum);
        dstBus.sendFrame(frame);
        Serial.printf("  → forwarded classic to CAN%d\n", srcNum == 0 ? 1 : 0);
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print helpers
// ─────────────────────────────────────────────────────────────────────────────

void printFrameClassic(CAN_FRAME *msg, int busNum)
{
    Serial.printf(
        "CAN%d [classic]  ID: %08X  Ext: %d  RTR: %d  DLC: %d  "
        "Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
        busNum,
        msg->id, msg->extended, msg->rtr, msg->length,
        msg->data.byte[0], msg->data.byte[1],
        msg->data.byte[2], msg->data.byte[3],
        msg->data.byte[4], msg->data.byte[5],
        msg->data.byte[6], msg->data.byte[7]
    );
}

void printFrameFD(CAN_FRAME_FD *msg, int busNum)
{
    Serial.printf(
        "CAN%d [FD]  ID: %08X  Ext: %d  DLC: %d  Data:",
        busNum, msg->id, msg->extended, msg->length
    );
    for (int i = 0; i < msg->length; i++) {
        Serial.printf(" %02X", msg->data.uint8[i]);
    }
    Serial.println();
}
