ESP32 CAN library with legacy and FD support
==========

This is an Arduino library for the built-in CAN (TWAI) controller(s) on ESP32 family chips.
It provides a high-level, Arduino-friendly API that works across all ESP-IDF versions
from v4.x through v5.5+ and across all ESP32 variants — from the original single-controller
ESP32 to the three-controller ESP32-P4 and the CAN-FD-capable ESP32-C5.

Check out the [CANipulator](https://www.tindie.com/products/fusion/canipulator-automotive-dual-can-esp32-interface/) as an example of a device with dual-CAN transceivers.

<img src="https://img.tindie.com/images/resize/Ic5QZOyXJkvT2W2Ekt6Fbf1h4rM=/p/fit-in/1370x912/filters:fill(fff)/i/26064/products/2024-07-07T01:54:25.239Z-0001.jpg" width="400" />


---

## Relationship to collin80/esp32_can

This library is a fork of [collin80/esp32_can](https://github.com/collin80/esp32_can),
which is itself part of a family of CAN libraries sharing the
[collin80/can_common](https://github.com/collin80/can_common) abstract base class.
That base class defines the `CAN_COMMON` interface and the `CAN_FRAME` / `CAN_FRAME_FD`
frame types used throughout.

The key differences introduced in this fork:

**IDF 5.5+ support (TWAI_TIER_NEW).** The original library targets IDF ≤ 5.4 and uses
`twai_driver_install` / `twai_receive` (blocking). IDF 5.5 replaced the entire driver with
`esp_driver_twai` (`twai_new_node_onchip`, callback-based async RX). This fork adds a
complete third implementation tier for the new driver, selected automatically at compile time.

**CAN-FD support.** On chips where `SOC_TWAI_SUPPORT_FD` is defined (currently ESP32-C5),
`setDataBaudrate()` enables the FD data phase and `sendFrameFD()` / `get_rx_buffFD()`
handle frames up to 64 bytes. The FD path is compiled away entirely on non-FD chips.

**Three-tier build system.** Rather than a single `#if` guard, the library now selects
one of three complete implementations at compile time based on `ESP_IDF_VERSION`:

```
TWAI_TIER_V1  →  IDF < 5.2    legacy single-controller API
TWAI_TIER_V2  →  IDF 5.2–5.4  legacy _v2 handle-based API
TWAI_TIER_NEW →  IDF ≥ 5.5    new esp_driver_twai component
```

Existing sketches written for the original library compile and run unchanged.

---

## Supported hardware

| Chip | TWAI controllers | CAN-FD | `CAN0` | `CAN1` | `CAN2` |
|---|---|---|---|---|---|
| ESP32 (original) | 1 | No | ✅ | — | — |
| ESP32-S2 | 1 | No | ✅ | — | — |
| ESP32-S3 | 1 | No | ✅ | — | — |
| ESP32-C3 | 1 | No | ✅ | — | — |
| ESP32-C6 | 2 | No | ✅ | ✅ | — |
| ESP32-C5 | 2 | **Yes** | ✅ | ✅ | — |
| ESP32-P4 | 3 | No | ✅ | ✅ | ✅ |

> The library selects the correct number of `CAN` globals at compile time using
> `SOC_TWAI_CONTROLLER_NUM`, so no code changes are needed when switching targets.

---

## Dependencies

- [collin80/can_common](https://github.com/collin80/can_common) — provides `CAN_COMMON`,
  `CAN_FRAME`, and `CAN_FRAME_FD`. Install via the Arduino Library Manager or manually.

---

## Installation

1. Install **can_common** via Arduino Library Manager or clone into your `libraries/` folder.
2. Clone or download this repository into `Arduino/libraries/esp32_can/`.
3. Select your ESP32 board in the Arduino IDE — the correct IDF tier is picked automatically.

---

## Basic usage

### Single CAN controller (ESP32, ESP32-S2, ESP32-S3, ESP32-C3)

```cpp
#include <esp32_can.h>

void setup() {
    Serial.begin(115200);

    CAN0.setCANPins(GPIO_NUM_16, GPIO_NUM_17); // RX, TX — adjust to your board
    CAN0.begin(500000);   // 500 kbps
    CAN0.watchFor();      // accept all IDs
}

void loop() {
    CAN_FRAME frame;
    if (CAN0.available() && CAN0.read(frame)) {
        Serial.printf("ID: %08X  DLC: %d  Data: %02X %02X %02X %02X\n",
            frame.id, frame.length,
            frame.data.byte[0], frame.data.byte[1],
            frame.data.byte[2], frame.data.byte[3]);
    }

    // Transmit
    CAN_FRAME tx;
    tx.id       = 0x123;
    tx.extended = false;
    tx.length   = 4;
    tx.data.byte[0] = 0xDE;
    tx.data.byte[1] = 0xAD;
    tx.data.byte[2] = 0xBE;
    tx.data.byte[3] = 0xEF;
    CAN0.sendFrame(tx);
}
```

### Filtering specific IDs

```cpp
// Accept only standard IDs 0x100–0x1FF (mask = 0x700, id = 0x100)
CAN0.watchFor(0x100, 0x700);

// Accept a single extended ID
CAN0.watchForRange(0x18DA00F1, 0x18DA00F1);

// Per-filter callback — fires when a matching frame arrives
CAN0.setCallback(0, [](CAN_FRAME *frame) {
    Serial.printf("Filter 0 hit: ID %08X\n", frame->id);
});
```

### Listen-only mode (bus monitoring / sniffer)

```cpp
CAN0.setListenOnlyMode(true);
CAN0.begin(500000);
CAN0.watchFor();
```

### Auto-detect bus speed

```cpp
// Probes common speeds from 1 Mbps down to 20 kbps.
// Returns the detected speed, or 0 if none found.
uint32_t detected = CAN0.beginAutoSpeed();
Serial.printf("Detected: %lu bps\n", detected);
```

---

### Dual classic CAN (ESP32-C6, ESP32-P4)

Both controllers use the same API. `CAN1` is available automatically when
`SOC_TWAI_CONTROLLER_NUM >= 2`.

```cpp
#include <esp32_can.h>

void setup() {
    // CAN0 — controller 0
    CAN0.setCANPins(GPIO_NUM_0, GPIO_NUM_1);
    CAN0.begin(500000);
    CAN0.watchFor();

    // CAN1 — controller 1
    CAN1.setCANPins(GPIO_NUM_2, GPIO_NUM_3);
    CAN1.begin(500000);
    CAN1.watchFor();
}

void loop() {
    CAN_FRAME frame;

    // Forward CAN0 → CAN1
    if (CAN0.available() && CAN0.read(frame)) {
        CAN1.sendFrame(frame);
    }

    // Forward CAN1 → CAN0
    if (CAN1.available() && CAN1.read(frame)) {
        CAN0.sendFrame(frame);
    }
}
```

### Three-controller classic CAN (ESP32-P4)

`CAN2` is available when `SOC_TWAI_CONTROLLER_NUM >= 3`. Usage is identical to `CAN0`/`CAN1`.

```cpp
#include <esp32_can.h>

void setup() {
    CAN0.setCANPins(GPIO_NUM_0,  GPIO_NUM_1);
    CAN0.begin(500000);
    CAN0.watchFor();

    CAN1.setCANPins(GPIO_NUM_4,  GPIO_NUM_5);
    CAN1.begin(500000);
    CAN1.watchFor();

    CAN2.setCANPins(GPIO_NUM_8,  GPIO_NUM_9);
    CAN2.begin(500000);
    CAN2.watchFor();
}
```

> **Note:** The ESP32-P4 does not support CAN-FD. Its three TWAI controllers are
> classic CAN 2.0B only. FD frames on the bus will be interpreted as errors.
> Each controller requires its own external transceiver (e.g. TJA1051).

---

### CAN-FD (ESP32-C5)

CAN-FD is only available on chips where `SOC_TWAI_SUPPORT_FD` is defined.
The FD data-phase baud rate must be set **before** calling `begin()`.

```cpp
#include <esp32_can.h>

void setup() {
    // Set FD data phase BEFORE begin()
    CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);
    CAN0.setDataBaudrate(2000000); // 2 Mbps data phase
    CAN0.begin(500000);            // 500 kbps arbitration phase
    CAN0.watchFor();

    CAN1.setCANPins(GPIO_NUM_6, GPIO_NUM_7);
    CAN1.setDataBaudrate(2000000);
    CAN1.begin(500000);
    CAN1.watchFor();
}

void loop() {
    // Receive FD frame
    CAN_FRAME_FD rxFD;
    if (CAN0.readFD(rxFD) && rxFD.fdMode) {
        Serial.printf("FD frame ID: %08X  DLC: %d\n", rxFD.id, rxFD.length);
        for (int i = 0; i < rxFD.length; i++) {
            Serial.printf(" %02X", rxFD.data.uint8[i]);
        }
        Serial.println();
    }

    // Transmit FD frame
    CAN_FRAME_FD txFD;
    txFD.id       = 0x123;
    txFD.extended = false;
    txFD.fdMode   = 1;     // mark as FD frame
    txFD.length   = 16;    // up to 64 bytes
    for (int i = 0; i < 16; i++) txFD.data.uint8[i] = i;
    CAN0.sendFrameFD(txFD);

    // Classic frames still work normally on a CAN-FD bus
    CAN_FRAME classic;
    classic.id     = 0x456;
    classic.length = 4;
    classic.data.byte[0] = 0xAA;
    CAN0.sendFrame(classic);
}
```

> **Mixed-bus note:** Classic CAN nodes and CAN-FD nodes can coexist on the same bus.
> Classic nodes will correctly acknowledge FD frames during the arbitration phase but
> will not participate in the higher-speed data phase.

---

## API reference

### Initialisation

| Method | Description |
|---|---|
| `setCANPins(rx, tx)` | Set GPIO pins — call before `begin()` |
| `setDataBaudrate(bps)` | Enable CAN-FD data phase — call before `begin()` (FD chips only) |
| `begin(bps)` | Start the controller at the given bit rate |
| `beginAutoSpeed()` | Auto-detect bus speed; returns detected bps or 0 |
| `set_baudrate(bps)` | Change speed on a running controller (triggers restart) |
| `setListenOnlyMode(bool)` | Receive-only, no TX or ACK |
| `setNoACKMode(bool)` | Transmit without requiring acknowledgement (self-test) |
| `setRXBufferSize(n)` | Set RX queue depth before `begin()` |

### Filtering

| Method | Description |
|---|---|
| `watchFor()` | Accept all frames (clears filter table) |
| `watchFor(id, mask)` | Accept frames matching `(id & mask) == (rxId & mask)` |
| `watchForRange(low, high)` | Accept IDs in the given inclusive range |
| `setCallback(filter, fn)` | Register a callback for a specific filter slot |
| `setGeneralCallback(fn)` | Register a callback for all accepted frames |

### Receive

| Method | Description |
|---|---|
| `available()` | Number of frames waiting in the RX queue |
| `read(frame)` | Read the oldest frame from the queue into `frame` |
| `readFD(frame)` | Read an FD frame (FD chips only) |

### Transmit

| Method | Description |
|---|---|
| `sendFrame(frame)` | Transmit a classic CAN frame; returns `true` on success |
| `sendFrameFD(frame)` | Transmit a CAN-FD frame (FD chips only) |

---

## Troubleshooting

**Linker errors referencing `twai_*` functions** — you are likely mixing IDF versions or
have a stale build cache. In Arduino IDE select *Sketch → Clean Build Folder* then rebuild.

**Frames received but callbacks never fire** — confirm you called `watchFor()` after `begin()`,
and that the filter ID/mask matches your traffic. The filter table uses exact-match logic:
`(receivedId & mask) == (filterId & mask)`.

**Bus-off recovery** — the internal watchdog task polls every 200 ms and automatically
restarts the controller after a bus-off event. No action needed in your sketch.

**CAN-FD: `setDataBaudrate()` has no effect** — it must be called *before* `begin()`.
Calling it afterwards requires `CAN0.disable(); CAN0.enable();` to take effect.

**ESP32-P4 with three controllers** — each controller needs its own transceiver chip and
its own pair of GPIO pins. GPIO assignment is entirely flexible via `setCANPins()`; there
are no fixed TWAI GPIO pins on the P4.

---

## Additional examples

**CANipulator_test_forwarder**<br>
This example simply forwards all traffic from CAN0 to CAN1 a vice-versa. Great for debugging which messages originate from a certain node.

**CANipulator_C5_test_forwarder**<br>
Same as the previous, but with FD support.

Credits:<br>
[Collin80/esp32_can](https://github.com/collin80/esp32_can) - Original esp32_can library<br>
[outlandnish/esp32_can](https://github.com/outlandnish/esp32_can) - Majority of twai_v2 library updates<br>
[sans-ltd/esp32_can](https://github.com/sans-ltd/esp32_can) - Bugfixes

Implements a CAN driver for the built-in CAN hardware on an ESP32. 
The builtin CAN is called CAN0, and also CAN1 if there is a second CAN port on the ESP32 chip in use.
This library requires the can_common library. That library is a common base that 
other libraries can be built off of to allow a more universal API for CAN.

The needed can_common library is found here: https://github.com/collin80/can_common
Some examples use the SmartLeds library found here: https://github.com/RoboticsBrno/SmartLeds
