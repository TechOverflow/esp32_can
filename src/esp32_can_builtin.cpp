/*
  ESP32_CAN.cpp – Library for ESP32 built-in TWAI/CAN module

  Three-tier build:
    TIER_NEW  (IDF ≥ 5.5) – esp_driver_twai: twai_new_node_onchip, callback-based async RX, CAN-FD capable chips
    TIER_V2   (IDF 5.2–5.4) – legacy driver _v2 handle API
    TIER_V1   (IDF < 5.2)   – legacy single-controller API

  Backwards-compatible with the CAN_COMMON / can_common interface so
  existing sketches compile unchanged.

  Original author: Collin Kidder
  twai_v2 updates: outlandnish / TechOverflow
  IDF 5.5+ / CAN-FD update: see changelog
*/

#include "Arduino.h"
#include "esp32_can_builtin.h"
#include <algorithm>   // for std::min

static inline uint16_t saturate_u16(uint32_t v) {
    return v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Speed table (TIER_V1 / TIER_V2 only)
//  TIER_NEW passes the integer bitrate directly to twai_onchip_node_config_t
// ─────────────────────────────────────────────────────────────────────────────
#if !defined(TWAI_TIER_NEW)

static const VALID_TIMING valid_timings[] =
{
    {TWAI_TIMING_CONFIG_1MBITS(),   1000000},
    {TWAI_TIMING_CONFIG_500KBITS(),  500000},
    {TWAI_TIMING_CONFIG_250KBITS(),  250000},
    {TWAI_TIMING_CONFIG_125KBITS(),  125000},
    {TWAI_TIMING_CONFIG_800KBITS(),  800000},
    {TWAI_TIMING_CONFIG_100KBITS(),  100000},
    {TWAI_TIMING_CONFIG_50KBITS(),    50000},
    {TWAI_TIMING_CONFIG_25KBITS(),    25000},
    // Custom entries – not fully validated on all silicon
    // (brp multiples of 2 up to 128, multiples of 4 up to 256;
    //  TSEG1 1–16, TSEG2 1–8; default clock 80 MHz)
    {{.brp=100, .tseg_1=7,  .tseg_2=2, .sjw=3, .triple_sampling=false}, 80000},
    {{.brp=120, .tseg_1=15, .tseg_2=4, .sjw=3, .triple_sampling=false}, 33333},
    // ECO2 / ESP32-S3 only:
    {{.brp=200, .tseg_1=15, .tseg_2=4, .sjw=3, .triple_sampling=false}, 20000},
    {TWAI_TIMING_CONFIG_25KBITS(), 0}   // terminator – speed == 0 stops search
};

#endif // !TWAI_TIER_NEW

// ─────────────────────────────────────────────────────────────────────────────
//  Constructors
// ─────────────────────────────────────────────────────────────────────────────

ESP32CAN::ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin, uint8_t busNumber)
    : CAN_COMMON(BI_NUM_FILTERS),
      _rxPin(rxPin), _txPin(txPin), _busNum(busNumber)
{
    rxBufferSize = BI_RX_BUFFER_SIZE;

#if defined(TWAI_TIER_NEW)
    // nothing extra – config built in enable()
#else
    twai_general_cfg = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
    twai_general_cfg.tx_queue_len = BI_TX_BUFFER_SIZE;
    twai_general_cfg.rx_queue_len = 6;
    twai_speed_cfg   = TWAI_TIMING_CONFIG_500KBITS();
    twai_filters_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  #if defined(TWAI_TIER_V2)
    twai_general_cfg.controller_id = busNumber;
  #endif
#endif

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        filters[i] = {0, 0, false, false};
    }
    readyForTraffic = false;
    cyclesSinceTraffic = 0;
#if defined(TWAI_TIER_NEW)
    portMUX_INITIALIZE(&txMux);
  #if defined(ESP32CAN_HAS_FD)
    fdSupported = true;   // lets can_common's beginFD() reach initFD()
  #endif
#endif
}

ESP32CAN::ESP32CAN() : CAN_COMMON(BI_NUM_FILTERS)
{
    _rxPin  = GPIO_NUM_16;
    _txPin  = GPIO_NUM_17;
    _busNum = 0;
    rxBufferSize = BI_RX_BUFFER_SIZE;

#if !defined(TWAI_TIER_NEW)
    twai_general_cfg = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
    twai_general_cfg.tx_queue_len = BI_TX_BUFFER_SIZE;
    twai_general_cfg.rx_queue_len = 6;
    twai_speed_cfg   = TWAI_TIMING_CONFIG_500KBITS();
    twai_filters_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#endif

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        filters[i] = {0, 0, false, false};
    }
    readyForTraffic    = false;
    cyclesSinceTraffic = 0;
#if defined(TWAI_TIER_NEW)
    portMUX_INITIALIZE(&txMux);
  #if defined(ESP32CAN_HAS_FD)
    fdSupported = true;   // lets can_common's beginFD() reach initFD()
  #endif
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
//  Pin / buffer setters (must be called before init)
// ─────────────────────────────────────────────────────────────────────────────

void ESP32CAN::setCANPins(gpio_num_t rxPin, gpio_num_t txPin)
{
    _rxPin = rxPin;
    _txPin = txPin;
#if !defined(TWAI_TIER_NEW)
    twai_general_cfg.rx_io = rxPin;
    twai_general_cfg.tx_io = txPin;
#endif
}

void ESP32CAN::setRXBufferSize(int newSize)  { rxBufferSize = newSize; }

void ESP32CAN::setTXBufferSize(int newSize)
{
#if !defined(TWAI_TIER_NEW)
    twai_general_cfg.tx_queue_len = newSize;
#else
    // Applied at the next enable(). Capped by the compile-time TX buffer pool
    // (BI_TX_BUFFER_SIZE), because every queued frame needs its own buffer.
    if (newSize < 1) newSize = 1;
    _txQueueDepth = (newSize > BI_TX_BUFFER_SIZE) ? BI_TX_BUFFER_SIZE : newSize;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filter helpers
// ─────────────────────────────────────────────────────────────────────────────

int ESP32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox < BI_NUM_FILTERS) {
        filters[mailbox].id         = id & mask;
        filters[mailbox].mask       = mask;
        filters[mailbox].extended   = extended;
        filters[mailbox].configured = true;
        return mailbox;
    }
    return -1;
}

int ESP32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        if (!filters[i].configured) {
            return _setFilterSpecific(i, id, mask, extended);
        }
    }
    if (debuggingMode) Serial.println("Could not set filter!");
    return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal frame processing
//  Common to all tiers – applies the 32-slot software filter table then
//  routes accepted frames to callbacks or the rx_queue.
// ─────────────────────────────────────────────────────────────────────────────

bool ESP32CAN::processFrame(CAN_FRAME &msg)
{
    CANListener *thisListener;
    cyclesSinceTraffic = 0;

    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        if (!filters[i].configured) continue;
        if ((msg.id & filters[i].mask) == filters[i].id &&
             filters[i].extended == msg.extended)
        {
            if (cbCANFrame[i]) {
                msg.fid = i;
                xQueueSend(callbackQueue, &msg, 0);
                return true;
            } else if (cbGeneral) {
                msg.fid = 0xFF;
                xQueueSend(callbackQueue, &msg, 0);
                return true;
            } else {
                for (int lp = 0; lp < SIZE_LISTENERS; lp++) {
                    thisListener = listener[lp];
                    if (thisListener != nullptr) {
                        if (thisListener->isCallbackActive(i)) {
                            msg.fid = 0x80000000ul | (lp << 24ul) | i;
                            xQueueSend(callbackQueue, &msg, 0);
                            return true;
                        } else if (thisListener->isCallbackActive(numFilters)) {
                            msg.fid = 0x80000000ul | (lp << 24ul) | 0xFF;
                            xQueueSend(callbackQueue, &msg, 0);
                            return true;
                        }
                    }
                }
            }
            xQueueSend(rx_queue, &msg, 0);
            if (debuggingMode) Serial.write('_');
            return true;
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callback dispatcher task (all tiers)
//  Reads from callbackQueue and fires registered C or object callbacks.
// ─────────────────────────────────────────────────────────────────────────────

// fid encoding on callbackQueue:
//   0x80000000 | (listener << 24) | mb  → object listener, already resolved
//   0x40000000 | filter                 → object listener, resolve here
//   mb (0..31) / 0xFF                   → C callback (per-filter / general)
#define ESP32CAN_FID_LISTENER   0x80000000ul
#define ESP32CAN_FID_RESOLVE    0x40000000ul

void ESP32CAN::sendCallback(CAN_FRAME *frame)
{
    int mb  = frame->fid & 0xFF;
    if (mb == 0xFF) mb = -1;

    if ((frame->fid & (ESP32CAN_FID_LISTENER | ESP32CAN_FID_RESOLVE)) == ESP32CAN_FID_RESOLVE) {
        for (int lp = 0; lp < SIZE_LISTENERS; lp++) {
            CANListener *l = listener[lp];
            if (l == nullptr) continue;
            if (l->isCallbackActive(mb))         { l->gotFrame(frame, mb); return; }
            if (l->isCallbackActive(numFilters)) { l->gotFrame(frame, -1); return; }
        }
        // No listener wanted it – fall back to the polling queue, like processFrame()
        if (rx_queue) xQueueSend(rx_queue, frame, 0);
        return;
    }

    if (frame->fid & 0x80000000ul) {
        int idx = (frame->fid >> 24) & 0x7F;
        listener[idx]->gotFrame(frame, mb);
    } else {
        if (mb > -1) (*cbCANFrame[mb])(frame);
        else         (*cbGeneral)(frame);
    }
}

#if defined(TWAI_TIER_NEW) && defined(ESP32CAN_HAS_FD)
void ESP32CAN::sendCallbackFD(CAN_FRAME_FD *frame)
{
    int mb  = frame->fid & 0xFF;
    if (mb == 0xFF) mb = -1;

    if (frame->fid & ESP32CAN_FID_RESOLVE) {
        for (int lp = 0; lp < SIZE_LISTENERS; lp++) {
            CANListener *l = listener[lp];
            if (l == nullptr) continue;
            if (l->isCallbackActive(mb))         { l->gotFrameFD(frame, mb); return; }
            if (l->isCallbackActive(numFilters)) { l->gotFrameFD(frame, -1); return; }
        }
        if (rx_queue_fd) xQueueSend(rx_queue_fd, frame, 0);
        return;
    }
    if (mb > -1 && cbCANFrameFD[mb]) (*cbCANFrameFD[mb])(frame);
    else if (cbGeneralFD)            (*cbGeneralFD)(frame);
}
#endif

void ESP32CAN::task_CAN(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    CAN_FRAME rxFrame;
#if defined(TWAI_TIER_NEW) && defined(ESP32CAN_HAS_FD)
    CAN_FRAME_FD rxFrameFD;
#endif

    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        bool busy = false;
        if (espCan->callbackQueue &&
            xQueueReceive(espCan->callbackQueue, &rxFrame, 0) == pdTRUE) {
            espCan->sendCallback(&rxFrame);
            busy = true;
        }
#if defined(TWAI_TIER_NEW) && defined(ESP32CAN_HAS_FD)
        if (espCan->callbackQueueFD &&
            xQueueReceive(espCan->callbackQueueFD, &rxFrameFD, 0) == pdTRUE) {
            espCan->sendCallbackFD(&rxFrameFD);
            busy = true;
        }
#endif
        if (!busy) vTaskDelay(pdMS_TO_TICKS(4));
#if defined(CONFIG_FREERTOS_UNICORE)
        vTaskDelay(pdMS_TO_TICKS(6));
#endif
    }
    vTaskDelete(nullptr);
}

// ═════════════════════════════════════════════════════════════════════════════
//  TIER_NEW  (IDF ≥ 5.5)  –  esp_driver_twai
// ═════════════════════════════════════════════════════════════════════════════
#if defined(TWAI_TIER_NEW)

// ── ISR callbacks ─────────────────────────────────────────────────────────────
// Everything called from here must be ISR-safe: plain data access and
// FreeRTOS *FromISR calls only. Object listeners are resolved later in task_CAN.

bool IRAM_ATTR ESP32CAN::onRxDone(twai_node_handle_t handle,
                                   const twai_rx_done_event_data_t * /*edata*/,
                                   void *user_ctx)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(user_ctx);
    if (!espCan->readyForTraffic) return false;

    uint8_t buf[64];                // CAN-FD can carry up to 64 bytes
    twai_frame_t raw = {};
    raw.buffer     = buf;
    raw.buffer_len = sizeof(buf);
    if (twai_node_receive_from_isr(handle, &raw) != ESP_OK) return false;

    // header.dlc is a DLC *code*; buffer_len is only the capacity we passed in.
    uint16_t rxLen = twaifd_dlc2len(raw.header.dlc);
    if (rxLen > sizeof(buf)) rxLen = sizeof(buf);

    espCan->cyclesSinceTraffic = 0;
    espCan->trafficEverSeen    = true;

    // Software filter table (same semantics as processFrame on legacy tiers)
    int f = -1;
    for (int i = 0; i < BI_NUM_FILTERS; i++) {
        const ESP32_FILTER &flt = espCan->filters[i];
        if (flt.configured && (raw.header.id & flt.mask) == flt.id &&
            flt.extended == (bool)raw.header.ide) { f = i; break; }
    }
    if (f < 0) return false;

    bool anyListener = false;
    for (int lp = 0; lp < SIZE_LISTENERS; lp++) {
        if (espCan->listener[lp] != nullptr) { anyListener = true; break; }
    }

    BaseType_t woken = pdFALSE;

#if defined(ESP32CAN_HAS_FD)
    if (raw.header.fdf) {
        // FD frames only ever go to the FD queues; a classic-only consumer must
        // never see a silently truncated FD frame.
        CAN_FRAME_FD fd;
        fd.id       = raw.header.id;
        fd.extended = raw.header.ide;
        fd.fdMode   = 1;
        fd.rrs      = raw.header.brs ? 1 : 0;   // received BRS bit, see sendFrameFD
        fd.length   = rxLen;
        for (int i = 0; i < 64; i++) fd.data.uint8[i] = (i < (int)rxLen) ? buf[i] : 0;

        if (espCan->cbCANFrameFD[f] || espCan->cbGeneralFD) {
            fd.fid = espCan->cbCANFrameFD[f] ? f : 0xFF;
            if (espCan->callbackQueueFD) xQueueSendFromISR(espCan->callbackQueueFD, &fd, &woken);
        } else if (anyListener) {
            fd.fid = ESP32CAN_FID_RESOLVE | f;
            if (espCan->callbackQueueFD) xQueueSendFromISR(espCan->callbackQueueFD, &fd, &woken);
        } else if (espCan->rx_queue_fd) {
            xQueueSendFromISR(espCan->rx_queue_fd, &fd, &woken);
        }
        return (woken == pdTRUE);
    }
#endif

    CAN_FRAME msg;
    msg.id       = raw.header.id;
    msg.extended = raw.header.ide;
    msg.rtr      = raw.header.rtr;
    msg.length   = (rxLen <= 8) ? rxLen : 8;   // classic DLC 9–15 still means 8 bytes
    for (int i = 0; i < 8; i++) msg.data.byte[i] = (i < (int)rxLen) ? buf[i] : 0;

    if (espCan->cbCANFrame[f] || espCan->cbGeneral) {
        msg.fid = espCan->cbCANFrame[f] ? f : 0xFF;
        xQueueSendFromISR(espCan->callbackQueue, &msg, &woken);
    } else if (anyListener) {
        msg.fid = ESP32CAN_FID_RESOLVE | f;
        xQueueSendFromISR(espCan->callbackQueue, &msg, &woken);
    } else {
        xQueueSendFromISR(espCan->rx_queue, &msg, &woken);
    }
    return (woken == pdTRUE);
}

bool IRAM_ATTR ESP32CAN::onTxDone(twai_node_handle_t /*handle*/,
                                   const twai_tx_done_event_data_t *edata,
                                   void *user_ctx)
{
    // Release the TX buffer the driver was holding a pointer to.
    ESP32CAN *espCan = static_cast<ESP32CAN *>(user_ctx);
    if (!edata || !edata->done_tx_frame) return false;
    uintptr_t base = (uintptr_t)&espCan->txFrames[0];
    uintptr_t p    = (uintptr_t)edata->done_tx_frame;
    if (p >= base && p < base + sizeof(espCan->txFrames)) {
        uint32_t idx = (p - base) / sizeof(twai_frame_t);
        portENTER_CRITICAL_ISR(&espCan->txMux);
        espCan->txBusyMask &= ~(1ul << idx);
        portEXIT_CRITICAL_ISR(&espCan->txMux);
    }
    return false;
}

bool IRAM_ATTR ESP32CAN::onError(twai_node_handle_t handle,
                                  const twai_error_event_data_t *edata,
                                  void *user_ctx)
{
    // Fires on every bus error, most of which are transient (no ACK,
    // arbitration loss, form error) and absorbed by the controller as TEC
    // drains. Do NOT touch readyForTraffic here: one error would gate callers
    // off sendFrame and TEC could never drain. onStateChange owns that flag.
    (void)handle;
    (void)edata;
    (void)user_ctx;
    return false;
}

bool IRAM_ATTR ESP32CAN::onStateChange(twai_node_handle_t handle,
                                        const twai_state_change_event_data_t *edata,
                                        void *user_ctx)
{
    // Authoritative writer of readyForTraffic:
    //   ACTIVE / WARNING / PASSIVE – controller can still transmit
    //   BUS_OFF                    – TX blocked until twai_node_recover()
    //                                completes (watchdog task starts it)
    ESP32CAN *espCan = static_cast<ESP32CAN *>(user_ctx);
    (void)handle;
    if (!edata) return false;

    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        espCan->readyForTraffic = false;
        espCan->_busOffPending  = true;
    } else {
        espCan->readyForTraffic = true;
    }
    return false;
}

// ── Watchdog task (TIER_NEW) ──────────────────────────────────────────────────
// Every 200 ms: starts bus-off recovery, and restarts the node if traffic that
// was flowing stops for > 2 s.

void ESP32CAN::CAN_WatchDog_Builtin(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(200);
    int recoverWait = 0;

    for (;;) {
        vTaskDelay(xDelay);
        espCan->cyclesSinceTraffic++;

        twai_node_handle_t node = espCan->node_handle;
        if (node == nullptr || !espCan->_nodeEnabled) continue;

        twai_node_status_t status;
        twai_node_record_t record;
        if (twai_node_get_info(node, &status, &record) != ESP_OK) continue;

        if (status.state == TWAI_ERROR_BUS_OFF) {
            // The driver reports a *disabled* node as BUS_OFF too; _nodeEnabled
            // (checked above) tells us this is a genuine bus-off. Recovery needs
            // 128 × 11 recessive bits, then onStateChange(ACTIVE) re-arms TX.
            // Re-issue every ~2 s in case the bus was held dominant.
            espCan->readyForTraffic = false;
            if (espCan->_busOffPending || ++recoverWait >= 10) {
                espCan->_busOffPending = false;
                recoverWait = 0;
                if (twai_node_recover(node) == ESP_OK) {
                    log_w("Bus %d bus-off, recovery started", espCan->_busNum);
                }
            }
            continue;
        }
        recoverWait = 0;

        // Stall detection — traffic was flowing, then nothing for > 2 s
        if (espCan->cyclesSinceTraffic > 10 &&
            espCan->readyForTraffic         &&
            espCan->trafficEverSeen)
        {
            espCan->cyclesSinceTraffic = 0;
            espCan->readyForTraffic    = false;
            if (twai_node_disable(node) == ESP_OK) espCan->_nodeEnabled = false;
            if (twai_node_enable(node) == ESP_OK) {
                espCan->_nodeEnabled    = true;
                espCan->readyForTraffic = true;
            }
        }
    }
}

// ── enable / disable (TIER_NEW) ───────────────────────────────────────────────

void ESP32CAN::enable()
{
    if (node_handle != nullptr) {
        // Already installed – just start it
        if (!_nodeEnabled && twai_node_enable(node_handle) == ESP_OK) {
            _nodeEnabled    = true;
            readyForTraffic = true;
        }
        return;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = _txPin,
            .rx = _rxPin,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = _baudrate,
        },
        .fail_retry_cnt = 3,     // -1 would retry an un-ACKed frame forever
        .tx_queue_depth = _txQueueDepth,
        .flags = {
            .enable_self_test   = noACK     ? 1u : 0u,
            .enable_listen_only = listenOnly ? 1u : 0u,
        },
    };

#if defined(ESP32CAN_HAS_FD)
    // CAN-FD data phase – only populated when caller set a data baudrate
    if (_dataBaudrate > 0) {
        node_cfg.data_timing.bitrate = _dataBaudrate;
    }
#endif

    if (twai_new_node_onchip(&node_cfg, &node_handle) != ESP_OK) {
        log_e("Failed to create TWAI node (bus %d)", _busNum);
        node_handle = nullptr;
        return;
    }
    log_i("TWAI node created (bus %d, %lu bps)", _busNum, _baudrate);

    // Fresh node: no frames in flight
    portENTER_CRITICAL(&txMux);
    txBusyMask = 0;
    portEXIT_CRITICAL(&txMux);
    _busOffPending = false;

    // Register callbacks
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done      = ESP32CAN::onRxDone;
    cbs.on_tx_done      = ESP32CAN::onTxDone;
    cbs.on_error        = ESP32CAN::onError;
    cbs.on_state_change = ESP32CAN::onStateChange;
    twai_node_register_event_callbacks(node_handle, &cbs, this);

    // Create queues
    callbackQueue = xQueueCreate(16,           sizeof(CAN_FRAME));
    rx_queue      = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));
#if defined(ESP32CAN_HAS_FD)
    callbackQueueFD = xQueueCreate(8,            sizeof(CAN_FRAME_FD));
    rx_queue_fd     = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME_FD));
#endif

    // Callback dispatcher task
    char taskName[24];
    snprintf(taskName, sizeof(taskName), "CAN_RX_CAN%d", _busNum);
    xTaskCreate(ESP32CAN::task_CAN, taskName, 8192, this, 15, &task_CAN_handler);

    if (twai_node_enable(node_handle) == ESP_OK) {
        log_i("TWAI node enabled (bus %d)", _busNum);
        _nodeEnabled    = true;
        readyForTraffic = true;
    } else {
        log_e("Failed to enable TWAI node (bus %d)", _busNum);
    }
}

void ESP32CAN::disable()
{
    readyForTraffic = false;

    if (node_handle != nullptr) {
        twai_node_handle_t node = node_handle;
        node_handle = nullptr;               // watchdog stops touching it
        if (_nodeEnabled) twai_node_disable(node);   // fails harmlessly if bus-off
        _nodeEnabled = false;
        twai_node_delete(node);
    }
    // Deleting the node drops anything still queued
    portENTER_CRITICAL(&txMux);
    txBusyMask = 0;
    portEXIT_CRITICAL(&txMux);

    if (task_CAN_handler != nullptr) {
        vTaskDelete(task_CAN_handler);
        task_CAN_handler = nullptr;
    }

    if (rx_queue)      { vQueueDelete(rx_queue);      rx_queue      = nullptr; }
    if (callbackQueue) { vQueueDelete(callbackQueue); callbackQueue = nullptr; }
#if defined(ESP32CAN_HAS_FD)
    if (rx_queue_fd)     { vQueueDelete(rx_queue_fd);     rx_queue_fd     = nullptr; }
    if (callbackQueueFD) { vQueueDelete(callbackQueueFD); callbackQueueFD = nullptr; }
#endif
}

// ── _init  (TIER_NEW) ─────────────────────────────────────────────────────────

void ESP32CAN::_init()
{
    if (debuggingMode) Serial.println("[TWAI] _init (TIER_NEW)");

    for (int i = 0; i < BI_NUM_FILTERS; i++) filters[i] = {0, 0, false, false};

    if (CAN_WatchDog_Builtin_handler == nullptr) {
        char wdName[28];
        snprintf(wdName, sizeof(wdName), "CAN_WD_BI_CAN%d", _busNum);
#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                    &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                                &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
}

// ── init / set_baudrate (TIER_NEW) ────────────────────────────────────────────

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _baudrate = ul_baudrate;
    _init();
    enable();
    return (node_handle != nullptr) ? ul_baudrate : 0;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    _baudrate = ul_baudrate;
    disable();
    enable();
    return (node_handle != nullptr) ? ul_baudrate : 0;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    // Save and restore mode after probing
    bool savedListen = listenOnly;
    listenOnly = true;
    _init();
    readyForTraffic = false;

    // Probe common speeds from fast to slow
    const uint32_t speeds[] = {1000000, 800000, 500000, 250000, 125000,
                                100000, 80000, 50000, 33333, 25000, 20000, 0};
    for (int i = 0; speeds[i] != 0; i++) {
        _baudrate = speeds[i];
        disable();
        Serial.print("Trying speed "); Serial.print(speeds[i]);
        enable();
        delay(600);
        if (cyclesSinceTraffic < 2) {
            disable();
            listenOnly = savedListen;
            enable();
            Serial.println(" SUCCESS!");
            return speeds[i];
        }
        Serial.println(" FAILED.");
    }

    Serial.println("None of the tested CAN speeds worked!");
    disable();
    listenOnly = savedListen;
    return 0;
}

// ── Mode setters (TIER_NEW) ───────────────────────────────────────────────────

void ESP32CAN::setListenOnlyMode(bool state)
{
    listenOnly = state;
    disable();
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    noACK = state;
    disable();
    enable();
}

// ── Transmit (TIER_NEW) ───────────────────────────────────────────────────────
// twai_node_transmit() keeps a pointer to the frame until on_tx_done, so each
// queued frame gets its own slot in txFrames/txBufs; onTxDone frees it.

bool ESP32CAN::_queueTx(uint32_t id, bool ext, bool rtr, bool fdf, bool brs,
                        const uint8_t *data, uint8_t len)
{
    if (node_handle == nullptr || !_nodeEnabled) return false;

    const uint32_t allSlots = (BI_TX_SLOTS >= 32) ? 0xFFFFFFFFul
                                                  : ((1ul << BI_TX_SLOTS) - 1);
    int slot = -1;
    portENTER_CRITICAL(&txMux);
    uint32_t freeMask = ~txBusyMask & allSlots;
    if (freeMask) {
        slot = __builtin_ctz(freeMask);
        txBusyMask |= (1ul << slot);
    }
    portEXIT_CRITICAL(&txMux);
    if (slot < 0) {                      // every buffer still in flight
        if (debuggingMode) Serial.write('F');
        return false;
    }

    twai_frame_t *f   = &txFrames[slot];
    uint8_t      *buf = txBufs[slot];
    memset(f, 0, sizeof(*f));

    if (len > BI_TX_PAYLOAD_MAX) len = BI_TX_PAYLOAD_MAX;
    // FD lengths above 8 only exist in steps (12, 16, 20, 24, 32, 48, 64);
    // round up and zero-pad so the bytes on the wire are defined.
    uint16_t wireLen = fdf ? twaifd_dlc2len(twaifd_len2dlc(len)) : len;
    if (wireLen > BI_TX_PAYLOAD_MAX) wireLen = BI_TX_PAYLOAD_MAX;
    memcpy(buf, data, len);
    if (wireLen > len) memset(buf + len, 0, wireLen - len);

    f->header.id  = id;
    f->header.ide = ext ? 1 : 0;
    f->header.rtr = rtr ? 1 : 0;
#if defined(ESP32CAN_HAS_FD)
    f->header.fdf = fdf ? 1 : 0;
    f->header.brs = brs ? 1 : 0;
#else
    (void)fdf; (void)brs;
#endif
    if (rtr) {
        // Remote frame: no payload, but the requested length goes in the DLC
        f->header.dlc = len;
        f->buffer     = buf;
        f->buffer_len = 0;
    } else {
        f->buffer     = buf;          // dlc 0 → driver derives it from buffer_len
        f->buffer_len = wireLen;
    }

    esp_err_t res = twai_node_transmit(node_handle, f, 0 /*no wait*/);
    if (res != ESP_OK) {
        portENTER_CRITICAL(&txMux);
        txBusyMask &= ~(1ul << slot);
        portEXIT_CRITICAL(&txMux);
    }

    switch (res) {
        case ESP_OK:            if (debuggingMode) Serial.write('<'); break;
        case ESP_ERR_TIMEOUT:   if (debuggingMode) Serial.write('T'); break;
        default:                if (debuggingMode) Serial.write('!'); break;
    }
    return (res == ESP_OK);
}

bool ESP32CAN::sendFrame(CAN_FRAME &txFrame)
{
    uint8_t len = txFrame.length <= 8 ? txFrame.length : 8;   // CAN_FRAME holds 8 bytes
    return _queueTx(txFrame.id, txFrame.extended, txFrame.rtr,
                    false, false, txFrame.data.byte, len);
}

#if defined(ESP32CAN_HAS_FD)
bool ESP32CAN::sendFrameFD(CAN_FRAME_FD &txFrame)
{
    // Always sent as an FD frame (fdf=1), as before.
    // BRS: on whenever a data baudrate is set, unless setPerFrameBRS(true),
    // in which case txFrame.rrs decides (lets a bridge mirror the RX BRS bit).
    bool brs = (_dataBaudrate > 0) && (!_perFrameBRS || txFrame.rrs);
    uint8_t len = txFrame.length <= 64 ? txFrame.length : 64;
    return _queueTx(txFrame.id, txFrame.extended, false, true, brs,
                    txFrame.data.uint8, len);
}

uint32_t ESP32CAN::get_rx_buffFD(CAN_FRAME_FD &msg)
{
    if (!rx_queue_fd || !uxQueueMessagesWaiting(rx_queue_fd)) return false;
    CAN_FRAME_FD frame;
    if (xQueueReceive(rx_queue_fd, &frame, 0) == pdTRUE) {
        msg = frame;
        return true;
    }
    return false;
}

uint16_t ESP32CAN::availableFD()
{
    if (!rx_queue_fd) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue_fd);
}

uint32_t ESP32CAN::initFD(uint32_t nominalRate, uint32_t dataRate)
{
    _dataBaudrate = dataRate;
    fd_DataSpeed  = dataRate;
    return init(nominalRate);
}

uint32_t ESP32CAN::set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed)
{
    _dataBaudrate = dataSpeed;
    fd_DataSpeed  = dataSpeed;
    return set_baudrate(nominalSpeed);
}
#endif

// ── Queue accessors (TIER_NEW – same as other tiers) ─────────────────────────

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    if (!rx_queue) return false;
    if (!uxQueueMessagesWaiting(rx_queue)) return false;
    CAN_FRAME frame;
    if (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) {
        msg = frame;
        return true;
    }
    return false;
}

void ESP32CAN::resetIfStale(uint32_t stallMs)
{
    // cyclesSinceTraffic is incremented every 200 ms by the watchdog task
    uint32_t threshold = stallMs / 200;
    if (cyclesSinceTraffic > threshold) {
        disable();
        enable();
    }
}

bool ESP32CAN::getControllerStats(CANControllerStats &stats)
{
    if (node_handle == nullptr) return false;

    twai_node_status_t status;
    twai_node_record_t record;
    if (twai_node_get_info(node_handle, &status, &record) != ESP_OK) {
        return false;
    }

    stats.txErrorCount  = status.tx_error_count;
    stats.rxErrorCount  = status.rx_error_count;
    stats.busErrorCount = saturate_u16(record.bus_err_num);
    // esp_driver_twai has no public RX overrun counter (IDF 5.5 / 6.1).
    stats.rxOverrunCount = 0;
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  TIER_V2  (IDF 5.2–5.4)  –  legacy _v2 handle-based API
// ═════════════════════════════════════════════════════════════════════════════
#elif defined(TWAI_TIER_V2)

// ── Watchdog task ─────────────────────────────────────────────────────────────

// Legacy driver state machine: BUS_OFF → twai_initiate_recovery → RECOVERING
// → STOPPED (needs twai_start). Stop/start is only valid while RUNNING/STOPPED.
// Error-passive is a normal operating state and is left to the controller.
void ESP32CAN::CAN_WatchDog_Builtin(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(200);
    twai_status_info_t status_info;

    for (;;) {
        vTaskDelay(xDelay);
        espCan->cyclesSinceTraffic++;

        twai_handle_t h = espCan->bus_handle;
        if (h == nullptr) continue;
        if (twai_get_status_info_v2(h, &status_info) != ESP_OK) continue;

        switch (status_info.state) {
            case TWAI_STATE_BUS_OFF:
                espCan->readyForTraffic = false;
                if (!espCan->_recovering &&
                    twai_initiate_recovery_v2(h) == ESP_OK) {
                    espCan->_recovering = true;
                    log_w("Bus %d bus-off, recovery started", espCan->_busNum);
                }
                break;

            case TWAI_STATE_RECOVERING:
                break;

            case TWAI_STATE_STOPPED:
                // Only restart if we got here via recovery, not a user stop
                if (espCan->_recovering) {
                    espCan->_recovering = false;
                    if (twai_start_v2(h) == ESP_OK) {
                        espCan->cyclesSinceTraffic = 0;
                        espCan->readyForTraffic    = true;
                    }
                }
                break;

            case TWAI_STATE_RUNNING:
            default:
                // Stall detection — traffic was flowing, then nothing for > 2 s
                if (espCan->cyclesSinceTraffic > 10 &&
                    espCan->readyForTraffic         &&
                    espCan->trafficEverSeen)
                {
                    espCan->cyclesSinceTraffic = 0;
                    espCan->readyForTraffic    = false;
                    twai_stop_v2(h);
                    if (twai_start_v2(h) == ESP_OK) {
                        espCan->readyForTraffic = true;
                    }
                }
                break;
        }
    }
}

// ── RX task ───────────────────────────────────────────────────────────────────

void ESP32CAN::task_LowLevelRX(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);

    while (1) {
        if (espCan->readyForTraffic) {
            twai_message_t message;
            if (twai_receive_v2(espCan->bus_handle, &message,
                                pdMS_TO_TICKS(100)) == ESP_OK)
            {
                CAN_FRAME msg;
                msg.id       = message.identifier;
                msg.length   = message.data_length_code;
                msg.rtr      = message.rtr;
                msg.extended = message.extd;
                for (int i = 0; i < 8; i++) msg.data.byte[i] = message.data[i];
                espCan->processFrame(msg);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ── enable / disable (TIER_V2) ────────────────────────────────────────────────

void ESP32CAN::enable()
{
    _recovering = false;
    twai_general_cfg.rx_io = _rxPin;
    twai_general_cfg.tx_io = _txPin;
    twai_general_cfg.mode  = listenOnly ? TWAI_MODE_LISTEN_ONLY :
                              noACK     ? TWAI_MODE_NO_ACK      :
                                          TWAI_MODE_NORMAL;

    if (twai_driver_install_v2(&twai_general_cfg, &twai_speed_cfg,
                                &twai_filters_cfg, &bus_handle) != ESP_OK) {
        log_e("Failed to install driver (bus %d)", _busNum);
        return;
    }

    callbackQueue = xQueueCreate(16,           sizeof(CAN_FRAME));
    rx_queue      = xQueueCreate(rxBufferSize,  sizeof(CAN_FRAME));

    char rxName[24], cbName[24];
    snprintf(rxName, sizeof(rxName), "CAN_LORX_CAN%d", _busNum);
    snprintf(cbName, sizeof(cbName), "CAN_RX_CAN%d",   _busNum);

    xTaskCreate(ESP32CAN::task_CAN, cbName, 8192, this, 15, &task_CAN_handler);
#if defined(CONFIG_FREERTOS_UNICORE)
    xTaskCreate(ESP32CAN::task_LowLevelRX, rxName, 4096, this, 19,
                &task_LowLevelRX_handler);
#else
    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, rxName, 4096, this, 19,
                            &task_LowLevelRX_handler, 1);
#endif

    if (twai_start_v2(bus_handle) != ESP_OK) {
        log_e("Failed to start driver (bus %d)", _busNum);
        return;
    }
    log_i("TWAI driver started (bus %d)", _busNum);
    readyForTraffic = true;
}

void ESP32CAN::disable()
{
    readyForTraffic = false;

    twai_status_info_t info;
    if (twai_get_status_info_v2(bus_handle, &info) == ESP_OK) {
        if (info.state == TWAI_STATE_RUNNING) {
            twai_stop_v2(bus_handle);
        }
    }

    for (auto task : {task_CAN_handler, task_LowLevelRX_handler}) {
        if (task != nullptr) { vTaskDelete(task); }
    }
    task_CAN_handler        = nullptr;
    task_LowLevelRX_handler = nullptr;

    if (rx_queue)      { vQueueDelete(rx_queue);      rx_queue      = nullptr; }
    if (callbackQueue) { vQueueDelete(callbackQueue); callbackQueue = nullptr; }

    twai_driver_uninstall_v2(bus_handle);
    bus_handle = nullptr;
}

// ── _init (TIER_V2) ───────────────────────────────────────────────────────────

void ESP32CAN::_init()
{
    for (int i = 0; i < BI_NUM_FILTERS; i++) filters[i] = {0, 0, false, false};

    if (CAN_WatchDog_Builtin_handler == nullptr) {
        char wdName[28];
        snprintf(wdName, sizeof(wdName), "CAN_WD_BI_CAN%d", _busNum);
#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                    &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, wdName, 2048, this, 10,
                                &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
}

// ── init / set_baudrate (TIER_V2) ─────────────────────────────────────────────

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _init();
    set_baudrate(ul_baudrate);

    if (debuggingMode) {
        uint32_t alerts = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                          TWAI_ALERT_AND_LOG  | TWAI_ALERT_ERR_ACTIVE |
                          TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR |
                          TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL;
        twai_reconfigure_alerts_v2(bus_handle, alerts, nullptr);
    }

    // The low-level RX task is created (and tracked) by enable(); creating a
    // second, untracked one here gave two readers on the same bus.
    return (bus_handle != nullptr) ? ul_baudrate : 0;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    disable();
    for (int i = 0; valid_timings[i].speed != 0; i++) {
        if (valid_timings[i].speed == ul_baudrate) {
            twai_speed_cfg = valid_timings[i].cfg;
            enable();
            return ul_baudrate;
        }
    }
    log_e("Could not find valid bit timing for %lu bps!", ul_baudrate);
    return 0;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    twai_general_config_t savedMode = twai_general_cfg;
    _init();
    readyForTraffic = false;
    twai_stop_v2(bus_handle);
    twai_general_cfg.mode = TWAI_MODE_LISTEN_ONLY;

    for (int i = 0; valid_timings[i].speed != 0; i++) {
        twai_speed_cfg = valid_timings[i].cfg;
        disable();
        Serial.print("Trying speed "); Serial.print(valid_timings[i].speed);
        enable();
        delay(600);
        if (cyclesSinceTraffic < 2) {
            disable();
            twai_general_cfg.mode = savedMode.mode;
            enable();
            Serial.println(" SUCCESS!");
            return valid_timings[i].speed;
        }
        Serial.println(" FAILED.");
    }

    Serial.println("None of the tested CAN speeds worked!");
    twai_stop_v2(bus_handle);
    return 0;
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    listenOnly = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    noACK = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
    enable();
}

// ── sendFrame (TIER_V2) ───────────────────────────────────────────────────────

bool ESP32CAN::sendFrame(CAN_FRAME &txFrame)
{
    twai_message_t raw = {};
    raw.identifier        = txFrame.id;
    raw.data_length_code  = txFrame.length;
    raw.rtr               = txFrame.rtr;
    raw.extd              = txFrame.extended;
    for (int i = 0; i < 8; i++) raw.data[i] = txFrame.data.byte[i];

    esp_err_t res = twai_transmit_v2(bus_handle, &raw, pdMS_TO_TICKS(4));
    switch (res) {
        case ESP_OK:           if (debuggingMode) Serial.write('<'); break;
        case ESP_ERR_TIMEOUT:  if (debuggingMode) Serial.write('T'); break;
        default:               if (debuggingMode) Serial.write('!'); break;
    }
    return (res == ESP_OK);
}

// ── Queue accessors (TIER_V2) ─────────────────────────────────────────────────

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    if (!rx_queue || !uxQueueMessagesWaiting(rx_queue)) return false;
    CAN_FRAME frame;
    if (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) { msg = frame; return true; }
    return false;
}

void ESP32CAN::resetIfStale(uint32_t stallMs)
{
    // cyclesSinceTraffic is incremented every 200 ms by the watchdog task
    uint32_t threshold = stallMs / 200;
    if (cyclesSinceTraffic > threshold) {
        disable();
        enable();
    }
}

bool ESP32CAN::getControllerStats(CANControllerStats &stats)
{
    if (bus_handle == nullptr) return false;

    twai_status_info_t info;
    if (twai_get_status_info_v2(bus_handle, &info) != ESP_OK) {
        return false;
    }

    // twai_status_info_t fields are uint32_t in the legacy driver.
    // Saturate to keep the wire format compact and stable across tiers.
    stats.txErrorCount   = saturate_u16(info.tx_error_counter);
    stats.rxErrorCount   = saturate_u16(info.rx_error_counter);
    stats.busErrorCount  = saturate_u16(info.bus_error_count);
    stats.rxOverrunCount = saturate_u16(info.rx_overrun_count);
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  TIER_V1  (IDF < 5.2)  –  legacy single-controller API
// ═════════════════════════════════════════════════════════════════════════════
#else // TWAI_TIER_V1

void ESP32CAN::CAN_WatchDog_Builtin(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(200);
    twai_status_info_t status_info;

    // BUS_OFF → twai_initiate_recovery → RECOVERING → STOPPED → twai_start
    for (;;) {
        vTaskDelay(xDelay);
        espCan->cyclesSinceTraffic++;

        if (twai_get_status_info(&status_info) != ESP_OK) continue;  // not installed

        if (status_info.state == TWAI_STATE_BUS_OFF) {
            espCan->cyclesSinceTraffic = 0;
            espCan->readyForTraffic    = false;
            if (!espCan->_recovering) {
                if (twai_initiate_recovery() == ESP_OK) {
                    espCan->_recovering = true;
                    log_w("Bus-off, recovery started");
                } else {
                    log_e("Could not initiate bus recovery");
                }
            }
        } else if (status_info.state == TWAI_STATE_STOPPED && espCan->_recovering) {
            espCan->_recovering = false;
            if (twai_start() == ESP_OK) espCan->readyForTraffic = true;
        }
    }
}

void ESP32CAN::task_LowLevelRX(void *pvParameters)
{
    ESP32CAN *espCan = static_cast<ESP32CAN *>(pvParameters);
    while (1) {
        if (espCan->readyForTraffic) {
            twai_message_t message;
            if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
                CAN_FRAME msg;
                msg.id       = message.identifier;
                msg.length   = message.data_length_code;
                msg.rtr      = message.rtr;
                msg.extended = message.extd;
                for (int i = 0; i < 8; i++) msg.data.byte[i] = message.data[i];
                espCan->processFrame(msg);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void ESP32CAN::enable()
{
    _recovering = false;
    twai_general_cfg.rx_io = _rxPin;
    twai_general_cfg.tx_io = _txPin;
    twai_general_cfg.mode  = listenOnly ? TWAI_MODE_LISTEN_ONLY :
                              noACK     ? TWAI_MODE_NO_ACK      :
                                          TWAI_MODE_NORMAL;

    if (twai_driver_install(&twai_general_cfg, &twai_speed_cfg,
                             &twai_filters_cfg) != ESP_OK) {
        log_e("Failed to install driver");
        return;
    }

    callbackQueue = xQueueCreate(16,           sizeof(CAN_FRAME));
    rx_queue      = xQueueCreate(rxBufferSize,  sizeof(CAN_FRAME));

    xTaskCreate(ESP32CAN::task_CAN,         "CAN_RX_CAN",   8192, this, 15, &task_CAN_handler);
#if defined(CONFIG_FREERTOS_UNICORE)
    xTaskCreate(ESP32CAN::task_LowLevelRX,  "CAN_LORX_CAN", 4096, this, 19, &task_LowLevelRX_handler);
#else
    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, "CAN_LORX_CAN", 4096, this, 19,
                            &task_LowLevelRX_handler, 1);
#endif

    if (twai_start() != ESP_OK) {
        log_e("Failed to start driver");
        return;
    }
    log_i("TWAI driver started");
    readyForTraffic = true;
}

void ESP32CAN::disable()
{
    readyForTraffic = false;
    twai_status_info_t info;
    if (twai_get_status_info(&info) == ESP_OK) {
        if (info.state == TWAI_STATE_RUNNING) twai_stop();
    }

    for (auto task : {task_CAN_handler, task_LowLevelRX_handler}) {
        if (task != nullptr) { vTaskDelete(task); }
    }
    task_CAN_handler        = nullptr;
    task_LowLevelRX_handler = nullptr;

    if (rx_queue)      { vQueueDelete(rx_queue);      rx_queue      = nullptr; }
    if (callbackQueue) { vQueueDelete(callbackQueue); callbackQueue = nullptr; }

    twai_driver_uninstall();
}

void ESP32CAN::_init()
{
    for (int i = 0; i < BI_NUM_FILTERS; i++) filters[i] = {0, 0, false, false};

    if (CAN_WatchDog_Builtin_handler == nullptr) {
#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, "CAN_WD_BI", 2048, this, 10,
                    &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, "CAN_WD_BI", 2048, this, 10,
                                &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _init();
    set_baudrate(ul_baudrate);

    if (debuggingMode) {
        uint32_t alerts = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                          TWAI_ALERT_AND_LOG  | TWAI_ALERT_ERR_ACTIVE |
                          TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR |
                          TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL;
        twai_reconfigure_alerts(alerts, nullptr);
    }

    // The low-level RX task is created (and tracked) by enable().
    return readyForTraffic ? ul_baudrate : 0;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    disable();
    for (int i = 0; valid_timings[i].speed != 0; i++) {
        if (valid_timings[i].speed == ul_baudrate) {
            twai_speed_cfg = valid_timings[i].cfg;
            enable();
            return ul_baudrate;
        }
    }
    log_e("Could not find valid bit timing for %lu bps!", ul_baudrate);
    return 0;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    twai_general_config_t savedMode = twai_general_cfg;
    _init();
    readyForTraffic = false;
    twai_stop();
    twai_general_cfg.mode = TWAI_MODE_LISTEN_ONLY;

    for (int i = 0; valid_timings[i].speed != 0; i++) {
        twai_speed_cfg = valid_timings[i].cfg;
        disable();
        Serial.print("Trying speed "); Serial.print(valid_timings[i].speed);
        enable();
        delay(600);
        if (cyclesSinceTraffic < 2) {
            disable();
            twai_general_cfg.mode = savedMode.mode;
            enable();
            Serial.println(" SUCCESS!");
            return valid_timings[i].speed;
        }
        Serial.println(" FAILED.");
    }

    Serial.println("None of the tested CAN speeds worked!");
    twai_stop();
    return 0;
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    listenOnly = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    noACK = state;
    disable();
    twai_general_cfg.mode = state ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
    enable();
}

bool ESP32CAN::sendFrame(CAN_FRAME &txFrame)
{
    twai_message_t raw = {};
    raw.identifier       = txFrame.id;
    raw.data_length_code = txFrame.length;
    raw.rtr              = txFrame.rtr;
    raw.extd             = txFrame.extended;
    for (int i = 0; i < 8; i++) raw.data[i] = txFrame.data.byte[i];

    esp_err_t res = twai_transmit(&raw, pdMS_TO_TICKS(4));
    switch (res) {
        case ESP_OK:           if (debuggingMode) Serial.write('<'); break;
        case ESP_ERR_TIMEOUT:  if (debuggingMode) Serial.write('T'); break;
        default:               if (debuggingMode) Serial.write('!'); break;
    }
    return (res == ESP_OK);
}

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return (uint16_t)uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    if (!rx_queue || !uxQueueMessagesWaiting(rx_queue)) return false;
    CAN_FRAME frame;
    if (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) { msg = frame; return true; }
    return false;
}

bool ESP32CAN::getControllerStats(CANControllerStats &stats)
{
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) {
        return false;
    }

    stats.txErrorCount   = saturate_u16(info.tx_error_counter);
    stats.rxErrorCount   = saturate_u16(info.rx_error_counter);
    stats.busErrorCount  = saturate_u16(info.bus_error_count);
    stats.rxOverrunCount = saturate_u16(info.rx_overrun_count);
    return true;
}

#endif // TWAI_TIER_*
