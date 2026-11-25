/*
  esp32_can_builtin.h - Header for ESP32 built-in TWAI/CAN driver
  Updated for:
    - ESP-IDF < 5.2  : legacy twai_driver_install (single controller)
    - ESP-IDF 5.2-5.4: _v2 handle-based legacy API (multi-controller)
    - ESP-IDF 5.5+   : new esp_driver_twai API (twai_new_node_onchip,
                        callback-based RX, CAN-FD on capable chips)
*/

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_idf_version.h"
#include "soc/soc_caps.h"
#include "can_common.h"

// ── CAN-FD capability ─────────────────────────────────────────────────────────
// IDF 5.5 names the capability SOC_TWAI_SUPPORT_FD; IDF 6.x renamed it to
// SOC_TWAI_FD_SUPPORTED (arduino-esp32 4.x). Use ESP32CAN_HAS_FD everywhere.
#if defined(SOC_TWAI_SUPPORT_FD) || defined(SOC_TWAI_FD_SUPPORTED)
  #define ESP32CAN_HAS_FD 1
#endif

// ── Driver-tier selection ─────────────────────────────────────────────────────
// TIER_NEW  : IDF 5.5+ esp_driver_twai (twai_new_node_onchip, callbacks)
// TIER_V2   : IDF 5.2–5.4 legacy _v2 handle API
// TIER_V1   : IDF < 5.2  legacy single-controller API
// ─────────────────────────────────────────────────────────────────────────────
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  #define TWAI_TIER_NEW  1
  #include "esp_twai.h"
  #include "esp_twai_onchip.h"
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
  #define TWAI_TIER_V2   1
  #include "driver/twai.h"
#else
  #define TWAI_TIER_V1   1
  #include "driver/twai.h"
#endif

#define BI_NUM_FILTERS  32
#define BI_RX_BUFFER_SIZE 32
#define BI_TX_BUFFER_SIZE 20

#if defined(TWAI_TIER_NEW)
// The new driver keeps a *pointer* to every queued frame until it has been
// sent, so each in-flight frame needs its own buffer. In flight = driver queue
// depth + frames already loaded into hardware TX slots (HAL supports up to 8).
// Slots are freed from on_tx_done; must stay <= 32 (tracked in a bitmask).
  #define BI_TX_SLOTS       (BI_TX_BUFFER_SIZE + 8 + 1)
  #if defined(ESP32CAN_HAS_FD)
    #define BI_TX_PAYLOAD_MAX 64
  #else
    #define BI_TX_PAYLOAD_MAX 8
  #endif
#endif

typedef struct {
    uint32_t id;
    uint32_t mask;
    bool     extended;
    bool     configured;
} ESP32_FILTER;

/// Per-controller diagnostic counters surfaced from the underlying IDF
/// driver. These are the same values the controller's automatic error
/// confinement state machine uses internally (TEC reaches 128 →
/// error-passive; reaches 256 → bus-off; etc).
///
/// All fields are saturated to uint16_t. `rxOverrunCount` is 0 on
/// TIER_NEW (the new esp_driver_twai doesn't expose it); use V1 or V2
/// builds if you need it. `busErrorCount` is the cumulative bus error
/// count since boot or last bus-off recovery.
typedef struct {
    uint16_t txErrorCount;     // TEC — transmit error counter
    uint16_t rxErrorCount;     // REC — receive error counter
    uint16_t busErrorCount;    // cumulative bus errors observed
    uint16_t rxOverrunCount;   // 0 on TIER_NEW (not exposed by driver)
} CANControllerStats;

// Speed table entry – used by TIER_V1 and TIER_V2 only.
// TIER_NEW uses bitrate integers directly.
#if !defined(TWAI_TIER_NEW)
typedef struct {
    twai_timing_config_t cfg;
    uint32_t             speed;
} VALID_TIMING;
#endif

class ESP32CAN : public CAN_COMMON
{
public:
    ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin, uint8_t busNumber = 0);
    ESP32CAN();

    // ── CAN_COMMON overrides ────────────────────────────────────────────────
    uint32_t init(uint32_t ul_baudrate)     override;
    uint32_t beginAutoSpeed()               override;
    uint32_t set_baudrate(uint32_t ul_baudrate) override;
    void     setListenOnlyMode(bool state)  override;
    void     setNoACKMode(bool state);
    bool     sendFrame(CAN_FRAME &txFrame)  override;
    bool     rx_avail()                     override;
    uint16_t available()                    override;
    uint32_t get_rx_buff(CAN_FRAME &msg)    override;

    // ── Public helpers ──────────────────────────────────────────────────────
    void enable();
    void disable();
    void setCANPins(gpio_num_t rxPin, gpio_num_t txPin);
    void setRXBufferSize(int newSize);
    void setTXBufferSize(int newSize);
    // Read controller diagnostic counters. Returns false if the driver
    // isn't initialized or the underlying IDF call fails. On success,
    // `stats` is populated; on failure its contents are undefined.
    // Safe to call from any task; no locking required.
    bool getControllerStats(CANControllerStats &stats);
#if defined(TWAI_TIER_NEW) || defined(TWAI_TIER_V2)
    void resetIfStale(uint32_t stallMs = 2000);
#endif

    // Shared queues (accessed by static callbacks/tasks)
    QueueHandle_t callbackQueue = nullptr;
    QueueHandle_t rx_queue      = nullptr;
#if defined(TWAI_TIER_NEW) && defined(ESP32CAN_HAS_FD)
    // FD frames (fdf=1) never go to rx_queue / callbackQueue, which only hold an
    // 8-byte CAN_FRAME. They pass the same software filter table and go to
    // callbackQueueFD (FD callbacks / listeners) or rx_queue_fd (polling).
    QueueHandle_t rx_queue_fd     = nullptr;
    QueueHandle_t callbackQueueFD = nullptr;
#endif

    // Traffic watchdog counter (reset by RX path)
    volatile int cyclesSinceTraffic = 0;
    volatile bool readyForTraffic   = false;

    // ── Task handles (TIER_V1 / TIER_V2) ───────────────────────────────────
#if !defined(TWAI_TIER_NEW)
    TaskHandle_t CAN_WatchDog_Builtin_handler = nullptr;
    TaskHandle_t task_CAN_handler             = nullptr;
    TaskHandle_t task_LowLevelRX_handler      = nullptr;
#else
    // TIER_NEW: only the callback-dispatch task is needed
    TaskHandle_t task_CAN_handler             = nullptr;
    // Watchdog task still used for bus-off recovery polling
    TaskHandle_t CAN_WatchDog_Builtin_handler = nullptr;
#endif

    // Internal frame processing (used by static helpers)
    bool processFrame(CAN_FRAME &msg);

// CAN-FD support (TIER_NEW on FD-capable chips only, see ESP32CAN_HAS_FD)
#if defined(TWAI_TIER_NEW) && defined(ESP32CAN_HAS_FD)
    // sendFrameFD transmits an FD frame. BRS (bit-rate switch) defaults to on
    // whenever a data baudrate is configured. With setPerFrameBRS(true), BRS is
    // taken per frame from txFrame.rrs instead (non-zero = BRS on) — useful for
    // bridges that must reproduce the received BRS bit (get_rx_buffFD reports
    // it in rrs). BRS always requires a configured data baudrate.
    bool sendFrameFD(CAN_FRAME_FD &txFrame) override;

    // Receive a full FD frame from rx_queue_fd. Populates the 64-byte payload,
    // fdMode, and carries the received BRS bit in msg.rrs. Returns true if a
    // frame was dequeued.
    uint32_t get_rx_buffFD(CAN_FRAME_FD &msg) override;

    // Number of FD frames waiting in rx_queue_fd.
    uint16_t availableFD();

    // Call this before init() to enable the FD data phase
    void setDataBaudrate(uint32_t dataBaudrate) { _dataBaudrate = dataBaudrate; }

    // See sendFrameFD. Default false (BRS on for every FD frame).
    void setPerFrameBRS(bool enable) { _perFrameBRS = enable; }

    // can_common FD entry points, so CAN0.beginFD(nominal, data) works too.
    uint32_t initFD(uint32_t nominalRate, uint32_t dataRate) override;
    uint32_t set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed) override;
#endif

protected:
    bool initializedResources = false;

private:
    // ── Filter table ────────────────────────────────────────────────────────
    ESP32_FILTER filters[BI_NUM_FILTERS];
    int rxBufferSize = BI_RX_BUFFER_SIZE;

    // ── Mode flags ──────────────────────────────────────────────────────────
    bool listenOnly = false;
    bool noACK      = false;

    // ── Stored pin / bus config ─────────────────────────────────────────────
    gpio_num_t _rxPin   = GPIO_NUM_16;
    gpio_num_t _txPin   = GPIO_NUM_17;
    uint8_t    _busNum  = 0;

    // ── IDF-tier driver state ───────────────────────────────────────────────
#if defined(TWAI_TIER_NEW)
    twai_node_handle_t node_handle = nullptr;
    uint32_t _baudrate  = 500000; // stored so enable() can re-use it
    uint32_t _dataBaudrate = 0;   // non-zero enables CAN-FD BRS
    uint32_t _txQueueDepth = BI_TX_BUFFER_SIZE;
    bool     _perFrameBRS  = false;
    // The driver reports "disabled" and "bus-off" as the same state, so track
    // whether *we* enabled the node to tell the two apart.
    volatile bool _nodeEnabled    = false;
    volatile bool _busOffPending  = false; // set by onStateChange, handled by watchdog

    // TX frame storage: must outlive twai_node_transmit() until on_tx_done.
    twai_frame_t      txFrames[BI_TX_SLOTS];
    uint8_t           txBufs[BI_TX_SLOTS][BI_TX_PAYLOAD_MAX];
    volatile uint32_t txBusyMask = 0;
    portMUX_TYPE      txMux;            // initialised in constructors
    bool _queueTx(uint32_t id, bool ext, bool rtr, bool fdf, bool brs,
                  const uint8_t *data, uint8_t len);
#elif defined(TWAI_TIER_V2)
    twai_handle_t bus_handle = nullptr;
    volatile bool _recovering = false;   // bus-off recovery in progress
    twai_general_config_t twai_general_cfg;
    twai_timing_config_t  twai_speed_cfg;
    twai_filter_config_t  twai_filters_cfg;
#else
    twai_general_config_t twai_general_cfg;
    twai_timing_config_t  twai_speed_cfg;
    twai_filter_config_t  twai_filters_cfg;
    volatile bool _recovering = false;   // bus-off recovery in progress
#endif

    // ── Internal helpers ────────────────────────────────────────────────────
    void _init();
    int  _setFilter(uint32_t id, uint32_t mask, bool extended)                       override;
    int  _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) override;
    void sendCallback(CAN_FRAME *frame);
#if defined(TWAI_TIER_NEW) && defined(ESP32CAN_HAS_FD)
    void sendCallbackFD(CAN_FRAME_FD *frame);
#endif

    // ── Static task / callback functions ───────────────────────────────────
    static void task_CAN(void *pvParameters);

#if !defined(TWAI_TIER_NEW)
    // Legacy tiers: blocking-receive task + watchdog task
    static void task_LowLevelRX(void *pvParameters);
    static void CAN_WatchDog_Builtin(void *pvParameters);
#else
    // New tier: ISR callback for RX, watchdog still polls node info
    static bool onRxDone(twai_node_handle_t handle,
                         const twai_rx_done_event_data_t *edata,
                         void *user_ctx);
    static bool onTxDone(twai_node_handle_t handle,
                         const twai_tx_done_event_data_t *edata,
                         void *user_ctx);
    static bool onError(twai_node_handle_t handle,
                        const twai_error_event_data_t *edata,
                        void *user_ctx);
    static void CAN_WatchDog_Builtin(void *pvParameters);
    static bool onStateChange(twai_node_handle_t handle,
                          const twai_state_change_event_data_t *edata,
                          void *user_ctx);
#endif
    volatile bool trafficEverSeen = false;
};
