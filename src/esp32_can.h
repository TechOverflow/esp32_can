#include "esp32_can_builtin.h"

extern ESP32CAN CAN0;
//Select which external chip you've got connected

// Multi-controller support needs the _v2 / esp_driver_twai APIs (IDF >= 5.2)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
  #if SOC_TWAI_CONTROLLER_NUM >= 2          // ESP32-C5, ESP32-C6, ESP32-P4
    extern ESP32CAN CAN1;
    #define ESP32CAN_HAS_CAN1 1
  #endif
  #if SOC_TWAI_CONTROLLER_NUM >= 3          // ESP32-P4
    extern ESP32CAN CAN2;
    #define ESP32CAN_HAS_CAN2 1
  #endif
#endif

extern volatile uint32_t biIntsCounter;
extern volatile uint32_t biReadFrames;

#define Can0 CAN0

#if defined(ESP32CAN_HAS_CAN1) || defined(USES_MCP2517FD) || defined(USES_MCP2515)
#define Can1 CAN1
#endif

#if defined(ESP32CAN_HAS_CAN2)
#define Can2 CAN2
#endif
