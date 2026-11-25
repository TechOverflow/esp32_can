#include "esp32_can.h"

//The objects in here are defined weak now. If you're using an EVTV board you need to do nothing.
//If you're using something else that uses different pins then just go ahead and redefine 
//the objects with whichever pins you need. 

//Set these to the proper pin numbers for you board. Set by default to correct for EVTV ESP32-Due
             //rxpin       txpin
ESP32CAN __attribute__((weak)) CAN0(GPIO_NUM_16, GPIO_NUM_17, 0) ;

#if defined(ESP32CAN_HAS_CAN1)
    ESP32CAN __attribute__((weak)) CAN1(GPIO_NUM_18, GPIO_NUM_19, 1);
#endif

#if defined(ESP32CAN_HAS_CAN2)
    // No sensible default pins on the P4 – call CAN2.setCANPins() before begin()
    ESP32CAN __attribute__((weak)) CAN2(GPIO_NUM_NC, GPIO_NUM_NC, 2);
#endif
