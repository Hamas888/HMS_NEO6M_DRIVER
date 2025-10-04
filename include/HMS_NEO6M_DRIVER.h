   /*
 ====================================================================================================
 * File:        HMS_NEO6M_DRIVER.h
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Oct 1 2025
 * Brief:       This Package Provide NEO6M Driver Library for Cross Platform (STM/ESP/nRF)
 * 
 ====================================================================================================
 * License: 
 * MIT License
 * 
 * Copyright (c) 2025 Hamas Saeed
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * For any inquiries, contact Hamas Saeed at hamasaeed@gmail.com
 *
 ====================================================================================================
 */

#ifndef HMS_NEO6M_DRIVER_H
#define HMS_NEO6M_DRIVER_H

#if defined(ARDUINO)                                                                                       // Platform detection
    #define HMS_NEO6M_PLATFORM_ARDUINO
#elif defined(ESP_PLATFORM)
    #define HMS_NEO6M_PLATFORM_ESP_IDF
#elif defined(__ZEPHYR__)
    #define HMS_NEO6M_PLATFORM_ZEPHYR
#elif defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || \
      defined(STM32F7) || defined(STM32G0) || defined(STM32G4) || defined(STM32H7) || \
      defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32L5) || \
      defined(STM32WB) || defined(STM32WL)
    #define HMS_NEO6M_PLATFORM_STM32_HAL
#endif

#if defined(HMS_NEO6M_PLATFORM_ARDUINO)
    #include <Arduino.h>
    #include <math.h>
#elif defined(HMS_NEO6M_PLATFORM_ESP_IDF)
    #include <math.h>
    #include <stdio.h>
    #include <stdint.h>
    #include <string.h>
#elif defined(HMS_NEO6M_PLATFORM_ZEPHYR)
    #include <stdio.h>
    #include <math.h>
    #include <stdint.h>
    #include <string.h>
    #include <zephyr/device.h>
    #include <zephyr/drivers/i2c.h>
#elif defined(HMS_NEO6M_PLATFORM_STM32_HAL)
    #include "main.h"
    #include <math.h>
    #include <stdio.h>
    #include <stdint.h>
    #include <string.h>
    #if defined(osCMSIS) || defined(FREERTOS)
        #define CHRONOLOG_STM32_FREERTOS
    #endif
#endif

// Define M_PI if not defined
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include "HMS_NEO6M_Config.h"

#if defined(HMS_NEO6M_DEBUG_ENABLED) && (HMS_NEO6M_DEBUG_ENABLED == 1)
    #define HMS_NEO6M_LOGGER_ENABLED
#endif

typedef enum {
    HMS_NEO6M_OK       = 0x00,
    HMS_NEO6M_BUSY     = 0x01,
    HMS_NEO6M_ERROR    = 0x02,
    HMS_NEO6M_TIMEOUT  = 0x03,
    HMS_NEO6M_NOT_FOUND= 0x04
} HMS_NEO6M_Status;

typedef enum {
    HMS_NEO6M_FIX_NO_FIX                = 0,
    HMS_NEO6M_FIX_2D_FIX                = 2,
    HMS_NEO6M_FIX_3D_FIX                = 3,
    HMS_NEO6M_FIX_UNKNOWN               = 255,
    HMS_NEO6M_FIX_TIME_ONLY             = 5,
    HMS_NEO6M_FIX_DEAD_RECKONING        = 1,
    HMS_NEO6M_FIX_GPS_DEAD_RECKONING    = 4
} HMS_NEO6M_FixType;

typedef struct {
    int32_t                     nanoSeconds;
    uint8_t                     month;
    uint8_t                     day;
    uint8_t                     hour;
    uint8_t                     minute;
    uint8_t                     second;
    uint8_t                     valid;
    uint16_t                    year;
    uint32_t                    timeAccuracyEstimate;
} HMS_NEO6M_Time;

typedef struct{
    uint8_t            header[HMS_NEO6M_UBX_HEADER_SIZE];
    uint8_t            id;  
    uint16_t           length;  
} HMS_NEO6M_Header;

typedef struct {
    int32_t            latitude;
    int32_t            longitude;
    int32_t            altitude;
    uint32_t           timeOfWeek;
    uint32_t           horizontalAccuracy;
    uint32_t           verticalAccuracy;
    uint32_t           heightAboveSeaLevel;
} HMS_NEO6M_NAV_Data;

typedef struct {
    bool             inGeofence;
    float            latitude;
    float            longitude;
    float            lastLatitude;
    float            lastLongitude;
    uint8_t          geofenceRadius;
} HMS_NEO6M_Location;

typedef struct {
    uint8_t          gpsFix;
    uint8_t          flags;
    uint8_t          numberofSatellites;
    uint32_t         ecefx;
    uint32_t         ecefy;
    uint32_t         ecefz;
    uint32_t         pacc;
    uint32_t         ecefvx;
    uint32_t         ecefvy;
    uint32_t         ecefvz;
    uint32_t         sacc;
    uint32_t         pdop;
    uint32_t         week;
    uint32_t         timeofFractional;
} HMS_NEO6M_NAV_MetaInfo;


class HMS_NEO6M {
    public:
        HMS_NEO6M();
        ~HMS_NEO6M();

        #if defined(HMS_NEO6M_PLATFORM_ARDUINO)
            HMS_NEO6M_Status begin(Stream &serial, uint32_t baudrate = 115200);
        #elif defined(HMS_NEO6M_PLATFORM_ESP_IDF)
            HMS_NEO6M_Status begin(Serial &serial, uint32_t baudrate = 115200);
        #elif defined(HMS_NEO6M_PLATFORM_ZEPHYR)
            HMS_NEO6M_Status begin(const struct device *uart, uint32_t baudrate = 115200);
        #elif defined(HMS_NEO6M_PLATFORM_STM32_HAL)
            HMS_NEO6M_Status begin(UART_HandleTypeDef *huart, uint32_t baudrate = 115200);
        #endif

        HMS_NEO6M_Status reset();
        HMS_NEO6M_Status getFix();
        HMS_NEO6M_Status wakeup();
        HMS_NEO6M_Status initCheck();
        HMS_NEO6M_Status putToSleep();
        HMS_NEO6M_Status configureUART();
        HMS_NEO6M_Status fetchTimeEUTCC();
        HMS_NEO6M_Status fetchCoordinates();
        HMS_NEO6M_Status fetchNaveMetaInfo();

        void setMaxRetries(uint8_t value)               { maxRetries = value;    }
        void setRetryInterval(uint8_t value)            { retryInterval = value; }
        void setGeofenceRadius(uint8_t value)           { geofenceRadius = value; }
        void setGeofenceCenter(float lat, float lon)    { location.lastLatitude = lat; location.lastLongitude = lon; }

        HMS_NEO6M_Time getTimeUTC() const               { return timeData;       }
        HMS_NEO6M_FixType getFixType() const            { return fixType;        }
        HMS_NEO6M_NAV_Data getNavData() const           { return navData;        }
        HMS_NEO6M_Location getLocation() const          { return location;       }
        HMS_NEO6M_NAV_MetaInfo getNavMetaInfo() const   { return navMetaInfo;    }
        uint8_t getGeofenceRadius() const               { return geofenceRadius; }
        bool isInGeofence() const                       { return location.inGeofence; }

    private:
        #if defined(HMS_NEO6M_PLATFORM_ARDUINO)
            Stream *neo6mSerial = nullptr;
        #elif defined(HMS_NEO6M_PLATFORM_ESP_IDF)
            Serial *neo6mSerial = nullptr;
        #elif defined(HMS_NEO6M_PLATFORM_ZEPHYR)
            const struct device *neo6mDevice = nullptr;
        #elif defined(HMS_NEO6M_PLATFORM_STM32_HAL)
            UART_HandleTypeDef *neo6mUart = nullptr;
        #endif
        bool                    isAwake;
        bool                    isInitialized;
        uint8_t                 geofenceRadius;
        uint8_t                 maxRetries;
        uint8_t                 retryInterval;
        uint8_t                 payload[HMS_NEO6M_UBX_PAYLOAD_SIZE];
        uint8_t                 checksum[HMS_NEO6M_UBX_CHECKSUM_SIZE];
        uint8_t                 respBuffer[HMS_NEO6M_UBX_RESPBUF_SIZE];
        HMS_NEO6M_Time          timeData;
        HMS_NEO6M_Header        header;
        HMS_NEO6M_FixType       fixType;
        HMS_NEO6M_NAV_Data      navData;
        HMS_NEO6M_Location      location;
        HMS_NEO6M_NAV_MetaInfo  navMetaInfo;

        HMS_NEO6M_Status init();
        HMS_NEO6M_Status sendUBXMessage(const uint8_t *msg, uint8_t len);
        HMS_NEO6M_Status parseResponse(const uint8_t *buffer, uint8_t len);
        HMS_NEO6M_Status receiveUBXResponse(uint8_t *respBuffer, uint8_t &respLen, uint32_t timeout);

        void neo6mDelay(uint32_t delay);
        void parseTimeUTC(const uint8_t* buffer);
        void parseCoordinates(const uint8_t* buffer);
        void parseNaveMetaInfo(const uint8_t* buffer);
        void makeChecksum(uint8_t *msg, uint8_t length);
        void logHex(const uint8_t *data, uint8_t length);
        void gpsCheck(const uint8_t* buffer, uint8_t msgClass, uint8_t msgId,uint16_t len);
        
        void updateGeofenceStatus();
        float calculateDistance(float lat1, float lon1, float lat2, float lon2);
        bool isWithinGeofence(float lat, float lon, float centerLat, float centerLon, float radiusMeters);
};

#endif // HMS_NEO6M_DRIVER_H