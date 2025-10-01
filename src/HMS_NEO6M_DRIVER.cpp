#include "HMS_NEO6M_DRIVER.h"

#if defined(HMS_NEO6M_LOGGER_ENABLED)
    #include "ChronoLog.h"
    ChronoLogger neo6mLogger("HMS_NEO6M", CHRONOLOG_LEVEL_DEBUG);
#endif

HMS_NEO6M::HMS_NEO6M() : payload{0}, respBuffer{0}, checksum{0} {

}

HMS_NEO6M::~HMS_NEO6M() {

}

#if defined(HMS_NEO6M_PLATFORM_ARDUINO)
    HMS_NEO6M_Status HMS_NEO6M::begin(Stream &serial, uint32_t baudrate = 115200) {
        return HMS_NEO6M_OK;
    }

    void sendUBXMessage(const uint8_t *msg, uint8_t len) {
        // Send UBX message via serial
    }
#elif defined(HMS_NEO6M_PLATFORM_ESP_IDF)
    HMS_NEO6M_Status HMS_NEO6M::begin(Serial &serial, uint32_t baudrate = 115200) {
        return HMS_NEO6M_OK;
    }
#elif defined(HMS_NEO6M_PLATFORM_ZEPHYR)
    HMS_NEO6M_Status HMS_NEO6M::begin(const struct device *uart, uint32_t baudrate = 115200) {
        // Initialize UART with the specified baudrate
        return HMS_NEO6M_OK;
    }
#elif defined(HMS_NEO6M_PLATFORM_STM32_HAL)
    HMS_NEO6M_Status HMS_NEO6M::begin(UART_HandleTypeDef *huart, uint32_t baudrate = 115200) {
        neo6mUart = huart;
        neo6mUart->Init.BaudRate = baudrate;
        HAL_UART_Init(neo6mUart);
        neo6mDelay(10000);

        return init();
    }

    HMS_NEO6M_Status HMS_NEO6M::sendUBXMessage(const uint8_t *msg, uint8_t len) {
        HAL_StatusTypeDef status = HAL_UART_Transmit(neo6mUart, msg, len, HAL_MAX_DELAY);

        uint8_t respLen = 0;
        receiveUBXResponse(respBuffer, respLen, HMS_NEO6M_RESPONSE_TIMEOUT);
        if(respLen > 0) { 
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                logHex(respBuffer, respLen);
                neo6mLogger.debug("Received UBX response of length %d", respLen);
            #endif
            gps.parse(gps.respBuffer, gps.respLen);
        } else {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.logError("No UBX response received");
            #endif
        }
        return (status == HAL_OK) ? HMS_NEO6M_OK : HMS_NEO6M_ERROR;
    }

    HMS_NEO6M_Status HMS_NEO6M::receiveUBXResponse(uint8_t *respBuffer, uint8_t &respLen, uint32_t timeout) {
        HAL_StatusTypeDef status = HAL_UART_Receive(
            neo6mUart, header.header, HMS_NEO6M_UBX_HEADER_SIZE, timeout
        );

        if(header.header[0] != HMS_NEO6M_UBX_SYNC_CHAR_1 || header.header[1] != HMS_NEO6M_UBX_SYNC_CHAR_2) {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.logError("Invalid UBX header sync");
            #endif
            return HMS_NEO6M_ERROR;
        }

        header.length = header.header[4] | (header.header[5] << 8);

        if (header.length + 8 > HMS_NEO6M_UBX_RESPBUF_SIZE) return HMS_NEO6M_ERROR;

        memcpy(respBuffer, header.header, HMS_NEO6M_UBX_HEADER_SIZE);

        status = HAL_UART_Receive(neo6mUart, respBuffer + HMS_NEO6M_UBX_HEADER_SIZE, header.length + 2 , timeout);  // read payload
        respLen = header.length + 8;

        return (status == HAL_OK) ? HMS_NEO6M_OK : HMS_NEO6M_ERROR;
    }
#endif


HMS_NEO6M_Status HMS_NEO6M::init() {
    if(configureUART() != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to Initialize UART");
        #endif
        return HMS_NEO6M_ERROR;
    }

    if(getFix() != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to acquire GPS Fix");
        #endif
        return HMS_NEO6M_ERROR;
    }

    if(fetchCoordinates() != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to fetch coordinates");
        #endif
        return HMS_NEO6M_ERROR;
    }

    if(fetchTimeEUTCC() != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to fetch UTC time");
        #endif
        return HMS_NEO6M_ERROR;
    }

    if(putToSleep() != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to put NEO6M to sleep");
        #endif
        return HMS_NEO6M_ERROR;
    }

    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::reset() {
    memset(payload, 0, sizeof(payload));
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_CFG;
    payload[3] = HMS_NEO6M_UBX_CFG_RST;
    payload[4] = 0x04;                      // length LSB
    payload[5] = 0x00;                      // length MSB
    payload[6] = 0x00;                      // navBbrMask LSB
    payload[7] = 0x00;                      // navBbrMask MSB
    payload[8] = 0x02;                      // resetMode (0x02 = hot start)
    payload[9] = 0x00;                      // reserved

    makeChecksum(&payload[2], 8);
    payload[10] = checksum[0];
    payload[11] = checksum[1];

    if(sendUBXMessage(payload, 12) != HMS_NEO6M_OK ) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to reset NEO6M");
        #endif
        return HMS_NEO6M_ERROR;
    }
    
    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("NEO6M has been reset");
    #endif
    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::getFix() {
    for (uint8_t retry = 0; retry < maxRetries; retry++) {
        // NavigationCommand(HEADER_SYNC_1, HEADER_SYNC_2, NAVIGATION_CLASS, NAVIGATION_SOLUTION_ID);
        neo6mDelay(500);

        if (commandParams.gpsFix >= 2 && commandParams.NumberofSatllites > 0) {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("Fix acquired with %d satellites", commandParams.NumberofSatllites);
            #endif
            return HMS_NEO6M_OK;
        }

        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.debug("Waiting for the Fix... Retry %d/%d", retry + 1, maxRetries);
        #endif
        neo6mDelay(retryInterval);
    }

    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.logError("No fix after %d retries", maxRetries);
    #endif
    return HMS_NEO6M_ERROR;

}

HMS_NEO6M_Status HMS_NEO6M::wakeup() {
    memset(payload, 0, sizeof(payload));
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_RXM;
    payload[3] = HMS_NEO6M_UBX_RXM_PMREQ;
    payload[4] = 0x08;                      // length LSB (8 bytes)
    payload[5] = 0x00;                      // length MSB

    payload[6]  = 0x00;                     // duration LSB
    payload[7]  = 0x00;
    payload[8]  = 0x00;
    payload[9]  = 0x00;                     // duration MSB
    payload[10] = 0x01;                     // flags LSB = normal mode (wake up from backup)
    payload[11] = 0x00;
    payload[12] = 0x00;
    payload[13] = 0x00;                     // flags MSB

    makeChecksum(&payload[2], 12);
    payload[14] = checksum[0];
    payload[15] = checksum[1];

    if(sendUBXMessage(payload, 16) != HMS_NEO6M_OK ) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to wake NEO6M from sleep");
        #endif
        return HMS_NEO6M_ERROR;
    }
}

HMS_NEO6M_Status HMS_NEO6M::putToSleep() {
    memset(payload, 0, sizeof(payload));
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_RXM;
    payload[3] = HMS_NEO6M_UBX_RXM_PMREQ;
    payload[4] = 0x08;                          // length LSB (8 bytes)
    payload[5] = 0x00;                          // length MSB

    payload[6]  = 0x00;                         // duration LSB
    payload[7]  = 0x00;
    payload[8]  = 0x00;
    payload[9]  = 0x00;                         // duration MSB
    payload[10] = 0x02;                         // flags LSB (backup mode)
    payload[11] = 0x00;
    payload[12] = 0x00;
    payload[13] = 0x00;                         // flags MSB

    makeChecksum(&payload[2], 12);
    payload[14] = checksum[0];
    payload[15] = checksum[1];

    if(sendUBXMessage(payload, 16) != HMS_NEO6M_OK ) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to put NEO6M to sleep");
        #endif
        return HMS_NEO6M_ERROR;
    }
    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("NEO6M is now in sleep mode");
    #endif
    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::configureUART() {
    memset(payload, 0, sizeof(payload));
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_CFG;               // msgClass = CFG
    payload[3] = HMS_NEO6M_UBX_CFG_PRT;                 // id = PRT
    payload[4]  = 0x14;                                 // length LSB = 20
    payload[5]  = 0x00;                                 // length MSB
    payload[6]  = 0x01;                                 // portID = 1 (UART1)
    payload[7]  = 0x00;                                 // reserved
    payload[8]  = 0x00;
    payload[9]  = 0x00;
    payload[10] = 0xD0;                                 // mode LSB   
    payload[11] = 0x08;
    payload[12] = 0x00;
    payload[13] = 0x00;
    payload[14] = 0x80;                                 // baudrate LSB
    payload[15] = 0x25;
    payload[16] = 0x00;
    payload[17] = 0x00;                                 // baudrate MSB
    payload[18] = 0x01;                                 // inProtoMask = UBX
    payload[19] = 0x00;
    payload[20] = 0x01;                                 // outProtoMask = UBX
    payload[21] = 0x00;
    payload[22] = 0x00;                                 // flags
    payload[23] = 0x00;
    payload[24] = 0x00;                                 // reserved
    payload[25] = 0x00;

    makeChecksum(&payload[2], 24);
    payload[26] = checksum[0];
    payload[27] = checksum[1];

    if( sendUBXMessage(payload, 28) != HMS_NEO6M_OK ) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to configure UART");
        #endif
        return HMS_NEO6M_ERROR;
    }
    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::fetchTimeEUTCC() {
    memset(payload, 0, sizeof(payload));
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_NAV;
    payload[3] = HMS_NEO6M_UBX_NAV_TIMEUTC;
    payload[4] = 0x00;   
    payload[5] = 0x00;  

    makeChecksum(&payload[2], 4);
    payload[6] = checksum[0];
    payload[7] = checksum[1];

    if(sendUBXMessage(payload, 8) != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to request UTC time");
        #endif
        return HMS_NEO6M_ERROR;
    }
    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::fetchCoordinates() {
    memset(payload, 0, sizeof(payload));
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_NAV;
    payload[3] = HMS_NEO6M_UBX_NAV_POSLLH;
    payload[4] = 0x00;   
    payload[5] = 0x00;   

    makeChecksum(&payload[2], 4);
    payload[6] = checksum[0];
    payload[7] = checksum[1];

    if(sendUBXMessage(payload, 8) != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to request coordinates");
        #endif
        return HMS_NEO6M_ERROR;
    }
    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::fetchNaveMetaInfo() {
    payload[0] = HMS_NEO6M_UBX_SYNC_CHAR_1;
    payload[1] = HMS_NEO6M_UBX_SYNC_CHAR_2;
    payload[2] = HMS_NEO6M_UBX_CLASS_NAV;
    payload[3] = HMS_NEO6M_UBX_NAV_SOL;
    payload[4] = 0x00;                          // Length LSB
    payload[5] = 0x00;                          // Length MSB
    payload[6] = 0x00;                          // Checksum LSB
    payload[7] = 0x00;                          // Checksum MSB

    makeChecksum(&payload[2], 4);
    payload[6] = checksum[0];
    payload[7] = checksum[1];
    if(sendUBXMessage(payload, 8) != HMS_NEO6M_OK) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Failed to request navigation meta info");
        #endif
        return HMS_NEO6M_ERROR;
    }
    return HMS_NEO6M_OK;
}

HMS_NEO6M_Status HMS_NEO6M::parseResponse(const uint8_t *buffer, uint8_t len) {
    uint8_t msgId       = buffer[3];
    uint8_t msgClass    = buffer[2];
    uint8_t payloadLen  = buffer[4] | (buffer[5] << 8);

    if (len != payloadLen + 8) {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.logError("Invalid UBX payload length");
        #endif
        return HMS_NEO6M_ERROR;
    }
    

    if(msgClass == HMS_NEO6M_UBX_CLASS_ACK) {
        gpsCheck(buffer , msgClass, msgId, len);
        return HMS_NEO6M_OK;
    }

    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("Parsing UBX message Class: 0x%02X, ID: 0x%02X, Length: %d", msgClass, msgId, payloadLen);
    #endif

    uint16_t msgType = (msgClass << 8) | msgId;

    switch (msgType) {
        case (HMS_NEO6M_UBX_CLASS_NAV << 8) | HMS_NEO6M_UBX_NAV_SOL: 
            parseNaveMetaInfo(buffer); 
            break;
        case (HMS_NEO6M_UBX_CLASS_NAV << 8) | HMS_NEO6M_UBX_NAV_POSLLH:
            parseCoordinates(buffer); 
            break;
        case (HMS_NEO6M_UBX_CLASS_NAV << 8) | HMS_NEO6M_UBX_NAV_TIMEUTC:
            parseTimeUTC(buffer);
        break;
        default:
            // sprintf(log_buf, "Unknown message type: 0x%04X\r\n", msg_type);
            // UART_LOGS(log_buf);
            return HMS_NEO6M_ERROR;
            break;
    }
    return HMS_NEO6M_OK;
}

void HMS_NEO6M::neo6mDelay(uint32_t delay) {
    #if defined(HMS_NEO6M_PLATFORM_ARDUINO)
        delay(delay);
    #elif defined(HMS_NEO6M_PLATFORM_ESP_IDF)
        vTaskDelay(delay / portTICK_PERIOD_MS);
    #elif defined(HMS_NEO6M_PLATFORM_ZEPHYR)
        k_msleep(delay);
    #elif defined(HMS_NEO6M_PLATFORM_STM32_HAL) && !defined(CHRONOLOG_STM32_FREERTOS)
        HAL_Delay(delay);
    #elif defined(HMS_NEO6M_PLATFORM_STM32_HAL) && defined(CHRONOLOG_STM32_FREERTOS)
        osDelay(delay);
    #endif
}

void HMS_NEO6M::parseTimeUTC(const uint8_t* buffer) {
    memcpy(&timeData.timeAccuracyEstimate,&buffer[10],sizeof(timeData.timeAccuracyEstimate));
    memcpy(&timeData.nanoSeconds,&buffer[14],sizeof(timeData.nanoSeconds));
    memcpy(&timeData.year,&buffer[18],sizeof(timeData.year));
    memcpy(&timeData.month,&buffer[20],sizeof(timeData.month));
    memcpy(&timeData.day,&buffer[21],sizeof(timeData.day));
    memcpy(&timeData.hour,&buffer[22],sizeof(timeData.hour));
    memcpy(&timeData.minute,&buffer[23],sizeof(timeData.minute));
    memcpy(&timeData.second,&buffer[24],sizeof(timeData.second));
    timeData.valid = buffer[25];

    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("Parsed UTC Time - %04d-%02d-%02d %02d:%02d:%02d (Nano: %d, Valid: 0x%02X)",
            timeData.year, timeData.month, timeData.day,
            timeData.hour, timeData.minute, timeData.second,
            timeData.nanoSeconds, timeData.valid
        );
    #endif
}

void HMS_NEO6M::parseCoordinates(const uint8_t* buffer) {
    memcpy(&navData.timeOfWeek, &buffer[6], sizeof(navData.timeOfWeek));
    memcpy(&navData.longitude, &buffer[10], sizeof(navData.longitude));
    memcpy(&navData.latitude, &buffer[14], sizeof(navData.latitude));
    memcpy(&navData.altitude, &buffer[18], sizeof(navData.altitude));
    memcpy(&navData.heightAboveSeaLevel, &buffer[22], sizeof(navData.heightAboveSeaLevel));
    memcpy(&navData.horizontalAccuracy, &buffer[26], sizeof(navData.horizontalAccuracy));
    memcpy(&navData.verticalAccuracy, &buffer[30], sizeof(navData.verticalAccuracy));

    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("Parsed Coordinates - Lat: %ld, Lon: %ld, Alt: %ld, Height: %ld, HorAcc: %ld, VerAcc: %ld",
            navData.latitude, navData.longitude, navData.altitude, navData.heightAboveSeaLevel, navData.horizontalAccuracy, navData.verticalAccuracy
        );
    #endif
}

void HMS_NEO6M::parseNaveMetaInfo(const uint8_t* buffer) {
    
    memcpy(&navMetaInfo.timeofFractional, &buffer[10], sizeof(navMetaInfo.timeofFractional));
    memcpy(&navMetaInfo.week, &buffer[14], sizeof(navMetaInfo.week));

    navMetaInfo.gpsFix = buffer[16];
    navMetaInfo.flags  = buffer[17];

    memcpy(&navMetaInfo.ecefx,&buffer[18],sizeof(navMetaInfo.ecefx));
    memcpy(&navMetaInfo.ecefy,&buffer[22],sizeof(navMetaInfo.ecefy));
    memcpy(&navMetaInfo.ecefz,&buffer[26],sizeof(navMetaInfo.ecefz));
    memcpy(&navMetaInfo.pacc,&buffer[30],sizeof(navMetaInfo.pacc));
    memcpy(&navMetaInfo.ecefvx,&buffer[34],sizeof(navMetaInfo.ecefvx));
    memcpy(&navMetaInfo.ecefvy,&buffer[38],sizeof(navMetaInfo.ecefvy));
    memcpy(&navMetaInfo.ecefvz,&buffer[42],sizeof(navMetaInfo.ecefvz));
    memcpy(&navMetaInfo.sacc,&buffer[46],sizeof(navMetaInfo.sacc));
    memcpy(&navMetaInfo.pdop,&buffer[50],sizeof(navMetaInfo.pdop));

    navMetaInfo.numberofSatellites = buffer[53];

    fixType = static_cast<HMS_NEO6M_FixType>(navMetaInfo.gpsFix);

    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("Parsed NAV-SOL - Fix: %d, Flags: 0x%02X, Satellites: %d, ECEF X: %ld, Y: %ld, Z: %ld",
            navMetaInfo.gpsFix, navMetaInfo.flags, navMetaInfo.numberofSatellites,
            navMetaInfo.ecefx, navMetaInfo.ecefy, navMetaInfo.ecefz
        );
    #endif
}

void HMS_NEO6M::makeChecksum(uint8_t *msg, uint8_t length) {
    memset(checksum, 0, HMS_NEO6M_UBX_CHECKSUM_SIZE);
    for (uint8_t i = 0; i < length; i++) {
        checksum[0] += msg[i];
        checksum[1] += checksum[0];
    }
}

void HMS_NEO6M::logHex(const uint8_t *data, uint8_t length) {
    uint8_t pos = 0; 
    char buff[HMS_NEO6M_UBX_RESPBUF_SIZE];
    for (uint8_t i = 0; i < length; i++) {
        pos += sprintf(buff + pos, "%02X ", data[i]);
    }
    #if defined(HMS_NEO6M_LOGGER_ENABLED)
        neo6mLogger.debug("Hex Dump: %s", buff);
    #endif
}

void HMS_NEO6M::gpsCheck(const uint8_t* buffer, uint8_t msgClass, uint8_t msgId,uint16_t len) {
    if (msgClass == HMS_NEO6M_UBX_CLASS_ACK && msgId == HMS_NEO6M_UBX_ACK_ACK) {
        if (len >= 8 && buffer[6] == HMS_NEO6M_UBX_CLASS_CFG && buffer[7] == HMS_NEO6M_UBX_CFG_PRT ) {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("GPS Detected -> ACK received for (Port Configuration)");
            #endif
        } else if (len >= 8 && buffer[6] == HMS_NEO6M_UBX_CLASS_CFG && buffer[7] == HMS_NEO6M_UBX_CFG_RST) {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("GPS Detected -> ACK received for (Reset)");
            #endif
        } else {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("GPS Detected -> ACK received for unknown command");
            #endif
        }
    } else if (msgClass == HMS_NEO6M_UBX_CLASS_ACK && msgId == HMS_NEO6M_UBX_ACK_NAK) {
        if (len >= 8 && buffer[6] == HMS_NEO6M_UBX_CLASS_CFG && buffer[7] == HMS_NEO6M_UBX_CFG_PRT) {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("GPS Detected but NAK received for CFG-PRT (Port Configuration)");
            #endif
        } else if (len >= 8 && buffer[6] == HMS_NEO6M_UBX_CLASS_CFG && buffer[7] == HMS_NEO6M_UBX_CFG_RST) {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("GPS Detected but NAK received for CFG-RST (Reset)");
            #endif
        } else {
            #if defined(HMS_NEO6M_LOGGER_ENABLED)
                neo6mLogger.debug("GPS Detected but NAK received for unknown command");
            #endif
        }
    } else {
        #if defined(HMS_NEO6M_LOGGER_ENABLED)
            neo6mLogger.debug("GPS not Detected: Invalid response");
        #endif
    }
}