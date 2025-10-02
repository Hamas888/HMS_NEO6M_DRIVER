   /*
 ====================================================================================================
 * File:        HMS_NEO6M_Config.h
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Sep 23 2025
 * Brief:       This Package Provide NEO6M Driver Configuration.
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

#ifndef HMS_NEO6M_CONFIG_H
#define HMS_NEO6M_CONFIG_H

/*
  ┌─────────────────────────────────────────────────────────────────────┐
  │ Note:     Enable only if ChronoLog is included                      │
  │ Requires: ChronoLog library → https://github.com/Hamas888/ChronoLog │
  └─────────────────────────────────────────────────────────────────────┘
*/
#define HMS_NEO6M_DEBUG_ENABLED           0                             // Enable debug messages (1=enabled, 0=disabled)


// UBX Protocol Constants
#define HMS_NEO6M_UBX_SYNC_CHAR_1         0xB5
#define HMS_NEO6M_UBX_SYNC_CHAR_2         0x62

// UBX Message Classes
#define HMS_NEO6M_UBX_CLASS_NAV           0x01                          // Navigation Results
#define HMS_NEO6M_UBX_CLASS_RXM           0x02                          // Receiver Manager
#define HMS_NEO6M_UBX_CLASS_ACK           0x05                          // Ack/Nak Messages
#define HMS_NEO6M_UBX_CLASS_CFG           0x06                          // Configuration Input

// UBX Navigation Message IDs
#define HMS_NEO6M_UBX_NAV_SOL             0x06                          // Navigation Solution Information
#define HMS_NEO6M_UBX_NAV_POSLLH          0x02                          // Geodetic Position Solution
#define HMS_NEO6M_UBX_NAV_TIMEUTC         0x21                          // UTC Time Solution

// UBX Configuration Message IDs  
#define HMS_NEO6M_UBX_CFG_PRT             0x00                          // Port Configuration
#define HMS_NEO6M_UBX_CFG_RST             0x04                          // Reset Receiver/Clear Backup
#define HMS_NEO6M_UBX_CFG_PM2             0x3B                          // Extended Power Management

// UBX Receiver Manager Message IDs
#define HMS_NEO6M_UBX_RXM_PMREQ           0x41                          // Requests a Power Management task

// UBX Acknowledge Message IDs
#define HMS_NEO6M_UBX_ACK_ACK             0x01                          // Message Acknowledged
#define HMS_NEO6M_UBX_ACK_NAK             0x00                          // Message Not Acknowledged

// Other Constants
#define HMS_NEO6M_UBX_HEADER_SIZE         6
#define HMS_NEO6M_UBX_RESPBUF_SIZE        256
#define HMS_NEO6M_UBX_PAYLOAD_SIZE        128
#define HMS_NEO6M_RESPONSE_TIMEOUT        1200
#define HMS_NEO6M_UBX_CHECKSUM_SIZE       2

#define HMS_NEO6M_DEFAULT_MAX_RETRIES     5                             // Default maximum number of retries for operations
#define HMS_NEO6M_DEFAULT_RETRY_INTERVAL  2                             // Default retry interval in seconds

#define HMS_NEO6M_GEOFENCE_RADIUS_DEFAULT 100                          // Default geofence radius in meters

#endif // HMS_NEO6M_CONFIG_H