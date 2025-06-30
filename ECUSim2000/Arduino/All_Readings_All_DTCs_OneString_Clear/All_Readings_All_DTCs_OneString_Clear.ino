#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

#define OBD_REQUEST_ID 0x7DF
#define OBD_RESPONSE_ID_ECM 0x7E8
#define OBD_RESPONSE_ID_TCM 0x7E9
#define OBD_RESPONSE_ID_ABS 0x7EA

#define PID_COOLANT_TEMP 0x05
#define PID_ENGINE_RPM   0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_TIRE_PRESSURE 0x14
#define PID_MAF          0x10
#define PID_DTC          0x03
#define PID_PENDING_DTC  0x07
#define PID_PERMANENT_DTC 0x0A
#define PID_CLEAR_DTC    0x04

void setup() {
  Serial.begin(9600);
  if (!Canbus.init(CANSPEED_500)) {
    Serial.println("CAN Bus Init Failed");
    while (1);
  }
  Serial.println("1: Read all 5 sensor values\n2: Get all DTCs\n3: Clear all DTCs");
}

void loop() {
  if (Serial.available()) {
    int selection = Serial.parseInt();
    while (Serial.available()) Serial.read();

    if (selection == 1) {
      requestPID(0x01, PID_COOLANT_TEMP);
      requestPID(0x01, PID_ENGINE_RPM);
      requestPID(0x01, PID_VEHICLE_SPEED);
      requestPID(0x01, PID_TIRE_PRESSURE);
      requestPID(0x01, PID_MAF);
    } else if (selection == 2) {
      String allDTCs = "";
      allDTCs += collectDTCs(PID_DTC, OBD_RESPONSE_ID_ECM, "ECM Stored");
      delay(100);
      allDTCs += collectDTCs(PID_PENDING_DTC, OBD_RESPONSE_ID_ECM, "ECM Pending");
      delay(100);
      allDTCs += collectDTCs(PID_PERMANENT_DTC, OBD_RESPONSE_ID_ECM, "ECM Permanent");
      delay(100);
      allDTCs += collectDTCs(PID_PENDING_DTC, OBD_RESPONSE_ID_TCM, "TCM Pending");
      delay(100);
      allDTCs += collectDTCs(PID_PENDING_DTC, OBD_RESPONSE_ID_ABS, "ABS Pending");
      if (allDTCs.endsWith(",")) allDTCs.remove(allDTCs.length() - 1);
      allDTCs = removeDuplicateDTCs(allDTCs);
      Serial.println("All DTCs: " + allDTCs);
    } else if (selection == 3) {
      clearDTCs();
    }
    Serial.println("1: Read all 5 sensor values\n2: Get all DTCs\n3: Clear all DTCs");
  }
}

void requestPID(uint8_t mode, uint8_t pid) {
  tCAN message;
  message.id = OBD_REQUEST_ID;
  message.header.rtr = 0;
  message.header.length = 8;
  message.data[0] = 0x02;
  message.data[1] = mode;
  message.data[2] = pid;
  memset(&message.data[3], 0, 5);

  if (!mcp2515_send_message(&message)) {
    Serial.print("Failed to send PID 0x"); Serial.println(pid, HEX);
    return;
  }

  tCAN response;
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (mcp2515_check_message() && mcp2515_get_message(&response) && response.id == OBD_RESPONSE_ID_ECM && response.data[1] == (mode | 0x40) && response.data[2] == pid) {
      int32_t value = 0;
      switch (pid) {
        case PID_COOLANT_TEMP:
          value = (int32_t)(response.data[3]) - 40;
          break;
        case PID_ENGINE_RPM:
          value = (((uint16_t)response.data[3] << 8) | response.data[4]) / 4;
          break;
        case PID_VEHICLE_SPEED:
          value = response.data[3];
          break;
        case PID_TIRE_PRESSURE:
          value = 15 + ((int32_t)response.data[3] * 27 / 255);
          break;
        case PID_MAF:
          value = (((uint16_t)response.data[3] << 8) | response.data[4]) / 100;
          break;
      }
      Serial.print("PID 0x"); Serial.print(pid, HEX); Serial.print(": "); Serial.println(value);
      return;
    }
  }
  Serial.print("No response for PID 0x"); Serial.println(pid, HEX);
}

String collectDTCs(uint8_t mode, uint16_t respID, String moduleName) {
  tCAN msg;
  msg.id = OBD_REQUEST_ID;
  msg.header.rtr = 0;
  msg.header.length = 8;
  msg.data[0] = 0x01;
  msg.data[1] = mode;
  memset(&msg.data[2], 0, 6);

  if (!mcp2515_send_message(&msg)) {
    Serial.println("Failed to send request for " + moduleName);
    return "";
  }

  String dtcString = "";
  tCAN response;
  unsigned long start = millis();
  bool isFirstFrame = true;
  int totalBytes = 0;
  int processedBytes = 0;

  while (millis() - start < 2000) {
    if (mcp2515_check_message() && mcp2515_get_message(&response) && response.id == respID) {
      if (isFirstFrame && response.data[0] == 0x10) {
        totalBytes = ((response.data[1] << 8) | response.data[2]) - 2;
        int startIdx = 4;
        int len = response.header.length;
        for (int i = startIdx; i + 1 < len; i += 2) {
          uint8_t A = response.data[i];
          uint8_t B = response.data[i + 1];
          if (A == 0 && B == 0) continue;
          char dtc[6];
          char type;
          switch (A >> 6) {
            case 0: type = 'P'; break;
            case 1: type = 'C'; break;
            case 2: type = 'B'; break;
            case 3: type = 'U'; break;
          }
          sprintf(dtc, "%c%02X%02X", type, A & 0x3F, B);
          dtcString += String(dtc) + ",";
          processedBytes += 2;
        }
        isFirstFrame = false;
        tCAN flowControl;
        flowControl.id = OBD_REQUEST_ID;
        flowControl.header.rtr = 0;
        flowControl.header.length = 8;
        flowControl.data[0] = 0x30;
        flowControl.data[1] = 0x00;
        flowControl.data[2] = 0x00;
        memset(&flowControl.data[3], 0, 5);
        mcp2515_send_message(&flowControl);
      } else if (!isFirstFrame && response.data[0] >= 0x21 && response.data[0] <= 0x2F) {
        int startIdx = 1;
        int len = response.header.length;
        for (int i = startIdx; i + 1 < len && processedBytes < totalBytes; i += 2) {
          uint8_t A = response.data[i];
          uint8_t B = response.data[i + 1];
          if (A == 0 && B == 0) continue;
          char dtc[6];
          char type;
          switch (A >> 6) {
            case 0: type = 'P'; break;
            case 1: type = 'C'; break;
            case 2: type = 'B'; break;
            case 3: type = 'U'; break;
          }
          sprintf(dtc, "%c%02X%02X", type, A & 0x3F, B);
          dtcString += String(dtc) + ",";
          processedBytes += 2;
        }
      } else if (response.data[1] == (mode | 0x40)) {
        int startIdx = 3;
        int len = response.header.length;
        for (int i = startIdx; i + 1 < len; i += 2) {
          uint8_t A = response.data[i];
          uint8_t B = response.data[i + 1];
          if (A == 0 && B == 0) continue;
          char dtc[6];
          char type;
          switch (A >> 6) {
            case 0: type = 'P'; break;
            case 1: type = 'C'; break;
            case 2: type = 'B'; break;
            case 3: type = 'U'; break;
          }
          sprintf(dtc, "%c%02X%02X", type, A & 0x3F, B);
          dtcString += String(dtc) + ",";
        }
        break;
      }
    }
  }
  if (dtcString.length() > 0) {
    Serial.println(moduleName + " DTCs: " + dtcString);
  } else {
    Serial.println(moduleName + ": No DTCs or no response");
  }
  return dtcString;
}

String removeDuplicateDTCs(String dtcList) {
  String uniqueDTCs = "";
  int start = 0;
  int comma;
  String dtcs[20];
  int dtcCount = 0;

  while ((comma = dtcList.indexOf(',', start)) != -1) {
    String dtc = dtcList.substring(start, comma);
    if (dtc.length() > 0) {
      bool isDuplicate = false;
      for (int i = 0; i < dtcCount; i++) {
        if (dtcs[i] == dtc) {
          isDuplicate = true;
          break;
        }
      }
      if (!isDuplicate) {
        dtcs[dtcCount++] = dtc;
        uniqueDTCs += dtc + ",";
      }
    }
    start = comma + 1;
  }
  String dtc = dtcList.substring(start);
  if (dtc.length() > 0) {
    bool isDuplicate = false;
    for (int i = 0; i < dtcCount; i++) {
      if (dtcs[i] == dtc) {
        isDuplicate = true;
        break;
      }
    }
    if (!isDuplicate) {
      dtcs[dtcCount++] = dtc;
      uniqueDTCs += dtc;
    }
  }
  return uniqueDTCs;
}

void clearDTCs() {
  tCAN message;
  message.id = OBD_REQUEST_ID;
  message.header.rtr = 0;
  message.header.length = 8;
  message.data[0] = 0x01;
  message.data[1] = PID_CLEAR_DTC;
  memset(&message.data[2], 0, 6);

  if (!mcp2515_send_message(&message)) {
    Serial.println("Failed to send DTC clear request");
    return;
  }

  tCAN response;
  unsigned long start = millis();
  bool ecmCleared = false, tcmCleared = false, absCleared = false;

  while (millis() - start < 2000) {
    if (mcp2515_check_message() && mcp2515_get_message(&response)) {
      if (response.data[1] == 0x44) {
        if (response.id == OBD_RESPONSE_ID_ECM && !ecmCleared) {
          Serial.println("DTCs cleared successfully for ECM");
          ecmCleared = true;
        } else if (response.id == OBD_RESPONSE_ID_TCM && !tcmCleared) {
          Serial.println("DTCs cleared successfully for TCM");
          tcmCleared = true;
        } else if (response.id == OBD_RESPONSE_ID_ABS && !absCleared) {
          Serial.println("DTCs cleared successfully for ABS");
          absCleared = true;
        }
      } else if (response.data[1] == 0x7F) {
        Serial.print("Error clearing DTCs for ID 0x");
        Serial.print(response.id, HEX);
        Serial.print(", NRC: 0x");
        Serial.println(response.data[3], HEX);
      }
    }
    if (ecmCleared && tcmCleared && absCleared) break;
  }

  if (!ecmCleared) Serial.println("No response or error clearing DTCs for ECM");
  if (!tcmCleared) Serial.println("No response or error clearing DTCs for TCM");
  if (!absCleared) Serial.println("No response or error clearing DTCs for ABS");
}