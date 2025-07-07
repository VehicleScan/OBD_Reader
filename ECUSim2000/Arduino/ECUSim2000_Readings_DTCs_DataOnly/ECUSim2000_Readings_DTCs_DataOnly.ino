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
  Canbus.init(CANSPEED_500);
}

int requestPID(uint8_t mode, uint8_t pid);
String collectDTCs(uint8_t mode, uint16_t respID);
String removeDuplicateDTCs(String dtcList);
bool clearDTCs();

void loop() {
  if (Serial.available() >= 2) {
    char command = Serial.read();
    if (Serial.read() != '\n') {
      while (Serial.available()) Serial.read();
      return;
    }

    if (command == 'r') {
      int values[5] = {
        requestPID(0x01, PID_COOLANT_TEMP),
        requestPID(0x01, PID_ENGINE_RPM),
        requestPID(0x01, PID_VEHICLE_SPEED),
        requestPID(0x01, PID_TIRE_PRESSURE),
        requestPID(0x01, PID_MAF)
      };
      Serial.print(values[0]); Serial.print(',');
      Serial.print(values[1]); Serial.print(',');
      Serial.print(values[2]); Serial.print(',');
      Serial.print(values[3]); Serial.print(',');
      Serial.println(values[4]);
    } 
    else if (command == 'd') {
      String allDTCs = "";
      allDTCs += collectDTCs(PID_DTC, OBD_RESPONSE_ID_ECM);
      delay(100);
      allDTCs += collectDTCs(PID_PENDING_DTC, OBD_RESPONSE_ID_ECM);
      delay(100);
      allDTCs += collectDTCs(PID_PERMANENT_DTC, OBD_RESPONSE_ID_ECM);
      delay(100);
      allDTCs += collectDTCs(PID_PENDING_DTC, OBD_RESPONSE_ID_TCM);
      delay(100);
      allDTCs += collectDTCs(PID_PENDING_DTC, OBD_RESPONSE_ID_ABS);
      
      if (allDTCs.length() > 0 && allDTCs.endsWith(",")) {
        allDTCs.remove(allDTCs.length() - 1);
      }
      allDTCs = removeDuplicateDTCs(allDTCs);
      Serial.println(allDTCs);
    } 
    else if (command == 'c') {
      Serial.println(clearDTCs() ? "1" : "0");
    }
  }
}

int requestPID(uint8_t mode, uint8_t pid) {
  tCAN message;
  message.id = OBD_REQUEST_ID;
  message.header.rtr = 0;
  message.header.length = 8;
  message.data[0] = 0x02;
  message.data[1] = mode;
  message.data[2] = pid;
  memset(&message.data[3], 0, 5);

  mcp2515_send_message(&message);

  tCAN response;
  unsigned long start = millis();
  while (millis() - start < 10) {
    if (mcp2515_check_message() && mcp2515_get_message(&response) && 
        response.id == OBD_RESPONSE_ID_ECM && 
        response.data[1] == (mode | 0x40) && 
        response.data[2] == pid) {
      
      switch (pid) {
        case PID_COOLANT_TEMP: return response.data[3] - 40;
        case PID_ENGINE_RPM: return (((uint16_t)response.data[3] << 8) | response.data[4]) / 4;
        case PID_VEHICLE_SPEED: return response.data[3];
        case PID_TIRE_PRESSURE: return 15 + ((int32_t)response.data[3] * 27 / 255);
        case PID_MAF: return (((uint16_t)response.data[3] << 8) | response.data[4]) / 100;
      }
    }
  }
  return 0;
}

String collectDTCs(uint8_t mode, uint16_t respID) {
  tCAN msg;
  msg.id = OBD_REQUEST_ID;
  msg.header.rtr = 0;
  msg.header.length = 8;
  msg.data[0] = 0x01;
  msg.data[1] = mode;
  memset(&msg.data[2], 0, 6);

  mcp2515_send_message(&msg);

  String dtcString = "";
  tCAN response;
  unsigned long start = millis();
  bool isFirstFrame = true;
  int totalBytes = 0;
  int processedBytes = 0;

  while (millis() - start < 25) {
    if (mcp2515_check_message() && mcp2515_get_message(&response) && response.id == respID) {
      if (isFirstFrame && response.data[0] == 0x10) {
        totalBytes = ((response.data[1] << 8) | response.data[2]) - 2;
        for (int i = 4; i + 1 < response.header.length; i += 2) {
          uint8_t A = response.data[i];
          uint8_t B = response.data[i + 1];
          if (A == 0 && B == 0) continue;
          char dtc[6];
          char type = "PCBU"[A >> 6];
          sprintf(dtc, "%c%02X%02X", type, A & 0x3F, B);
          dtcString += String(dtc) + ",";
          processedBytes += 2;
        }
        isFirstFrame = false;
      }
      else if (response.data[1] == (mode | 0x40)) {
        for (int i = 3; i + 1 < response.header.length; i += 2) {
          uint8_t A = response.data[i];
          uint8_t B = response.data[i + 1];
          if (A == 0 && B == 0) continue;
          char dtc[6];
          char type = "PCBU"[A >> 6];
          sprintf(dtc, "%c%02X%02X", type, A & 0x3F, B);
          dtcString += String(dtc) + ",";
        }
        break;
      }
    }
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
      uniqueDTCs += dtc;
    }
  }
  return uniqueDTCs;
}

bool clearDTCs() {
  tCAN message;
  message.id = OBD_REQUEST_ID;
  message.header.rtr = 0;
  message.header.length = 8;
  message.data[0] = 0x01;
  message.data[1] = PID_CLEAR_DTC;
  memset(&message.data[2], 0, 6);

  mcp2515_send_message(&message);

  tCAN response;
  unsigned long start = millis();
  bool ecmCleared = false, tcmCleared = false, absCleared = false;

  while (millis() - start < 25) {
    if (mcp2515_check_message() && mcp2515_get_message(&response)) {
      if (response.data[1] == 0x44) {
        if (response.id == OBD_RESPONSE_ID_ECM) ecmCleared = true;
        else if (response.id == OBD_RESPONSE_ID_TCM) tcmCleared = true;
        else if (response.id == OBD_RESPONSE_ID_ABS) absCleared = true;
      }
    }
    if (ecmCleared && tcmCleared && absCleared) break;
  }
  return (ecmCleared && tcmCleared && absCleared);
}
