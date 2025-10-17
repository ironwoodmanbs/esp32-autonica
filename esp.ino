#include <ETH.h>

#include <dummy.h>




#include "FIFO.h"
#include "BluetoothSerial.h"

#include <WiFi.h>
// String device_name = "ARC-025";
BluetoothSerial SerialBT;
uint8_t tittle[8];
uint8_t blueTitle[8];
uint8_t synchra[8];
uint8_t data[400];
uint8_t ffPlusData[16];

uint8_t buffer[8]; // для приёма блютуз данных
uint16_t bufferIndex = 0;    // для приёма блютуз данных

uint8_t countFF = 0;
uint8_t countBlueFF = 0;
bool isTittle = false;
bool isBlueTittle = false;
bool isData = false;
bool isBlueData = false;
uint16_t countBytes = 0;
uint16_t countBlueBytes = 0;
fifo_t uartfifo;
fifo_t btfifo;

uint8_t rmodearray[8];

#define RXD1 16
#define TXD1 17


void setup() {
  SerialBT.end();
  uint8_t mac[6];
  // esp_read_mac(mac, ESP_MAC_BT); 
  WiFi.macAddress(mac);
  char macStr[18]; 
  snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  String deviceName = "ARC-025 " + String(macStr);
  SerialBT.begin(deviceName);
  Serial.begin(115200);
  Serial1.begin(2000000, SERIAL_8N1, RXD1, TXD1);
  btfifo = fifo_create(1000,sizeof(uint8_t));
  uartfifo = fifo_create(10000, sizeof(uint8_t));
  for (int i = 0; i<6;i++)
    synchra[i] = 0xff;
 uint16_t crc = testCRC(synchra,6);
 synchra[6] = crc;
 synchra[7] = crc >> 8;
 rmodearray[0] = 0xA3;
 rmodearray[1] = 0xB4;
 crc = testCRC(rmodearray,6);
 rmodearray[6] = crc;
 rmodearray[7] = crc >> 8;

}

void loop() {
  // uint8_t testdata;
  //   while (SerialBT.available()) 
  //   {
  //   testdata = SerialBT.read();
  //   fifo_add(btfifo, &testdata);  
  //   }

  //   while (Serial1.available()) 
  //   {
  //   testdata = Serial1.read();
  //   fifo_add(uartfifo, &testdata);
  //   }

if (SerialBT.available()) {
        int availableBytesBT = SerialBT.available();
        uint8_t bufferBT[availableBytesBT]; 
        int bytesReadBT = SerialBT.readBytes(bufferBT, availableBytesBT);
        for (int i = 0; i < bytesReadBT; i++) {
            if (!fifo_add(btfifo, &bufferBT[i])) {
                break; 
            }
        }
    }


    if (Serial1.available()) {
        int availableBytesUART = Serial1.available();
        uint8_t bufferUART[availableBytesUART]; 
        int bytesReadUART = Serial1.readBytes(bufferUART, availableBytesUART);
        for (int i = 0; i < bytesReadUART; i++) {
            if (!fifo_add(uartfifo, &bufferUART[i])) {
                break; 
            }
        }
    }

  if(uartfifo->storedbytes >= 8 && isTittle == false && isData == false)      findSync();   
  if(btfifo->storedbytes>=8 && isBlueTittle == false && isBlueData == false)  findBlueSync();

  if(uartfifo->storedbytes >= 8 && isTittle == true)                          findTittle();
  if(btfifo->storedbytes >= 8 && isBlueTittle == true)                        findBlueTittle();

  if(uartfifo->storedbytes >= countBytes && isData == true)                   findData();
  if(btfifo->storedbytes >= countBlueBytes && isBlueData == true)             findBlueData();



}


void findSync(){

  uint8_t number;
        while (!fifo_is_empty(uartfifo)) {
            if (fifo_get(uartfifo, &number)) {
                if (number == 0xFF) {
                    countFF++;
                    if (countFF == 6) {
                        countFF = 0;
                        fifo_get(uartfifo, &number);
                        fifo_get(uartfifo, &number);
                        isTittle = true;
                          // Serial.write(synchra,8);
                          SerialBT.write(synchra,8); ///////////////
                        findTittle();
                        return;
                    }
                } else {
                    countFF = 0;
                }
            }
        }
}


void findBlueSync(){
  uint8_t number;
        while (!fifo_is_empty(btfifo)) {
            if (fifo_get(btfifo, &number)) {
                if (number == 0xFF) {
                    countBlueFF++;
                    if (countBlueFF == 6) {
                        countBlueFF = 0;
                        fifo_get(btfifo, &number);
                        fifo_get(btfifo, &number);
                        isBlueTittle = true;
                        // Serial1.write(synchra,8); 
                        findBlueTittle();
                        return;
                    }
                } else {
                    countBlueFF = 0;
                }
            }
        }
}


void findTittle(){
    if(uartfifo->storedbytes >= 8){
        for (int i = 0; i < 8; ++i) {
            fifo_get(uartfifo, &tittle[i]);
        }
        isTittle = false;
        if(testCRC(tittle,8) == 0) {
          SerialBT.write(tittle,8); ///////////////
            if (tittle[0] >= 0xB0  ){
                countBytes = tittle[5];
                if (tittle[5] == 0) countBytes = 256;
                isData = true;
                findData();
            }
        }
    }

}
void findBlueTittle(){
  if(btfifo->storedbytes >= 8)
  {
    for (int i = 0; i < 8; ++i) {
      fifo_get(btfifo, &blueTitle[i]);
    }
    isBlueTittle = false;
    if(testCRC(blueTitle,8) == 0)
      {
      memcpy(ffPlusData,synchra,8);
      memcpy(&ffPlusData[8],blueTitle ,8);
      Serial1.write(ffPlusData,16); 
      if(blueTitle[0] == 0xA3) {SerialBT.write(synchra,8);SerialBT.write(rmodearray,8);}  
      if (blueTitle[0] >= 0xB0 ){
          countBlueBytes = blueTitle[5];
          if (blueTitle[5] == 0) countBlueBytes = 256;
          isBlueData = true;
          findBlueData();
          
        }
      }
  }
}
void findBlueData(){
if(btfifo->storedbytes >= countBlueBytes){
    for (int i = 0; i<countBlueBytes; i++)
        fifo_get(btfifo, &data[i]); 
    Serial1.write(data,countBlueBytes); 
    isBlueData = false;
}
}
void findData(){
    if(uartfifo->storedbytes >= countBytes){
    for (int i = 0; i<countBytes; i++)
        fifo_get(uartfifo, &data[i]);
    // Serial.write(data,countBytes);
     SerialBT.write(data,countBytes); ///////////////
    isData = false;
    }
}


uint16_t testCRC(const uint8_t *data, int size) // функция подсчета CRC
{
    uint16_t crc = 0xFFFF; // начальное значение CRC
    for (int i = 0; i < size; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // полином CRC-16 Modbus
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}
