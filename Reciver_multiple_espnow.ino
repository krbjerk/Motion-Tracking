/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
//Reciver multiple

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int rx;
  int ry;
  int rz;
  int posx;
  int posy;
  int posz;
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  //Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].rx = myData.rx;
  boardsStruct[myData.id-1].ry = myData.ry;
  boardsStruct[myData.id-1].rz = myData.rz;
  boardsStruct[myData.id-1].posx = myData.posx;
  boardsStruct[myData.id-1].posy = myData.posy;
  boardsStruct[myData.id-1].posz = myData.posz;
  /*Serial.printf("rx value: %d \n", boardsStruct[myData.id-1].rx);
  Serial.printf("ry value: %d \n", boardsStruct[myData.id-1].ry);
  Serial.printf("rz value: %d \n", boardsStruct[myData.id-1].rz);
  Serial.printf("posx value: %d \n", boardsStruct[myData.id-1].posx);
  Serial.printf("posy value: %d \n", boardsStruct[myData.id-1].posy);
  Serial.printf("posz value: %d \n", boardsStruct[myData.id-1].posz);*/
  //Serial.printf("rx: ", boardsStruct[myData.id-1].rx, " ry: ", boardsStruct[myData.id-1].ry, " rz: ", boardsStruct[myData.id-1].rz, " posx: ", boardsStruct[myData.id-1].posx, " posy: ", boardsStruct[myData.id-1].posy, " posz: ", boardsStruct[myData.id-1].posz);
  //Serial.println();
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // Acess the variables for each board
  int rx = boardsStruct[0].rx;
  int ry = boardsStruct[0].ry;
  int rz = boardsStruct[0].rz;
  int posx = boardsStruct[0].posx;
  int posy = boardsStruct[0].posy;
  int posz = boardsStruct[0].posz;
  Serial.println("x"+String(rx)+"y"+String(ry)+"z"+String(rz)+"a"+String(posx)+"b"+String(posy)+"c"+String(-posz));  
}
