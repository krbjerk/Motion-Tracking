// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include "QuickMedianLib.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float Vx = 0;
float Vy = 0;
float Vz = 0;

float posX = 0;
float posY = 0;
float posZ = 0;

float sigX = 0;
float sigY = 0;
float sigZ = 0;

float lastTime = 0;

float ChangeBiasH = 2;
float ChangeBiasL = 0.5;
float avg = 0;
int counter = 0;





// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x23, 0x82, 0x7C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    int rx;
    int ry;
    int rz;
    int posX;
    int posY;
    int posZ;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    #ifdef USE_PIN
        SerialBT.setPin(pin);
        Serial.println("Using PIN");
    #endif
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

      // Init Serial Monitor
 
        // Set device as a Wi-Fi Station
   Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    /*
    mpu.setXGyroOffset(120);
    mpu.setYGyroOffset(-23);
    mpu.setZGyroOffset(27); 
    mpu.setXAccelOffset(-1538);
    mpu.setYAccelOffset(64);
    mpu.setZAccelOffset(950);
    */
    mpu.setXGyroOffset(-579);
    mpu.setYGyroOffset(-10);
    mpu.setZGyroOffset(-38); 
    mpu.setXAccelOffset(-2035);
    mpu.setYAccelOffset(-1808);
    mpu.setZAccelOffset(1260);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(20);
        mpu.CalibrateGyro(20);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        
            float currentStep = millis()-lastTime;
            lastTime = millis();
            
            float ChangeBiasVx = ChangeBiasH;
            float ChangeBiasVy = ChangeBiasH;
            float ChangeBiasVz = ChangeBiasH;
            float ChangeBiasPx = ChangeBiasH;
            float ChangeBiasPy = ChangeBiasH;
            float ChangeBiasPz = ChangeBiasH;

            if (aaWorld.x > 0 && Vx > 0 || aaWorld.x < 0 && Vx < 0){
                ChangeBiasVx = ChangeBiasL;
            }
            if (aaWorld.y > 0 && Vy > 0 || aaWorld.y < 0 && Vy < 0){
                ChangeBiasVy = ChangeBiasL;
            }
            if (aaWorld.z > 0 && Vz > 0 || aaWorld.z < 0 && Vz < 0){
                ChangeBiasVz = ChangeBiasL;
            }
            if (posX > 0 && Vx > 0 || posX < 0 && Vx < 0){
                ChangeBiasPx = ChangeBiasL;
            }
            if (posY > 0 && Vy > 0 || posY < 0 && Vy < 0){
                ChangeBiasPy = ChangeBiasL;
            }
            if (posZ > 0 && Vz > 0 || posZ < 0 && Vz < 0){
                ChangeBiasPz = ChangeBiasL;
            }

            float SigVx = 1000/(1+exp((-Vx-550)/100)) + 1000/(1+exp((-Vx+550)/100))-1000;
            float SigVy = 1000/(1+exp((-Vy-550)/100)) + 1000/(1+exp((-Vy+550)/100))-1000;
            float SigVz = 1000/(1+exp((-Vz-550)/100)) + 1000/(1+exp((-Vz+550)/100))-1000;

            Vx = ChangeBiasVx*aaWorld.x*currentStep/1000+Vx;
            Vy = ChangeBiasVy*aaWorld.y*currentStep/1000+Vy;
            Vz = ChangeBiasVz*aaWorld.z*currentStep/1000+Vz;

            /*
          posX = ChangeBiasVx*1/2*(aaWorld.x)*currentStep/1000*currentStep/1000 + ChangeBiasPx*Vx*currentStep/1000 + posX;
            if ( ypr[0]* 180/M_PI > 20 || ypr[0]* 180/M_PI < -20){
                 posY = ChangeBiasVy*1/2*(aaWorld.y)*currentStep/1000*currentStep/1000 + ChangeBiasPy*Vy*currentStep/1000 + posY - ypr[0]* 180/M_PI/60;
            } else {
                posY = ChangeBiasVy*1/2*(aaWorld.y)*currentStep/1000*currentStep/1000 + ChangeBiasPy*Vy*currentStep/1000 + posY;
            }
            
            posZ = ChangeBiasVz*1/2*(aaWorld.z)*currentStep/1000*currentStep/1000 + ChangeBiasPz*Vz*currentStep/1000 + posZ;

            if (posY > 140){
                posY = 140;
                Vy = 0;
            }
            if (posY < -140){
                posY = -140;
                Vy = 0;
            }
            if (posX < -140){
                posX = -140;
                Vx = 0;
            }
            if (posX > 140){
                posX = 140;
                Vx = 0;
            }
            if (posZ > 140){
                posZ = 140;
                Vz = 0;
            }
            if (posZ < -140){
                posZ = -140;
                Vz = 0;                
            }*/

            
            sigX = 50/(1+exp(-posX/10-100/30))+ 50/(1+exp(-posX/10+100/30))-50;
            sigY = 50/(1+exp(-posY/10-100/30))+ 50/(1+exp(-posY/10+100/30))-50;
            sigZ = 30/(1+exp(-posZ/10-100/30))+ 30/(1+exp(-posZ/10+100/30))-20;

            

            int rx = ypr[1]* 180/M_PI;
            int ry = ypr[2]* 180/M_PI;
            int rz = ypr[0]* 180/M_PI;



            if (counter == 100){
                posX = 0;
                posY = 0;
                posZ = 0;
                Vx = 0;
                Vy = 0;
                Vz = 0;
                counter++;
            } else {
                counter++;
            }

            posX = 0;
            if (rx > -30 && rx < 30){
                posY = (rz+25)*0.7 + (ChangeBiasVy*1/2*(aaWorld.y)*currentStep/1000*currentStep/1000 + ChangeBiasPy*Vy*currentStep/1000 + posY)*0.3;
            } else {
                //(ChangeBiasVy*1/2*(aaWorld.y)*currentStep/1000*currentStep/1000 + ChangeBiasPy*Vy*currentStep/1000 + posY)*0.3;
            }
            posZ = ry*0.7;
            //(ChangeBiasVz*1/2*(aaWorld.z)*currentStep/1000*currentStep/1000 + ChangeBiasPz*Vz*currentStep/1000 + posZ)*0.3;

            if (posY > 90){
                posY = 90;
            }
            if (posY < -90){
                posY = -90;
            }
            if (posX < -90){
                posX = -90;
            }
            if (posX > 90){
                posX = 90;
            }
            if (posZ > 90){
                posZ = 90;
            }
            if (posZ < -90){
                posZ = -90;               
            }

            myData.id = 1;
            myData.rx = rx;
            myData.ry = ry;
            myData.rz = rz;
            myData.posX = posX;
            myData.posY = posY;
            myData.posZ = posZ;
            
            Serial.println("x"+String(rx)+"y"+String(ry)+"z"+String(rz)+"a"+String(posX)+"b"+String(posY)+"c"+String(-posZ));
            //Serial.println("x: "+String(SigVx)+" y: "+ String(SigVy)+ " z: "+ String(SigVz));
            //Serial.println(aaWorld.z);
            // Send message via ESP-NOW
            //Serial.println(rx);

            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
            
            if (result == ESP_OK) {
                //Serial.println("Sent with success");
            }
            else {
                //Serial.println("Error sending the data");
            }

            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
            
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
