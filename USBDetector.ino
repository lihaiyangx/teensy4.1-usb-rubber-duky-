/* USBDetector.ino, CREATED 03M 14D, 2022Y  
 * arduino IDE 1.8.19(Teensyduino 1.56), for Teensy 4.1
 * USB EHCI Host ; USB Slav Interface:Serial, Keyboard, Mouse, RawHID(64Bytes)
 * Description: \readmeli.txt
 * By li.
 * 
 * USBDetector20220823, \src\main.cpp, CREATED 08M 23D, 2022Y
 * VS code + PIO IDE 2.5.2(platformio/teensy 4.17.0: Teensyduino 1.57),  for Teensy 4.1
 * USB EHCI Host ; USB Slav Interface:Serial, Keyboard, Mouse, RawHID(64Bytes)
 * Description: \readmeli.txt
 * By li.
*/

// Q1:鼠标无法进行拖拽动作
// Q2；部分特殊按键的键值数据格式不对，可能是底层程序里没有定义那些特殊数据   b'\xb1\x00*\x00\xb1\x00*\x00'

#include <Arduino.h>
#include "USBHost_t36.h"
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>            // a basic DS1307 library that returns time as a time_t
#include <exception>              // exception header


//=======================================================================================
// DEBUG
//=======================================================================================
// #define USBHOST_PRINT_DEBUG               // define in USBHost_t36.h file


//=======================================================================================
// My defines
//=======================================================================================
#define KEYBOARD_DATA_SERIALMONITOR   // show keyboard data serial monitor 
#define KEYBOARD_DATA_FOREWARD        // keyboard data to pc
#define KEYBOARD_DATA_PI              // keyboard data to raspberrypi
#define MOUSE_DATA_SERIALMONITOR      // show mouse data serial monitor
#define MOUSE_DATA_FOREWARD           // mouse data to pc
#define MOUSE_DATA_PI                 // mouse data to raspberrypi
#define RAWHID_DATA_SERIALMONITOR     // rawhid data serial monitor
#define RAWHID_DATA_FOREWARD          // rawhid data to pc, data stream dev->teensy->pc
#define RAWHID_DATA_DOWNWARD          // rawhid data to dev, data stream pc->teensy->dev
#define RAWHID_DATA_PI                // rawhid data to raspberrypi
#define USE_ST77XX                    // show USB info tft


//======================================================================================
// USB Host Ojbects
//======================================================================================
// 以下这些实例对象都要在内存分配相应的空间
USBHost myusb;                              // Initialize myusb Controller
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHub hub3(myusb);
USBHub hub4(myusb);
KeyboardController keyboard1(myusb);        // Attach keyboard1 controller to myusb
KeyboardController keyboard2(myusb); 
MouseController mouse1(myusb);
MouseController mouse2(myusb);
RawHIDController rawhid1(myusb);            // RawHIDController 是 USBHIDInput 的子类
RawHIDController rawhid2(myusb);
RawHIDController rawhid3(myusb); 
RawHIDController rawhid4(myusb, 0xffc90004); 
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
USBHIDParser hid5(myusb);
USBHIDParser hid6(myusb);
//msController msDrive1(myusb);
//msController msDrive2(myusb);


//====================================================================
// Connection configuration of ST77XX LCD TFT
//====================================================================
#ifdef USE_ST77XX
#include <ST7735_t3.h>
#include <st7735_t3_font_Arial.h>
#include <ST7789_t3.h>
#define BLACK ST77XX_BLACK
#define WHITE ST77XX_WHITE
#define YELLOW ST77XX_YELLOW
#define GREEN ST77XX_GREEN
#define RED ST77XX_RED
#endif
// Default pins: 8 = RST, 9 = D/C, 10 = CS
// 补充pin11与pin13: 11 = MOSI, 13 = SCLK
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10
#ifdef USE_ST77XX
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_RST);
#endif

//===============================================================
//  Software serial,  Best for Teensy 4.1
//===============================================================
//SoftwareSerial mySerial(0, 1);    // (RX pin, TX pin)
//SoftwareSerial mySerial(7, 8);
//SoftwareSerial mySerial(15, 14);
//SoftwareSerial mySerial(16, 17);
//SoftwareSerial mySerial(21, 20);
//SoftwareSerial mySerial(25, 24);
//SoftwareSerial mySerial(28, 29);
SoftwareSerial mySerial(34, 35);    // Serial8,  raspberrypi <-> teensy



void ShowUpdatedDeviceListInfo();
void MouseDataProcess();
void mySerialReadProcess();
bool OnReceiveHidData(uint32_t usage, const uint8_t *data, uint32_t len);
void ShowHIDExtrasPress(uint32_t top, uint16_t key);
void time_t_to_char(time_t tm, char buffer_t[]);
uint8_t ascii_to_hex(char ch);
void time_t_to_byte(time_t tm, byte buffer_t[]);
void SyncTimeProcess();
void OnPress(int key);
void OnRawPress(uint8_t keycode);
void ShowUpdatedDeviceListInfo();
void KeyboardDataProcess();
char hex_to_ascii(uint8_t he);
void digitalClockDisplay();
void printDigits(int digits);
void ShowUpdatedHIDMouseDeviceListInfo();
void OnRawRelease(uint8_t keycode);
void RawHidInProcess();
void RawHidOutProcess();
void mySerialWriteProcess(byte write_buffer[], uint8_t write_buffer_size, bool keyboard_data);



//=====================================================================================================
// Setup()
//=====================================================================================================
void setup()
{
    myusb.begin();       
    Serial.begin(115200);                              // 230400 or no this line
    while (!Serial) ;                                  // wait for Arduino Serial Monitor
    Serial.println("USBDetector: Waiting for USB Device... \n");       
    mySerial.begin(115200);                            // set the data rate for the SoftwareSerial port
    mySerial.println("Hello raspberrypi, I am teensy."); 
    Serial.println("mySerial open, say hello to raspberry");
    Mouse.begin();                                     // initialize mouse control
    Mouse.screenSize(1920, 1080);                      // configure screen size
    Keyboard.begin();   
    rawhid1.attachReceive(OnReceiveHidData);
    rawhid2.attachReceive(OnReceiveHidData);
    rawhid3.attachReceive(OnReceiveHidData);
    rawhid4.attachReceive(OnReceiveHidData);   
#ifdef KEYBOARD_DATA_FOREWARD 
    keyboard1.attachPress(OnPress);
    keyboard1.attachRawPress(OnRawPress);
    keyboard1.attachRawRelease(OnRawRelease);
    // keyboard1.attachExtrasPress(OnHIDExtrasPress);
    // keyboard1.attachExtrasRelease(OnHIDExtrasRelease);
    keyboard2.attachPress(OnPress);
    keyboard2.attachRawPress(OnRawPress);
    keyboard2.attachRawRelease(OnRawRelease);
    // keyboard2.attachExtrasPress(OnHIDExtrasPress);
    // keyboard2.attachExtrasRelease(OnHIDExtrasRelease); 
#endif  
#ifdef USE_ST77XX
    tft.init(240, 240);                    // initialize a ST7789 chip, 240x240 pixels
    Serial.println(F("tft Initialized"));    
    // tft.setFrameBuffer(frame_buffer);   // explicitly set the frame buffer  
    //uint16_t time = millis();
    //tft.fillScreen(BLACK);
    //time = millis() - time;
    //Serial.println(time, DEC);
    //delay(2000);    
    tft.setRotation(3);                   // 180度
    delay(100);
    tft.fillScreen(BLACK);
    tft.setTextColor(YELLOW);
    tft.setTextSize(2);
    tft.println("USBDetector: Waiting for Device...");
    delay(100);
    // tft.useFrameBuffer(true);         // 此函数啥功能，没有找到函数的定义，不注释掉它，后面没法显示
#endif
    // RTC 
    setSyncProvider(RTC.get);            // the function to get the time from the RTC, 计时器与RTC同步 
    // LED
    // pinMode(LED_BUILTIN, OUTPUT);       // initialize LED digital pin as an output.
}
/////////////////////////////////Setup() end///////////////////////////////////////////////////////////////


//=========================================================================================================
// Loop()
//=========================================================================================================
byte buffer_[64];
elapsedMillis msUntilNextSend;
unsigned int packetCount = 0;
byte command_buffer[64];       // 存储raspberrypi 发来的控制信号，注意采纳后清空
bool new_raspberrypi_timestamp = false;            // flag to judge wether a new timestamp date is received

void loop()
{
    // try{
    myusb.Task();                       // Task(), polls connected usb devices for updates to their status.   
    ShowUpdatedDeviceListInfo();        // Both serial and tft
    ShowUpdatedHIDMouseDeviceListInfo();
    // }
    // catch (...) {
    //     Serial.println(F("exception occur!"));
    //     // cerr << e.what() << endl;
    //     // Serial.print(e.what());
    //     setup();  // reset?
    // }   
    MouseDataProcess();                 // Mouse data: forward, serialmonitor, raspberrypi.
    KeyboardDataProcess();              // Keyboard Data: to  raspberrypi.
    // mySerial
    new_raspberrypi_timestamp = false;  // the last timestamp received is old
    mySerialReadProcess();              // read data from raspberry serial port
    // time dispose   
    SyncTimeProcess();                  // Sync Arduino clock to the time received on the serial port(raspberrypi).    
    // time test OK
    /*      
    time_t time_now_s = now();                  // s,  9,      0:00:08 1 1 1970
    Serial.print("time_now_s: ");  Serial.println(time_now_s);    
    time_t time_now_ms = now_millisecond();     // ms, 9431,  now_millisecond()在TimeLib.h中新定义
    Serial.print("time_now_ms: ");  Serial.println(time_now_ms);   
    time_t time_now_us = now_microseconds();    // us, 9431571, now_microseconds()在TimeLib.h中新定义
    Serial.print("time_now_us: ");  Serial.println(time_now_us);              
                                                // 不用考虑计数器溢出，够用几十年？ 
                                                // 不对，底层代码用的四个字节不是八个字节，这里会溢出，运行中发现数据类型是signed int, 咋办？
                                                // 测试中us的低三位不变，是因为程序的执行顺序问题, 实际获取的是动态的us数据
    */
    
    // rawhid test OK
    // RawHidInProcess();                 // rawhid data forward.   // 最后转到OnReceiveHidData()实现
    // RawHidOutProcess();                // rawhid data downward.

    // heartbeat  不知道为啥添加这里会影响该程序运行
    // if(digitalRead(LED_BUILTIN) == HIGH){
    //     digitalWrite(LED_BUILTIN, LOW);
    // }else{
    //     digitalWrite(LED_BUILTIN, HIGH);
    // }
    // delay(1000);
}
///////////////////////////////loop() end//////////////////////////////////////////////////////////////////


//==========================================================================================================
// rawhid
//       RawHidInProcess():  Device ->  Teensy -> PC
//       RawHidOutProcess():  PC -> Teensy -> Device
//       Device <-> 'rawhid' api <-> Teensy
//       Teensy <-> 'RawHID' api <-> PC
// RawHID的send(),recv()方法继承自usb_rawhid_class(C:\Users\LHY\.platformio\packages\framework-arduinoteensy
// \cores\teensy4\usb_rawhid.h);  
//==========================================================================================================
void RawHidInProcess()
{
    // Device ->  Teensy  ---------------------------------------------------------------- OK  
    if (rawhid1) // See if we have some RAW data
    {
        int ch;
        uint8_t buffer[64];  // RECV_buffer[64]
        uint8_t count_chars = 0;
        memset(buffer, 0, sizeof(buffer));
        if (Serial.available()) 
        {
            while (((ch = Serial.read()) != -1) && (count_chars < sizeof(buffer))) 
            {
                buffer[count_chars++] = ch;
                Serial.print(ch);
            }
            rawhid1.sendPacket(buffer);
            Serial.print("\n buffer:");
            //Serial.print(buffer);
            //Serial.println(); 
        }
    }

    // Teensy -> PC  ----------------------------------------------------------------------- OK 
    /*
    if (msUntilNextSend > 50000)  // every 50 seconds, send a packet to the computer
    {
        msUntilNextSend = msUntilNextSend - 50000;
        byte send_buffer_t[64] = {0x3f, 0x3e, 0x42, 0xf5, 0x0b, 0xff, 0x2d, 0x53, 
                                 0x74, 0x61, 0x72, 0x74, 0x2d, 0x40, 0x00, 0xa6, 
                                 0x29, 0x23, 0xbe, 0x84, 0xe1, 0x6c, 0xd6, 0xae, 
                                 0x52, 0x90, 0x49, 0xf1, 0xf1, 0xbb, 0xe9, 0xeb};
        int tx_packet_len = 0;
        // send
        tx_packet_len = RawHID.send(send_buffer_t, 1);  //
        if(tx_packet_len!=0){
            Serial.print("teensy to pc, rawhid data size:0x");
            Serial.println(tx_packet_len, HEX);
        }
        else {
            Serial.println(F("Unable to transmit packet"));
        }
    }
    */
}


void RawHidOutProcess()
{
    // PC -> Teensy   --------------------------------------------------------------------- OK
    /* PC通过设备的VENDOR_ID,PRODUCT_ID,RAWHID_USAGE_PAGE,RAWHID_USAGE来识别将要输出数据的目标设备*/
    byte recv_buffer_t[256];
    int rx_packet_len = 0;
    if(RawHID.available()){
        rx_packet_len = RawHID.recv(recv_buffer_t, 1);  // when timeout=0, do not wait
    }
    if(rx_packet_len!=0)
    {
        Serial.print("RAWHID data(PC->T4.1): size:0x"); 
        Serial.println(rx_packet_len, HEX);
        /*
        for (int i=0; i<rx_packet_len; i++) 
        {
            // Serial.print("0x");
            Serial.printf("%02x ", (int)recv_buffer_t[i]);
            if((i+1)%16 == 0)  {Serial.println();}
            else  {Serial.print(" ");}
        }
        Serial.println();
        */

        #ifdef RAWHID_DATA_PI
        byte send_buffer_t[257];
        uint8_t tx_packet_len = rx_packet_len+1;
        send_buffer_t[0] = 0xCD;                // RAWHID data(t4.1->pi) identifier -> 0xCD
        for(int i=0; i<rx_packet_len; i++){
            send_buffer_t[i+1] = recv_buffer_t[i];
            // Serial.println(data_buffer[i]);
        }
        mySerialWriteProcess(send_buffer_t, (tx_packet_len), false);
        Serial.print("RAWHID data(t4.1->pi): size:0x");
        Serial.println(tx_packet_len, HEX);
        #endif

        // Teensy -> Device   ----------------------------------------------------------------- NO 
        #ifdef RAWHID_DATA_DOWNWARD  
        // byte send_buffer_t_[64];
        byte send_buffer_t_[64] = {0x3f, 0x3e, 0x42, 0xf5, 0x0b, 0xff, 0x2d, 0x53,   // uint8_t
                                   0x74, 0x61, 0x72, 0x74, 0x2d, 0x40, 0x00, 0xa6, 
                                   0x29, 0x23, 0xbe, 0x84, 0xe1, 0x6c, 0xd6, 0xae, 
                                   0x52, 0x90, 0x49, 0xf1, 0xf1, 0xbb, 0xe9, 0xeb};
         if(rawhid1.sendPacket((uint8_t*)send_buffer_t_))
        {
            Serial.print("test RAWHID data(t4.1->dev): size:");
            Serial.println(sizeof(send_buffer_t_), HEX);
        }

        if(rawhid1.sendPacket((uint8_t*)recv_buffer_t))
        {
            Serial.print("RAWHID data(t4.1->dev): size:");
            Serial.println(rx_packet_len, HEX);
        }else{
            Serial.print("RAWHID data(t4.1->dev): Is there something wrong?\n");
        }
        #endif

        #ifdef RAWHID_DATA_SERIALMONITOR
        Serial.print("RAWHID data(PC->T4.1):");
        Serial.println();
        int len = rx_packet_len;
        int num_line = 0;
        while (len) 
        {
            uint8_t cb = (len > 16) ? 16 : len;
            uint8_t i;
            for (i = 0; i < cb; i++) {
                Serial.printf("%02x ", (int)recv_buffer_t[num_line*16+i]);
            }
            Serial.print(": ");
            for (i = 0; i < cb; i++) {
                Serial.write(((recv_buffer_t[num_line*16+i] >= ' ') && (recv_buffer_t[num_line*16+i] <= '~')) ? recv_buffer_t[num_line*16+i] : '.');
            }
            len -= cb;
            num_line++;
            Serial.println();
        }
        #endif
    }

   
    
}
//////////////////////rawhid end////////////////////////////////////////////////////////////////////////////


//==========================================================================================================
// mySerial
// pi->teensy,   a time package:  '0x62a5a00c',  unit:s,                  size:9
//             a string package:  "Hello teensy, I am raspberrypi.",      size:31(0-30),  "算一个???, 不算
//                                "raspberrypi cmd: Warning",             size:24 
// teensy->pi, USB data + timestamp        
//==========================================================================================================
char myserial_recv_buffer[40];             
uint8_t index_t = 0;
unsigned long raspberry_timestamp;
void mySerialReadProcess()
{
    if (mySerial.available())       // 有数据接收到
    {    
        myserial_recv_buffer[index_t] = mySerial.read();  // 这里采用每次读一个字符
        // test:for all received chars
        // Serial.write(myserial_recv_buffer[index_t]); 
        // Serial.println(index_t);
        // timestamp ------------------------------------------------------------------------- OK  
        if(index_t < 9)         // 正在接收时..., timestamp or warning or greeting
        {           
            index_t++;
        }
        else if(myserial_recv_buffer[0] == '0' && myserial_recv_buffer[1]== 'x' && index_t == 9)
        {   
            // Serial.println();                              
            // for(int j=0; j<10; j++) Serial.write(myserial_recv_buffer[j]);
            // Serial.println();
            new_raspberrypi_timestamp = true;
            raspberry_timestamp = 
                              ascii_to_hex(myserial_recv_buffer[2]) *16*16*16*16*16*16*16
                            + ascii_to_hex(myserial_recv_buffer[3]) *16*16*16*16*16*16
                            + ascii_to_hex(myserial_recv_buffer[4]) *16*16*16*16*16
                            + ascii_to_hex(myserial_recv_buffer[5]) *16*16*16*16
                            + ascii_to_hex(myserial_recv_buffer[6]) *16*16*16
                            + ascii_to_hex(myserial_recv_buffer[7]) *16*16
                            + ascii_to_hex(myserial_recv_buffer[8]) *16
                            + ascii_to_hex(myserial_recv_buffer[9]) *1;
            // Serial.println(raspberry_timestamp);     
            index_t = 0;
            for(int k=0; k<9; k++)  myserial_recv_buffer[k] = 'g';  // 覆盖 '0',  'x'
        }
        // warning ---------------------------------------------------------------------------- OK
        else if(index_t < 23)   // 正在接收接收..., warning or greeting
        {           
            index_t++;
        }
        else if(myserial_recv_buffer[0] == 'r' && myserial_recv_buffer[1] == 'a' && index_t == 23)
        {
            // warning: here to do something
            //
            Serial.println();                          
            for(int i=0; i<24; i++)  Serial.write(myserial_recv_buffer[i]);
            Serial.println();
            index_t = 0;
            for(int k=0; k<9; k++)  myserial_recv_buffer[k] = 'g';
        }
        // greeting --------------------------------------------------------------------------- OK 
        else if(index_t < 30)   // 正在接收接收..., greeting
        {           
            index_t++;
        }
        else if(myserial_recv_buffer[0] == 'H' && myserial_recv_buffer[1] == 'e' &&index_t == 30)
        {
            Serial.println();                          
            for(int k=0; k<31; k++)  Serial.write(myserial_recv_buffer[k]);
            Serial.println();
            index_t = 0;
            for(int k=0; k<32; k++)  myserial_recv_buffer[k] = 'g';
        }  
    }
    else                   // 没有收到数据
    {
        index_t = 0;       // this is necessary，没有的话索引会紊乱
    }
}

void char_to_time_t()
{
    //
}

uint8_t ascii_to_hex(char ch)               // char -> uin8_t
{
    uint8_t deposit;
    switch(ch)
    {
        case '0':  deposit = 0x00;  break;
        case '1':  deposit = 0x01;  break;
        case '2':  deposit = 0x02;  break;
        case '3':  deposit = 0x03;  break;
        case '4':  deposit = 0x04;  break;
        case '5':  deposit = 0x05;  break;
        case '6':  deposit = 0x06;  break;
        case '7':  deposit = 0x07;  break;
        case '8':  deposit = 0x08;  break;
        case '9':  deposit = 0x09;  break;
        case 'a':  deposit = 0x0a;  break;
        case 'b':  deposit = 0x0b;  break;
        case 'c':  deposit = 0x0c;  break;
        case 'd':  deposit = 0x0d;  break;
        case 'e':  deposit = 0x0e;  break;
        case 'f':  deposit = 0x0f;  break;
        default:   deposit = 0xff;  break;  // return 0xff indicates false
    }
    return deposit;
}


char myserial_send_buffer[10];
byte myserial_send_buffer_b[4];
void mySerialWriteProcess(byte write_buffer[], uint8_t write_buffer_size, bool keyboard_data)
{
    // write buffer data
    uint8_t num_size = write_buffer_size;
    byte write_buffer_t[num_size];
    for (int i=0; i<num_size; i++)   { write_buffer_t[i] = write_buffer[i];}
    for (int i=0; i<num_size; i++)   { mySerial.write(write_buffer_t[i]); }
        
    // write timestamp data, s
    time_t timestamp = now();
    // time_t_to_char(timestamp, myserial_send_buffer);
    // for (int i=0; i<10; i++)   { mySerial.write(myserial_send_buffer[i]); }
    time_t_to_byte(timestamp, myserial_send_buffer_b);
    for (int i=0; i<4; i++)   { mySerial.write(myserial_send_buffer_b[i]); }
    
    // ms
    if(keyboard_data)    // 键盘数据, 填充ms时间戳
    {
        // extern time_t now_millisecond();
        time_t time_now_ms = now_millisecond();  
        // time_t_to_char(time_now_ms, myserial_send_buffer);
        time_t_to_byte(time_now_ms, myserial_send_buffer_b);        
    }
    else                // 非键盘数据, 填充空
    {
        // for (int i=0; i<10; i++)   { myserial_send_buffer[i] = '0'; }
        for (int i=0; i<4; i++)   { myserial_send_buffer_b[i] = 0x00; }        
    }
    // for (int i=0; i<10; i++)   { mySerial.write(myserial_send_buffer[i]); } 
    for (int i=0; i<4; i++)   { mySerial.write(myserial_send_buffer_b[i]); }
}

void time_t_to_byte(time_t tm, byte buffer_t[])  // 4个字节长度的时间数据 -> 4个分开的字节
{
    buffer_t[0] = byte(tm >> (8*3));
    buffer_t[1] = byte(tm >> (8*2));
    buffer_t[2] = byte(tm >> (8*1));
    buffer_t[3] = byte(tm);
}

void time_t_to_char(time_t tm, char buffer_t[])  // 4个字节长度的时间数据 -> 8个字符
{
    time_t quotient = tm;  
    buffer_t[9] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[8] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[7] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[6] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[5] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[4] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[3] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[2] = hex_to_ascii(quotient % 16);   quotient = quotient / 16;
    buffer_t[1] = 'x';
    buffer_t[0] = '0';
}

char hex_to_ascii(uint8_t he)  // uin8_t -> char
{
    char deposit;
    switch(he)
    {       
        case 0x00:  deposit = '0';  break;
        case 0x01:  deposit = '1';  break;
        case 0x02:  deposit = '2';  break;
        case 0x03:  deposit = '3';  break;
        case 0x04:  deposit = '4';  break;
        case 0x05:  deposit = '5';  break;
        case 0x06:  deposit = '6';  break;
        case 0x07:  deposit = '7';  break;
        case 0x08:  deposit = '8';  break;
        case 0x09:  deposit = '9';  break;
        case 0x0a:  deposit = 'a';  break;
        case 0x0b:  deposit = 'b';  break;
        case 0x0c:  deposit = 'c';  break;
        case 0x0d:  deposit = 'd';  break;
        case 0x0e:  deposit = 'e';  break;
        case 0x0f:  deposit = 'f';  break;
        default:    deposit = 'g';  break;  // return g indicates false
    }
    return deposit;
}
///////////////////////////////////mySerial end//////////////////////////////////////////////////////////////////////


//===================================================================================================================
// time
//===================================================================================================================
unsigned long raspberrypi_time;                    // time_t
const unsigned long DEFAULT_TIME = 1357041600;     // Jan 1 2013     // 8 bytes
time_t this_sync_time = 0;
time_t next_sync_time = 0;
time_t sync_interval =  0.1*60*60;                  // 同步时间间隔x小时: x*60min*60sec
void SyncTimeProcess()                             // teensy 与 raspberrypi 同步时间
{  
    raspberrypi_time = raspberry_timestamp;        // mySerial read timestamp
    // Serial.print("raspberrypi_time: ");  
    // Serial.println(raspberrypi_time);
    if(raspberrypi_time >= DEFAULT_TIME && now() >= next_sync_time && new_raspberrypi_timestamp) // check the integer is a valid time (greater than Jan 1 2013)
    {
        RTC.set(raspberrypi_time);           // set the RTC and the system time to the received value
        setTime(raspberrypi_time);           // Sync Arduino clock to the raspberrypi_time received on the serial port
        Serial.println("Sync RTC with Raspberrypi ONCE");
        this_sync_time = now();
        next_sync_time = this_sync_time + sync_interval; 
        Serial.println(now());
        digitalClockDisplay();       
    }   
    // Serial.print("timeStatus: ");  Serial.println(timeStatus());  // 这个是计时器的？？应该是计时器的
    // digitalClockDisplay(); 
}

void digitalClockDisplay()    // digital clock display of the time   
{                             // 格林威治时间, 北京时间 = 格林威治时间 + 8h
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");  Serial.print(day());
    Serial.print(" ");  Serial.print(month());
    Serial.print(" ");  Serial.print(year()); 
    Serial.println(); 
}

void printDigits(int digits)  // utility function for digital clock display
{
    Serial.print(":");        // prints preceding colon
    if(digits < 10)
        Serial.print('0');    // prints leading 0
    Serial.print(digits);
}
//////////////////////////////// time end  ////////////////////////////////////////////////////////////////////////////////


// ===============================================================================================================================
// Mouse
//      8 Bytes: MouseX=0/1/2(鼠标编号), buttons=0/1/2(无/左点击/右点击), mouseX=+-n,  mouseY=+/-n(上滚/下滚),  wheel=+-n,  wheelH=0/?
//               后面两个字节空置保留
// ===============================================================================================================================
// int8_t mouse_data_buffer[6];             // 解决坐标正负的数据类型，但无法与其他数据类型统一
const uint8_t mouse_data_size = 8;
byte mouse1_data_buffer[mouse_data_size];   // -1 = 255 - 256，  -1得到的是255； pi用到时要进一步处理
byte mouse2_data_buffer[mouse_data_size]; 
void MouseDataProcess()
{
    // mouse1-----------------------------------------------------------------------
    if (mouse1.available())                 // mouse1Event
    {
        mouse1_data_buffer[0] = 0xA1;       // mouse1 identifier
        mouse1_data_buffer[1] = mouse1.getButtons();
        mouse1_data_buffer[2] = mouse1.getMouseX();
        mouse1_data_buffer[3] = mouse1.getMouseY();
        mouse1_data_buffer[4] = mouse1.getWheel();  
        mouse1_data_buffer[5] = mouse1.getWheelH();
#ifdef MOUSE_DATA_SERIALMONITOR
        Serial.print("Mouse1: buttons = ");  Serial.print(mouse1_data_buffer[1]);
        Serial.print(",  mouseX = ");        Serial.print(mouse1_data_buffer[2]);
        Serial.print(",  mouseY = ");        Serial.print(mouse1_data_buffer[3]);
        Serial.print(",  wheel = ");         Serial.print(mouse1_data_buffer[4]);
        Serial.print(",  wheelH = ");        Serial.println(mouse1_data_buffer[5]);
#endif 
#ifdef MOUSE_DATA_PI      
       /*if (mySerial.available())  // 采用次判断时会导致鼠标数据一部分包，没有发出去
       {   for (int i=0; i<6; i++)  { mySerial.write(mouse1_data_buffer[i]); }  }*/
       // mySerial.print("mouse1:");
       // for (int i=0; i<6; i++)  { mySerial.write(mouse1_data_buffer[i]); }
       mySerialWriteProcess(mouse1_data_buffer, mouse_data_size, false);
#endif
#ifdef MOUSE_DATA_FOREWARD 
        // getMouseX(),getMouseY()是相对位置的变化量, moveTo()是位置绝对量
        Mouse.move(mouse1.getMouseX(), mouse1.getMouseY());  
        Mouse.click(mouse1.getButtons());                    
        Mouse.scroll(mouse1.getWheel(),mouse1.getWheelH());  
#endif
        mouse1.mouseDataClear();            // Note here.  
        for (int i=0; i<6; i++)             // clear mouse1_data_buffer
        { mouse1_data_buffer[i] = 0x00; }       
    }
    // mouse2--------------------------------------------------------------------------   
    if (mouse2.available())
    {
        mouse2_data_buffer[0] = 0xA2;      // mouse2 identifier
        mouse2_data_buffer[1] = mouse2.getButtons();
        mouse2_data_buffer[2] = mouse2.getMouseX();
        mouse2_data_buffer[3] = mouse2.getMouseY();
        mouse2_data_buffer[4] = mouse2.getWheel();  
        mouse2_data_buffer[5] = mouse2.getWheelH();
#ifdef MOUSE_DATA_SERIALMONITOR
        Serial.print("Mouse2: buttons = ");   Serial.print(mouse2.getButtons());
        Serial.print(",  mouseX = ");         Serial.print(mouse2.getMouseX());
        Serial.print(",  mouseY = ");         Serial.print(mouse2.getMouseY());
        Serial.print(",  wheel = ");          Serial.print(mouse2.getWheel());
        Serial.print(",  wheelH = ");         Serial.println(mouse2.getWheelH());
#endif
#ifdef MOUSE_DATA_PI
       // mySerial.print("mouse2:");
       // for (int i=0; i<6; i++)   { mySerial.write(mouse2_data_buffer[i]); }
       mySerialWriteProcess(mouse2_data_buffer, mouse_data_size, false);
#endif
#ifdef MOUSE_DATA_FOREWARD 
        Mouse.move(mouse2.getMouseX(), mouse2.getMouseY());
        Mouse.click(mouse2.getButtons());
        Mouse.scroll(mouse2.getWheel(),mouse2.getWheelH());
#endif
        mouse2.mouseDataClear();          // Note here. 
        for (int i=0; i<6; i++)           // clear mouse2_data_buffer
        { mouse2_data_buffer[i] = 0x00;}         
    }
}


// ==================================================================================
// Keyboard
//         8 Bytes:  kbX=1/2, modifier= , oemkey=0xXX, LEDS=0Xxx, 后面4个字节空置留用
// ===================================================================================
#ifdef KEYBOARD_INTERFACE
uint8_t keyboard_last_leds = 0;
// uint8_t keyboard_modifiers = 0;
#elif !defined(KEYBOARD_DATA_SERIALMONITOR)
//#Warning: "USB type does not have Serial, so turning on KEYBOARD_DATA_SERIALMONITOR"
#define KEYBOARD_DATA_SERIALMONITOR
#endif
uint8_t keyboard_modifiers = 0;
uint8_t keyboard_leds_temp = 0;
const uint8_t keyboard_data_size = 8;
byte keyboard1_data_buffer[keyboard_data_size];  
byte keyboard2_data_buffer[keyboard_data_size]; 
void KeyboardDataProcess()
{
    // for identifier kb1 or kb2
    if(keyboard1.available())
    {
        // mySerial.printf("keyboard1 event");
        Serial.printf("keyboard1 event");
        Serial.println();
        #ifdef KEYBOARD_DATA_PI
        keyboard1_data_buffer[0] = 0xB1;                       // keyboard1 identifier -> 0xB1
        keyboard1_data_buffer[1] = keyboard1.getModifiers();
        keyboard1_data_buffer[2] = keyboard1.getOemKey();
        keyboard1_data_buffer[3] = keyboard1.LEDS();
        // for (int i=0; i<4; i++)   { mySerial.write(keyboard1_data_buffer[i]); }
        mySerialWriteProcess(keyboard1_data_buffer, keyboard_data_size, true);        
        #endif
        keyboard1.keyboardEventClear();
    }
    if(keyboard2.available())
    {
        // mySerial.printf("keyboard2 event");
        Serial.printf("keyboard2 event");
        Serial.println();
        #ifdef KEYBOARD_DATA_PI
        keyboard2_data_buffer[0] = 0xB2;                      // keyboard2 identifier -> 0xB2
        keyboard2_data_buffer[1] = keyboard2.getModifiers();
        keyboard2_data_buffer[2] = keyboard2.getOemKey();
        keyboard2_data_buffer[3] = keyboard2.LEDS();
        // for (int i=0; i<4; i++)   { mySerial.write(keyboard2_data_buffer[i]); } 
        mySerialWriteProcess(keyboard2_data_buffer, keyboard_data_size, true);       
        #endif
        keyboard2.keyboardEventClear();
    }
}
  
//show keyboard date by serialmonitor-------------------------------------
void OnPress(int key)
{
#ifdef KEYBOARD_DATA_SERIALMONITOR
    Serial.print("key：'");
    switch (key) 
    {
        case KEYD_UP       : Serial.print("UP"); break;
        case KEYD_DOWN     : Serial.print("DN"); break;
        case KEYD_LEFT     : Serial.print("LEFT"); break;
        case KEYD_RIGHT    : Serial.print("RIGHT"); break;
        case KEYD_INSERT   : Serial.print("Ins"); break;
        case KEYD_DELETE   : Serial.print("Del"); break;
        case KEYD_PAGE_UP  : Serial.print("PUP"); break;
        case KEYD_PAGE_DOWN: Serial.print("PDN"); break;
        case KEYD_HOME     : Serial.print("HOME"); break;
        case KEYD_END      : Serial.print("END"); break;
        case KEYD_F1       : Serial.print("F1"); break;
        case KEYD_F2       : Serial.print("F2"); break;
        case KEYD_F3       : Serial.print("F3"); break;
        case KEYD_F4       : Serial.print("F4"); break;
        case KEYD_F5       : Serial.print("F5"); break;
        case KEYD_F6       : Serial.print("F6"); break;
        case KEYD_F7       : Serial.print("F7"); break;
        case KEYD_F8       : Serial.print("F8"); break;
        case KEYD_F9       : Serial.print("F9"); break;
        case KEYD_F10      : Serial.print("F10"); break;
        case KEYD_F11      : Serial.print("F11"); break;
        case KEYD_F12      : Serial.print("F12"); break;                 
        case KEYD_Esc      : Serial.print("Esc"); break;  // added
        case KEYD_Tab      : Serial.print("Tab"); break;  // added
        // case KEYD_Win      : Serial.print("Win"); break;       // key=0???
        // case KEYD_CapsLock : Serial.print("CapsLock"); break;  // key=0???
        default            : Serial.print((char)key); break;
    }
    Serial.print("'");
    Serial.print("  keyvalue：0x");
    Serial.print(key, HEX);
    Serial.print("  MOD: 0x");
    if(keyboard1)
    {
        Serial.print(keyboard1.getModifiers(), HEX);    
        Serial.print("  OEM: 0x");
        Serial.print(keyboard1.getOemKey(), HEX);  
        Serial.print("  LEDS: 0x");
        Serial.println(keyboard1.LEDS(), HEX);
    } 
    if(keyboard2) 
    {
        Serial.print(keyboard2.getModifiers(), HEX);    
        Serial.print("  OEM: 0x");
        Serial.print(keyboard2.getOemKey(), HEX);  
        Serial.print("  LEDS: 0x");
        Serial.println(keyboard2.LEDS(), HEX);
    }
    //Serial.print("key ");
    //Serial.print((char)keyboard1.getKey());
    //Serial.print("  ");
    //Serial.print((char)keyboard2.getKey());
    //Serial.println();
#endif
}

void OnRawPress(uint8_t keycode) 
{
#ifdef KEYBOARD_INTERFACE
    if (keyboard_leds != keyboard_last_leds)
    {
        keyboard_leds_temp = keyboard_last_leds;
        keyboard_last_leds = keyboard_leds;
        if(keyboard1.LEDS() != keyboard_leds_temp)
        {
            Serial.printf("New LEDS from keyboard1: %x \n", keyboard_leds);    
            keyboard1.LEDS(keyboard_leds);
            //keyboard1_data_buffer[3] = keyboard_leds;
        }
        else if(keyboard2.LEDS() != keyboard_leds_temp) 
        {  
            Serial.printf("New LEDS from keyboard2: %x \n", keyboard_leds);  
            keyboard2.LEDS(keyboard_leds);
            //keyboard2_data_buffer[3] = keyboard_leds;
        }
    }   
    if (keycode >= 103 && keycode < 111)              // 103=0x67,  111=0x6F
    {   // one of the modifier keys was pressed, so lets turn it on global..
        uint8_t keybit = 1 << (keycode - 103);
        keyboard_modifiers |= keybit;
        Keyboard.set_modifier(keyboard_modifiers);   // press and hold modifiers
    } 
    else 
    {
        if (keyboard1.getModifiers() != keyboard_modifiers) 
        {   
            #ifdef KEYBOARD_DATA_SERIALMONITOR
            Serial.printf("Mods mismatch: %x != %x\n", keyboard_modifiers, keyboard1.getModifiers());
            #endif
            keyboard_modifiers = keyboard1.getModifiers();
            //keyboard1_data_buffer[1] = keyboard_modifiers;
            Keyboard.set_modifier(keyboard_modifiers);
        }
        else if (keyboard2.getModifiers() != keyboard_modifiers) 
        {   
            Serial.printf("Mods mismatch: %x != %x\n", keyboard_modifiers, keyboard2.getModifiers());
            keyboard_modifiers = keyboard2.getModifiers();
            //keyboard2_data_buffer[1] = keyboard_modifiers;
            Keyboard.set_modifier(keyboard_modifiers);
        }
        // 如何判断keycode 来自KB1还是KB2
        //keyboard1_data_buffer[2] = keycode;
        //keyboard2_data_buffer[2] = keycode;
        Keyboard.press(0XF000 | keycode);             // 0XF000 | keycode ???
    }
#endif
#ifdef KEYBOARD_DATA_SERIALMONITOR
    Serial.print("OnRawPress keycode: 0x");  
    Serial.print(keycode, HEX);    // keycode是标准的键值  例如当按下r时，keycode=0x15
    Serial.print(" Modifiers: 0x"); 
    // extern uint8_t keyboard_modifiers;
    Serial.println(keyboard_modifiers, HEX);
#endif
}


// OnRawRelease(),  parameter:keycode --------------------------------------
void OnRawRelease(uint8_t keycode) 
{
#ifdef KEYBOARD_INTERFACE
    if (keycode >= 103 && keycode < 111) {
        // one of the modifier keys was pressed, so lets turn it on global..
        uint8_t keybit = 1 << (keycode - 103);
        keyboard_modifiers &= ~keybit;
        //keyboard1_data_buffer[1] = keyboard_modifiers;
        //keyboard2_data_buffer[1] = keyboard_modifiers;
        Keyboard.set_modifier(keyboard_modifiers);
    } 
    else {
        //keyboard1_data_buffer[2] = keyboard1.getOemKey();
        //keyboard2_data_buffer[2] = keyboard2.getOemKey();
        Keyboard.release(0XF000 | keycode);  //
    }
#endif
#ifdef KEYBOARD_DATA_SERIALMONITOR
    Serial.print("OnRawRelease keycode: 0x");
    Serial.print(keycode, HEX);
    Serial.print(" Modifiers: 0x");
    Serial.println(keyboard1.getModifiers(), HEX);
#endif
}


// OnHIDExtrasPress(),   parameter:top, key------------------------------------
void OnHIDExtrasPress(uint32_t top, uint16_t key)
{
#ifdef KEYBOARD_INTERFACE
    if (top == 0xc0000) 
    {
        Keyboard.press(0XE400 | key);  // ???
        #ifndef KEYMEDIA_INTERFACE
        #error "KEYMEDIA_INTERFACE is Not defined"
        #endif
    }
#endif
    
#ifdef KEYBOARD_DATA_SERIALMONITOR
    ShowHIDExtrasPress(top, key);
#endif
}


// OnHIDExtrasRelease(), parameter:top, key  -----------------------------------
void OnHIDExtrasRelease(uint32_t top, uint16_t key)
{
#ifdef KEYBOARD_INTERFACE
    if (top == 0xc0000) 
    {
        Keyboard.release(0XE400 | key);
    }
#endif

#ifdef KEYBOARD_DATA_SERIALMONITOR
    Serial.print("HID (");
    Serial.print(top, HEX);
    Serial.print(") key release:");
    Serial.println(key, HEX);
#endif
}

byte rawhid_in_data_buffer[256];
byte rawhid_out_data_buffer[64];

// OnReceiveHidData(),  rawhid , parameter -------------------------------------------------------------
bool OnReceiveHidData(uint32_t usage, const uint8_t *data, uint32_t len) 
{
    // Called for maybe both HIDS for rawhid basic test.  One is for the Teensy
    // to output to Serial. while still having Raw Hid...

    // Serial.println("TEST HEARTBEAT");

    if (usage == 0xffc90004)
    {
        // Lets trim off trailing null characters.
        while ((len > 0) && (data[len - 1] == 0)){
            len--;
        }
        if (len){
            Serial.print("RawHid Serial: ");
            Serial.write(data, len);
        }
    } 
    else //RUBBEY DUKY HID kb, len=8, rawhid,是键值报告; BadUSB HID kb, len=64, rawhid,前八个字节是键值报告 
    {
        Serial.print("rawhid data(dev->t4.1):  size:0x");
        Serial.print(len, HEX);
        Serial.print("  usage:");   
        Serial.println(usage, HEX);

        #ifdef RAWHID_DATA_PI
        const uint8_t *ip;
        ip = data;
        byte data_buffer[256];
        uint8_t data_buffer_size = len;
        data_buffer[0] = 0xCF;  // rawhid data(t4.1->pi) identifier -> 0xCF
        for(int i=(0+1); i<(data_buffer_size+1); i++){
            data_buffer[i] = *ip;
            ip++;
            // Serial.println(data_buffer[i]);
        }
        mySerialWriteProcess(data_buffer, (data_buffer_size+1), false);
        Serial.print("rawhid data(t4.1->pi): size:0x");
        Serial.println(data_buffer_size+1, HEX);
        #endif

        #ifdef RAWHID_DATA_FOREWARD
        int tx_packet_len = 0;
        tx_packet_len = RawHID.send(data, 1);  //======================================
        if(tx_packet_len!=0)  {
            Serial.print("rawhid data(t4.1->pc): size:0x");
            Serial.println(tx_packet_len, HEX);
        }
        else {
            Serial.println(F("RAWHID_DATA_FOREWARD: Unable to transmit packet"));
        }
        #endif 

        #ifdef RAWHID_DATA_SERIALMONITOR   
        Serial.print("rawhid data(dev->t4.1): ");
        Serial.println();
        uint32_t len_t = len;
        while (len_t) {
            uint8_t cb = (len_t > 16) ? 16 : len_t;  // 条件运算符Exp1 ? Exp2 : Exp3
            const uint8_t *p = data;
            for (uint8_t i = 0; i < cb; i++) {
                Serial.printf("%02x ", *p++);
            }
            Serial.print(": ");
            for (uint8_t i = 0; i < cb; i++) {
                Serial.write(((*data >= ' ') && (*data <= '~')) ? *data : '.');
                data++;
            }
            len_t -= cb;
            Serial.println();
        }
        #endif   
  }
  return true;
}


//=============================================================
// Device and Keyboard Output To Serial objects...  20220315 修改.
// 可以添加hub,keyboard,hid,但是无法直接添加HIDMouse,USBDriver无法驱动MouseController.
//=============================================================
#ifdef KEYBOARD_DATA_SERIALMONITOR
USBDriver *drivers[] = {&hub1, &hub2,  &hub3, &hub4, &keyboard1,  &keyboard2, &hid1, &hid2, &hid3, &hid4, &hid5, &hid6}; //指针数组？  结构体，USBDriver是基类（USBDriver继承了USBHost）  
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))  // count of devices
const char * driver_names[CNT_DEVICES] = {"Hub1", "Hub2", "Hub3", "Hub4", "KB1",  "KB2", "HID1", "HID2", "HID3", "HID4", "HID5", "HID6"};
bool driver_active[CNT_DEVICES] = {false, false, false, false, false, false, false, false, false, false, false, false};  // 连接状态，false默认无连接
bool driver_active0[CNT_DEVICES] = {false, false, false, false, false, false, false, false, false, false, false, false};  // For tft show 
#endif


// ShowUpdatedDeviceListInfo()------------------------------------------------------------------------------------------------------
void ShowUpdatedDeviceListInfo()
{
    // Serial显示HUB和键盘设备枚举信息
#ifdef KEYBOARD_DATA_SERIALMONITOR
    for (uint8_t i = 0; i < CNT_DEVICES; i++) // 遍历每一个设备
    {
        if (*drivers[i] != driver_active[i]) 
        {
            if (driver_active[i]) // 设备断开
            {
                Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
                driver_active[i] = false;
            } 
            else // 设备连接
            { 
                // *drivers[i]是结构体指针，drivers[i]是一个结构体，idVendor()是结构体里的一个变量（函数），就是读取VID的值
                Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct()); 
                // ooutput:  *** Device KB 413c:2113 - connected ***
                driver_active[i] = true;
                      
                const uint8_t *psz = drivers[i]->manufacturer();
                if (psz && *psz)                                  // 判断设备厂商信息非空,只有在非空条件下才输出
                  Serial.printf("  manufacturer: %s\n", psz);
                psz = drivers[i]->product();
                if (psz && *psz) 
                  Serial.printf("  product: %s\n", psz);
                psz = drivers[i]->serialNumber();
                if (psz && *psz) 
                  Serial.printf("  Serial: %s\n", psz);
                
                // Note: with some keyboards there is an issue that they don't output in boot protocol mode
                // and may not work.  The above code can try to force the keyboard into boot mode, but there
                // are issues with doing this blindly with combo devices like wireless keyboard/mouse, which
                // may cause the mouse to not work.  Note: the above id is in the builtin list of
                // vendor IDs that are already forced
                if (drivers[i] == &keyboard1) 
                {
                    if (keyboard1.idVendor() == 0x04D9) 
                    {
                        Serial.println("Gigabyte vendor: force boot protocol");
                        // Gigabyte keyboard
                        keyboard1.forceBootProtocol();
                    }
                }
            }
        }
    }
#endif

    // tft显示HUB和键盘设备枚举信息
#ifdef USE_ST77XX
    for (uint8_t j = 0; j < CNT_DEVICES; j++)
    {
        if (*drivers[j] != driver_active0[j]) 
        {
            if (driver_active0[j])
            {
                tft.printf("*Device %s - disconnected*\n", driver_names[j]);
                driver_active0[j] = false;
            } 
            else
            { 
                tft.printf("*Device %s %x:%x - connected*\n", driver_names[j], drivers[j]->idVendor(), drivers[j]->idProduct());
                driver_active0[j] = true;             
                const uint8_t *psz0 = drivers[j]->manufacturer();
                if (psz0 && *psz0)
                  tft.printf("  manufacturer: %s\n", psz0);
                psz0 = drivers[j]->product();
                if (psz0 && *psz0) 
                  tft.printf("  product: %s\n", psz0);
                psz0 = drivers[j]->serialNumber();
                if (psz0 && *psz0) 
                  tft.printf("  Serial: %s\n", psz0);
                  
                if (drivers[j] == &keyboard1) 
                {
                    if (keyboard1.idVendor() == 0x04D9) 
                    {
                        tft.println("Gigabyte vendor: force boot protocol");
                        // Gigabyte keyboard
                        keyboard1.forceBootProtocol();
                    }
                }
            }
        }
    }
#endif
}



//=============================================================
// 0315, HIDMouse Output To Serial objects...     
//=============================================================
#ifdef MOUSE_DATA_SERIALMONITOR
USBHIDInput *hiddrivers[] = {&mouse1, &mouse2, &rawhid1, &rawhid2, &rawhid3, &rawhid4};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_HIDDEVICES] = {"MS1", "MS2", "RAWHID1", "RAWHID2", "RAWHID3", "RAWHID4"};      // CNT_DEVICES这个变量搞得不对吧   这三行
bool hid_driver_active[CNT_HIDDEVICES] = {false, false, false, false, false, false};
bool hid_driver_active0[CNT_HIDDEVICES] = {false, false, false, false, false, false};  // for tft show,连接状态标志位
bool show_changed_only = false;
#endif

// serial显示鼠标枚举信息
//--------------------------------------------------------------------------------------------------------------------------------------------
void ShowUpdatedHIDMouseDeviceListInfo()
{
#ifdef MOUSE_DATA_SERIALMONITOR
    for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) 
    {
        if (*hiddrivers[i] != hid_driver_active[i]) 
        {
            if (hid_driver_active[i]) 
            {
                Serial.printf("*** Device %s - disconnected ***\n", hid_driver_names[i]);
                hid_driver_active[i] = false;
            } 
            else 
            {
                Serial.printf("*** Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
                hid_driver_active[i] = true;
                const uint8_t *psz = hiddrivers[i]->manufacturer();
                if (psz && *psz)   
                    Serial.printf("  manufacturer: %s\n", psz);
                psz = hiddrivers[i]->product();
                if (psz && *psz)   
                    Serial.printf("  product: %s\n", psz);
                psz = hiddrivers[i]->serialNumber();
                if (psz && *psz)   
                    Serial.printf("  Serial: %s\n", psz);
            }
        }
    }
#endif

// tft显示鼠标枚举信息
#ifdef USE_ST77XX
    for (uint8_t j = 0; j < CNT_HIDDEVICES; j++)
    {
        if (*hiddrivers[j] != hid_driver_active0[j]) 
        {
            if (hid_driver_active0[j])
            {
                tft.printf("*Device %s - disconnected*\n", hid_driver_names[j]);
                hid_driver_active0[j] = false;
            } 
            else
            { 
                tft.printf("*Device %s %x:%x - connected*\n", hid_driver_names[j], hiddrivers[j]->idVendor(), hiddrivers[j]->idProduct());
                hid_driver_active0[j] = true;             
                const uint8_t *psz0 = hiddrivers[j]->manufacturer();
                if (psz0 && *psz0)
                    tft.printf("  manufacturer: %s\n", psz0);
                psz0 = hiddrivers[j]->product();
                if (psz0 && *psz0) 
                    tft.printf("  product: %s\n", psz0);
                psz0 = hiddrivers[j]->serialNumber();
                if (psz0 && *psz0) 
                    tft.printf("  Serial: %s\n", psz0);
            }
        }
    }
#endif
}


/*
//=============================================================
// 0426, MSC   
//=============================================================
//#ifdef MOUSE_DATA_SERIALMONITOR
USBHIDInput *hiddrivers[] = {&msDrive1, &msDrive2};
#define CNT_msDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * ms_driver_names[CNT_DEVICES] = {"ms1", "ms2"};
bool ms_driver_active[CNT_DEVICES] = {false, false};
bool ms_driver_active0[CNT_DEVICES] = {false, false};  // for tft show,连接状态标志位
bool show_changed_only = false;
#endif

// serial显示msc枚举信息
// Show USB drive information for the selected USB drive.  //for msc
int showUSBmsDriveInfo(msController *drive) 
{
    if(drive == &msDrive1) 
    {
        Serial.printf(F("msDrive1 is "));
    } 
    else 
    {
        Serial.printf(F("msDrive2 is "));
    }
    if(drive->msDriveInfo.mounted == true) 
    {   // check if mounted.
        Serial.printf(F("Mounted\n\n"));
    } 
    else 
    {
        Serial.printf(F("NOT Mounted\n\n"));
    }
    
    // Now we will print out the information.
    Serial.printf(F("   connected %d\n"),drive->msDriveInfo.connected);
    Serial.printf(F("   initialized %d\n"),drive->msDriveInfo.initialized);
    Serial.printf(F("   USB Vendor ID: %4.4x\n"),drive->msDriveInfo.idVendor);
    Serial.printf(F("  USB Product ID: %4.4x\n"),drive->msDriveInfo.idProduct);
    Serial.printf(F("      HUB Number: %d\n"),drive->msDriveInfo.hubNumber);
    Serial.printf(F("        HUB Port: %d\n"),drive->msDriveInfo.hubPort);
    Serial.printf(F("  Device Address: %d\n"),drive->msDriveInfo.deviceAddress);
    Serial.printf(F("Removable Device: "));
    
    if(drive->msDriveInfo.inquiry.Removable == 1)
    {  
        Serial.printf(F("YES\n"));
    }
    else
    {
        Serial.printf(F("NO\n"));
    }
    
    Serial.printf(F("        VendorID: %8.8s\n"),drive->msDriveInfo.inquiry.VendorID);
    Serial.printf(F("       ProductID: %16.16s\n"),drive->msDriveInfo.inquiry.ProductID);
    Serial.printf(F("      RevisionID: %4.4s\n"),drive->msDriveInfo.inquiry.RevisionID);
    Serial.printf(F("         Version: %d\n"),drive->msDriveInfo.inquiry.Version);
    Serial.printf(F("    Sector Count: %ld\n"),drive->msDriveInfo.capacity.Blocks);
    Serial.printf(F("     Sector size: %ld\n"),drive->msDriveInfo.capacity.BlockSize);
    Serial.printf(F("   Disk Capacity: %.f Bytes\n\n"),(double_t)drive->msDriveInfo.capacity.Blocks *
                      (double_t)drive->msDriveInfo.capacity.BlockSize);
    return 0;
}
static uint8_t mscError = 0;
*/


// 媒体播放器等设备的按键
void ShowHIDExtrasPress(uint32_t top, uint16_t key)
{
#ifdef KEYBOARD_DATA_SERIALMONITOR
  Serial.print("HID (");
  Serial.print(top, HEX);
  Serial.print(") key press:");
  Serial.print(key, HEX);
  if (top == 0xc0000) 
  {
    switch (key) 
    {
      case  0x20 : Serial.print(" - +10"); break;
      case  0x21 : Serial.print(" - +100"); break;
      case  0x22 : Serial.print(" - AM/PM"); break;
      case  0x30 : Serial.print(" - Power"); break;
      case  0x31 : Serial.print(" - Reset"); break;
      case  0x32 : Serial.print(" - Sleep"); break;
      case  0x33 : Serial.print(" - Sleep After"); break;
      case  0x34 : Serial.print(" - Sleep Mode"); break;
      case  0x35 : Serial.print(" - Illumination"); break;
      case  0x36 : Serial.print(" - Function Buttons"); break;
      case  0x40 : Serial.print(" - Menu"); break;
      case  0x41 : Serial.print(" - Menu  Pick"); break;
      case  0x42 : Serial.print(" - Menu Up"); break;
      case  0x43 : Serial.print(" - Menu Down"); break;
      case  0x44 : Serial.print(" - Menu Left"); break;
      case  0x45 : Serial.print(" - Menu Right"); break;
      case  0x46 : Serial.print(" - Menu Escape"); break;
      case  0x47 : Serial.print(" - Menu Value Increase"); break;
      case  0x48 : Serial.print(" - Menu Value Decrease"); break;
      case  0x60 : Serial.print(" - Data On Screen"); break;
      case  0x61 : Serial.print(" - Closed Caption"); break;
      case  0x62 : Serial.print(" - Closed Caption Select"); break;
      case  0x63 : Serial.print(" - VCR/TV"); break;
      case  0x64 : Serial.print(" - Broadcast Mode"); break;
      case  0x65 : Serial.print(" - Snapshot"); break;
      case  0x66 : Serial.print(" - Still"); break;
      case  0x80 : Serial.print(" - Selection"); break;
      case  0x81 : Serial.print(" - Assign Selection"); break;
      case  0x82 : Serial.print(" - Mode Step"); break;
      case  0x83 : Serial.print(" - Recall Last"); break;
      case  0x84 : Serial.print(" - Enter Channel"); break;
      case  0x85 : Serial.print(" - Order Movie"); break;
      case  0x86 : Serial.print(" - Channel"); break;
      case  0x87 : Serial.print(" - Media Selection"); break;
      case  0x88 : Serial.print(" - Media Select Computer"); break;
      case  0x89 : Serial.print(" - Media Select TV"); break;
      case  0x8A : Serial.print(" - Media Select WWW"); break;
      case  0x8B : Serial.print(" - Media Select DVD"); break;
      case  0x8C : Serial.print(" - Media Select Telephone"); break;
      case  0x8D : Serial.print(" - Media Select Program Guide"); break;
      case  0x8E : Serial.print(" - Media Select Video Phone"); break;
      case  0x8F : Serial.print(" - Media Select Games"); break;
      case  0x90 : Serial.print(" - Media Select Messages"); break;
      case  0x91 : Serial.print(" - Media Select CD"); break;
      case  0x92 : Serial.print(" - Media Select VCR"); break;
      case  0x93 : Serial.print(" - Media Select Tuner"); break;
      case  0x94 : Serial.print(" - Quit"); break;
      case  0x95 : Serial.print(" - Help"); break;
      case  0x96 : Serial.print(" - Media Select Tape"); break;
      case  0x97 : Serial.print(" - Media Select Cable"); break;
      case  0x98 : Serial.print(" - Media Select Satellite"); break;
      case  0x99 : Serial.print(" - Media Select Security"); break;
      case  0x9A : Serial.print(" - Media Select Home"); break;
      case  0x9B : Serial.print(" - Media Select Call"); break;
      case  0x9C : Serial.print(" - Channel Increment"); break;
      case  0x9D : Serial.print(" - Channel Decrement"); break;
      case  0x9E : Serial.print(" - Media Select SAP"); break;
      case  0xA0 : Serial.print(" - VCR Plus"); break;
      case  0xA1 : Serial.print(" - Once"); break;
      case  0xA2 : Serial.print(" - Daily"); break;
      case  0xA3 : Serial.print(" - Weekly"); break;
      case  0xA4 : Serial.print(" - Monthly"); break;
      case  0xB0 : Serial.print(" - Play"); break;
      case  0xB1 : Serial.print(" - Pause"); break;
      case  0xB2 : Serial.print(" - Record"); break;
      case  0xB3 : Serial.print(" - Fast Forward"); break;
      case  0xB4 : Serial.print(" - Rewind"); break;
      case  0xB5 : Serial.print(" - Scan Next Track"); break;
      case  0xB6 : Serial.print(" - Scan Previous Track"); break;
      case  0xB7 : Serial.print(" - Stop"); break;
      case  0xB8 : Serial.print(" - Eject"); break;
      case  0xB9 : Serial.print(" - Random Play"); break;
      case  0xBA : Serial.print(" - Select DisC"); break;
      case  0xBB : Serial.print(" - Enter Disc"); break;
      case  0xBC : Serial.print(" - Repeat"); break;
      case  0xBD : Serial.print(" - Tracking"); break;
      case  0xBE : Serial.print(" - Track Normal"); break;
      case  0xBF : Serial.print(" - Slow Tracking"); break;
      case  0xC0 : Serial.print(" - Frame Forward"); break;
      case  0xC1 : Serial.print(" - Frame Back"); break;
      case  0xC2 : Serial.print(" - Mark"); break;
      case  0xC3 : Serial.print(" - Clear Mark"); break;
      case  0xC4 : Serial.print(" - Repeat From Mark"); break;
      case  0xC5 : Serial.print(" - Return To Mark"); break;
      case  0xC6 : Serial.print(" - Search Mark Forward"); break;
      case  0xC7 : Serial.print(" - Search Mark Backwards"); break;
      case  0xC8 : Serial.print(" - Counter Reset"); break;
      case  0xC9 : Serial.print(" - Show Counter"); break;
      case  0xCA : Serial.print(" - Tracking Increment"); break;
      case  0xCB : Serial.print(" - Tracking Decrement"); break;
      case  0xCD : Serial.print(" - Pause/Continue"); break;
      case  0xE0 : Serial.print(" - Volume"); break;
      case  0xE1 : Serial.print(" - Balance"); break;
      case  0xE2 : Serial.print(" - Mute"); break;
      case  0xE3 : Serial.print(" - Bass"); break;
      case  0xE4 : Serial.print(" - Treble"); break;
      case  0xE5 : Serial.print(" - Bass Boost"); break;
      case  0xE6 : Serial.print(" - Surround Mode"); break;
      case  0xE7 : Serial.print(" - Loudness"); break;
      case  0xE8 : Serial.print(" - MPX"); break;
      case  0xE9 : Serial.print(" - Volume Up"); break;
      case  0xEA : Serial.print(" - Volume Down"); break;
      case  0xF0 : Serial.print(" - Speed Select"); break;
      case  0xF1 : Serial.print(" - Playback Speed"); break;
      case  0xF2 : Serial.print(" - Standard Play"); break;
      case  0xF3 : Serial.print(" - Long Play"); break;
      case  0xF4 : Serial.print(" - Extended Play"); break;
      case  0xF5 : Serial.print(" - Slow"); break;
      case  0x100: Serial.print(" - Fan Enable"); break;
      case  0x101: Serial.print(" - Fan Speed"); break;
      case  0x102: Serial.print(" - Light"); break;
      case  0x103: Serial.print(" - Light Illumination Level"); break;
      case  0x104: Serial.print(" - Climate Control Enable"); break;
      case  0x105: Serial.print(" - Room Temperature"); break;
      case  0x106: Serial.print(" - Security Enable"); break;
      case  0x107: Serial.print(" - Fire Alarm"); break;
      case  0x108: Serial.print(" - Police Alarm"); break;
      case  0x150: Serial.print(" - Balance Right"); break;
      case  0x151: Serial.print(" - Balance Left"); break;
      case  0x152: Serial.print(" - Bass Increment"); break;
      case  0x153: Serial.print(" - Bass Decrement"); break;
      case  0x154: Serial.print(" - Treble Increment"); break;
      case  0x155: Serial.print(" - Treble Decrement"); break;
      case  0x160: Serial.print(" - Speaker System"); break;
      case  0x161: Serial.print(" - Channel Left"); break;
      case  0x162: Serial.print(" - Channel Right"); break;
      case  0x163: Serial.print(" - Channel Center"); break;
      case  0x164: Serial.print(" - Channel Front"); break;
      case  0x165: Serial.print(" - Channel Center Front"); break;
      case  0x166: Serial.print(" - Channel Side"); break;
      case  0x167: Serial.print(" - Channel Surround"); break;
      case  0x168: Serial.print(" - Channel Low Frequency Enhancement"); break;
      case  0x169: Serial.print(" - Channel Top"); break;
      case  0x16A: Serial.print(" - Channel Unknown"); break;
      case  0x170: Serial.print(" - Sub-channel"); break;
      case  0x171: Serial.print(" - Sub-channel Increment"); break;
      case  0x172: Serial.print(" - Sub-channel Decrement"); break;
      case  0x173: Serial.print(" - Alternate Audio Increment"); break;
      case  0x174: Serial.print(" - Alternate Audio Decrement"); break;
      case  0x180: Serial.print(" - Application Launch Buttons"); break;
      case  0x181: Serial.print(" - AL Launch Button Configuration Tool"); break;
      case  0x182: Serial.print(" - AL Programmable Button Configuration"); break;
      case  0x183: Serial.print(" - AL Consumer Control Configuration"); break;
      case  0x184: Serial.print(" - AL Word Processor"); break;
      case  0x185: Serial.print(" - AL Text Editor"); break;
      case  0x186: Serial.print(" - AL Spreadsheet"); break;
      case  0x187: Serial.print(" - AL Graphics Editor"); break;
      case  0x188: Serial.print(" - AL Presentation App"); break;
      case  0x189: Serial.print(" - AL Database App"); break;
      case  0x18A: Serial.print(" - AL Email Reader"); break;
      case  0x18B: Serial.print(" - AL Newsreader"); break;
      case  0x18C: Serial.print(" - AL Voicemail"); break;
      case  0x18D: Serial.print(" - AL Contacts/Address Book"); break;
      case  0x18E: Serial.print(" - AL Calendar/Schedule"); break;
      case  0x18F: Serial.print(" - AL Task/Project Manager"); break;
      case  0x190: Serial.print(" - AL Log/Journal/Timecard"); break;
      case  0x191: Serial.print(" - AL Checkbook/Finance"); break;
      case  0x192: Serial.print(" - AL Calculator"); break;
      case  0x193: Serial.print(" - AL A/V Capture/Playback"); break;
      case  0x194: Serial.print(" - AL Local Machine Browser"); break;
      case  0x195: Serial.print(" - AL LAN/WAN Browser"); break;
      case  0x196: Serial.print(" - AL Internet Browser"); break;
      case  0x197: Serial.print(" - AL Remote Networking/ISP Connect"); break;
      case  0x198: Serial.print(" - AL Network Conference"); break;
      case  0x199: Serial.print(" - AL Network Chat"); break;
      case  0x19A: Serial.print(" - AL Telephony/Dialer"); break;
      case  0x19B: Serial.print(" - AL Logon"); break;
      case  0x19C: Serial.print(" - AL Logoff"); break;
      case  0x19D: Serial.print(" - AL Logon/Logoff"); break;
      case  0x19E: Serial.print(" - AL Terminal Lock/Screensaver"); break;
      case  0x19F: Serial.print(" - AL Control Panel"); break;
      case  0x1A0: Serial.print(" - AL Command Line Processor/Run"); break;
      case  0x1A1: Serial.print(" - AL Process/Task Manager"); break;
      case  0x1A2: Serial.print(" - AL Select Tast/Application"); break;
      case  0x1A3: Serial.print(" - AL Next Task/Application"); break;
      case  0x1A4: Serial.print(" - AL Previous Task/Application"); break;
      case  0x1A5: Serial.print(" - AL Preemptive Halt Task/Application"); break;
      case  0x200: Serial.print(" - Generic GUI Application Controls"); break;
      case  0x201: Serial.print(" - AC New"); break;
      case  0x202: Serial.print(" - AC Open"); break;
      case  0x203: Serial.print(" - AC Close"); break;
      case  0x204: Serial.print(" - AC Exit"); break;
      case  0x205: Serial.print(" - AC Maximize"); break;
      case  0x206: Serial.print(" - AC Minimize"); break;
      case  0x207: Serial.print(" - AC Save"); break;
      case  0x208: Serial.print(" - AC Print"); break;
      case  0x209: Serial.print(" - AC Properties"); break;
      case  0x21A: Serial.print(" - AC Undo"); break;
      case  0x21B: Serial.print(" - AC Copy"); break;
      case  0x21C: Serial.print(" - AC Cut"); break;
      case  0x21D: Serial.print(" - AC Paste"); break;
      case  0x21E: Serial.print(" - AC Select All"); break;
      case  0x21F: Serial.print(" - AC Find"); break;
      case  0x220: Serial.print(" - AC Find and Replace"); break;
      case  0x221: Serial.print(" - AC Search"); break;
      case  0x222: Serial.print(" - AC Go To"); break;
      case  0x223: Serial.print(" - AC Home"); break;
      case  0x224: Serial.print(" - AC Back"); break;
      case  0x225: Serial.print(" - AC Forward"); break;
      case  0x226: Serial.print(" - AC Stop"); break;
      case  0x227: Serial.print(" - AC Refresh"); break;
      case  0x228: Serial.print(" - AC Previous Link"); break;
      case  0x229: Serial.print(" - AC Next Link"); break;
      case  0x22A: Serial.print(" - AC Bookmarks"); break;
      case  0x22B: Serial.print(" - AC History"); break;
      case  0x22C: Serial.print(" - AC Subscriptions"); break;
      case  0x22D: Serial.print(" - AC Zoom In"); break;
      case  0x22E: Serial.print(" - AC Zoom Out"); break;
      case  0x22F: Serial.print(" - AC Zoom"); break;
      case  0x230: Serial.print(" - AC Full Screen View"); break;
      case  0x231: Serial.print(" - AC Normal View"); break;
      case  0x232: Serial.print(" - AC View Toggle"); break;
      case  0x233: Serial.print(" - AC Scroll Up"); break;
      case  0x234: Serial.print(" - AC Scroll Down"); break;
      case  0x235: Serial.print(" - AC Scroll"); break;
      case  0x236: Serial.print(" - AC Pan Left"); break;
      case  0x237: Serial.print(" - AC Pan Right"); break;
      case  0x238: Serial.print(" - AC Pan"); break;
      case  0x239: Serial.print(" - AC New Window"); break;
      case  0x23A: Serial.print(" - AC Tile Horizontally"); break;
      case  0x23B: Serial.print(" - AC Tile Vertically"); break;
      case  0x23C: Serial.print(" - AC Format"); break;
    }
  }
  Serial.println();
#endif
}




 /*
    //msc
    if(mscError = msDrive1.mscInit())
        Serial.printf(F("msDrive1 not connected: Code: %d\n\n"),  mscError);
    else
        Serial.printf(F("msDrive1  connected\n"));
    
    if(mscError = msDrive2.mscInit())
        Serial.printf(F("msDrive2 not connected: Code: %d\n\n"),  mscError);
    else
        Serial.printf(F("msDrive2  connected\n"));
    */

        /*
        char op = 0;
        Serial.printf(F("\nPress a key to show USB drive info:\n\n"));
      
        while(!Serial.available()) 
            yield();
        op = Serial.read();
        if(Serial.available()) 
            Serial.read(); // Get rid of CR or LF if there.
        */

        /*
        // Check if msDrive1 is plugged in and initialized
        if((mscError = msDrive1.checkConnectedInitialized()) != MS_INIT_PASS) 
        {
            Serial.printf(F("msDrive1 not connected: Code: %d\n\n"),  mscError);
        } 
        else 
        {
            Serial.printf(F("msDrive1  connected/initilized\n"));
            showUSBDriveInfo(&msDrive1);
        }
        // Check if msDrive2 is plugged in and initialized
        if((mscError = msDrive2.checkConnectedInitialized()) != MS_INIT_PASS) 
        {
            Serial.printf(F("msDrive2 not connected: Code: %d\n\n"),  mscError);
        } 
        else 
        {
            Serial.printf(F("msDrive2  connected/initilized\n"));
            showUSBDriveInfo(&msDrive2);
        }
        */



    // mySerial test ============================================================================ OK
    // raspberrypi -> teessy --------------------------------------------OK
    /*if (mySerial.available())           // return int    >= 10
    {       
        Serial.write(mySerial.read());
        
    }*/      
    // teessy -> raspberrypi --------------------------------------------OK
    /*if (mySerial.available())
    {
        byte byte_temp = 0xFF;  // OK
        char char_temp[10] = {'A', 'B', 'A', 'B', 'C', 'A', 'B', 'C', 'D'};  // OK
        for (int i=0; i<8; i++)
        {
            mySerial.write(char_temp[i]);  // mySerial.read()
        }
    }*/



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 键鼠数据对照：
/* mouse 
 * BYTE1 BYTE2 BYTE3 BYTE4
 * BYTE1 -- 
       |--bit7:   1   表示   Y   坐标的变化量超出－256   ~   255的范围,0表示没有溢出  
       |--bit6:   1   表示   X   坐标的变化量超出－256   ~   255的范围，0表示没有溢出  
       |--bit5:   Y   坐标变化的符号位，1表示负数，即鼠标向下移动  
       |--bit4:   X   坐标变化的符号位，1表示负数，即鼠标向左移动  
       |--bit3:     恒为1  
       |--bit2:     1表示中键按下  
       |--bit1:     1表示右键按下  
       |--bit0:     1表示左键按下  
   BYTE2 -- X坐标变化量，与byte的bit4组成9位符号数,负数表示向左移，正数表右移。用补码表示变化量  
   BYTE3 -- Y坐标变化量，与byte的bit5组成9位符号数，负数表示向下移，正数表上移。用补码表示变化量 
   BYTE4 -- 滚轮变化
*/


/* keyboard
 * BYTE1 BYTE2 BYTE3 BYTE4 BYTE5 BYTE6 BYTE7 BYTE8 
 * BYTE1 -- 
       |--bit0:   Left Control是否按下，按下为1  
       |--bit1:   Left Shift  是否按下，按下为1  
       |--bit2:   Left Alt    是否按下，按下为1  
       |--bit3:   Left GUI    是否按下，按下为1  
       |--bit4:   Right Control是否按下，按下为1   
       |--bit5:   Right Shift 是否按下，按下为1  
       |--bit6:   Right Alt   是否按下，按下为1  
       |--bit7:   Right GUI   是否按下，按下为1 
   BYTE3 --第一列10进制键值，第二列16进制键值，第四列是按键
0 00 Reserved (no event indicated)9 N/A √ √ √ 4/101/104
1 01 Keyboard ErrorRollOver9 N/A √ √ √ 4/101/104
2 02 Keyboard POSTFail9 N/A √ √ √ 4/101/104
3 03 Keyboard ErrorUndefined9 N/A √ √ √ 4/101/104
4 04 Keyboard a and A4 31 √ √ √ 4/101/104
5 05 Keyboard b and B 50 √ √ √ 4/101/104
6 06 Keyboard c and C4 48 √ √ √ 4/101/104
7 07 Keyboard d and D 33 √ √ √ 4/101/104
8 08 Keyboard e and E 19 √ √ √ 4/101/104
9 09 Keyboard f and F 34 √ √ √ 4/101/104
10 0A Keyboard g and G 35 √ √ √ 4/101/104
11 0B Keyboard h and H 36 √ √ √ 4/101/104
12 0C Keyboard i and I 24 √ √ √ 4/101/104
13 0D Keyboard j and J 37 √ √ √ 4/101/104
14 0E Keyboard k and K 38 √ √ √ 4/101/104
15 0F Keyboard l and L 39 √ √ √ 4/101/104
16 10 Keyboard m and M4 52 √ √ √ 4/101/104
17 11 Keyboard n and N 51 √ √ √ 4/101/104
18 12 Keyboard o and O4 25 √ √ √ 4/101/104
19 13 Keyboard p and P4 26 √ √ √ 4/101/104
20 14 Keyboard q and Q4 17 √ √ √ 4/101/104
21 15 Keyboard r and R 20 √ √ √ 4/101/104
22 16 Keyboard s and S4 32 √ √ √ 4/101/104
23 17 Keyboard t and T 21 √ √ √ 4/101/104
24 18 Keyboard u and U 23 √ √ √ 4/101/104
25 19 Keyboard v and V 49 √ √ √ 4/101/104
26 1A Keyboard w and W4 18 √ √ √ 4/101/104
27 1B Keyboard x and X4 47 √ √ √ 4/101/104
28 1C Keyboard y and Y4 22 √ √ √ 4/101/104
29 1D Keyboard z and Z4 46 √ √ √ 4/101/104
30 1E Keyboard 1 and !4 2 √ √ √ 4/101/104
31 1F Keyboard 2 and @4 3 √ √ √ 4/101/104
32 20 Keyboard 3 and #4 4 √ √ √ 4/101/104
33 21 Keyboard 4 and $4 5 √ √ √ 4/101/104
34 22 Keyboard 5 and %4 6 √ √ √ 4/101/104
35 23 Keyboard 6 and ^4 7 √ √ √ 4/101/104
36 24 Keyboard 7 and &4 8 √ √ √ 4/101/104
37 25 Keyboard 8 and *4 9 √ √ √ 4/101/104
38 26 Keyboard 9 and (4 10 √ √ √ 4/101/104
39 27 Keyboard 0 and )4 11 √ √ √ 4/101/104
40 28 Keyboard Return (ENTER)5 43 √ √ √ 4/101/104
41 29 Keyboard ESCAPE 110 √ √ √ 4/101/104
42 2A Keyboard DELETE (Backspace)13 15 √ √ √ 4/101/104
43 2B Keyboard Tab 16 √ √ √ 4/101/104
44 2C Keyboard Spacebar 61 √ √ √ 4/101/104
45 2D Keyboard - and (underscore)4 12 √ √ √ 4/101/104
46 2E Keyboard = and +4 13 √ √ √ 4/101/104
47 2F Keyboard [ and {4 27 √ √ √ 4/101/104
48 30 Keyboard ] and }4 28 √ √ √ 4/101/104
49 31 Keyboard \ and | 29 √ √ √ 4/101/104
50 32 Keyboard Non-US # and ~2 42 √ √ √ 4/101/104
51 33 Keyboard ; and :4 40 √ √ √ 4/101/104
52 34 Keyboard ‘ and “4 41 √ √ √ 4/101/104
53 35 Keyboard Grave Accent and Tilde4 1 √ √ √ 4/101/104
54 36 Keyboard, and <4 53 √ √ √ 4/101/104
55 37 Keyboard . and >4 54 √ √ √ 4/101/104
56 38 Keyboard / and ?4 55 √ √ √ 4/101/104
57 39 Keyboard Caps Lock11 30 √ √ √ 4/101/104
58 3A Keyboard F1 112 √ √ √ 4/101/104
59 3B Keyboard F2 113 √ √ √ 4/101/104
60 3C Keyboard F3 114 √ √ √ 4/101/104
61 3D Keyboard F4 115 √ √ √ 4/101/104
62 3E Keyboard F5 116 √ √ √ 4/101/104
63 3F Keyboard F6 117 √ √ √ 4/101/104
64 40 Keyboard F7 118 √ √ √ 4/101/104
65 41 Keyboard F8 119 √ √ √ 4/101/104
66 42 Keyboard F9 120 √ √ √ 4/101/104
67 43 Keyboard F10 121 √ √ √ 4/101/104
68 44 Keyboard F11 122 √ √ √ 101/104
69 45 Keyboard F12 123 √ √ √ 101/104
70 46 Keyboard PrintScreen1 124 √ √ √ 101/104
71 47 Keyboard Scroll Lock11 125 √ √ √ 4/101/104
72 48 Keyboard Pause1 126 √ √ √ 101/104
73 49 Keyboard Insert1 75 √ √ √ 101/104
74 4A Keyboard Home1 80 √ √ √ 101/104
75 4B Keyboard PageUp1 85 √ √ √ 101/104
76 4C Keyboard Delete Forward1;14 76 √ √ √ 101/104
77 4D Keyboard End1 81 √ √ √ 101/104
78 4E Keyboard PageDown1 86 √ √ √ 101/104
79 4F Keyboard RightArrow1 89 √ √ √ 101/104
80 50 Keyboard LeftArrow1 79 √ √ √ 101/104
81 51 Keyboard DownArrow1 84 √ √ √ 101/104
82 52 Keyboard UpArrow1 83 √ √ √ 101/104
83 53 Keypad Num Lock and Clear11 90 √ √ √ 101/104
84 54 Keypad /1 95 √ √ √ 101/104
85 55 Keypad * 100 √ √ √ 4/101/104
86 56 Keypad - 105 √ √ √ 4/101/104
87 57 Keypad + 106 √ √ √ 4/101/104
88 58 Keypad ENTER5 108 √ √ √ 101/104
89 59 Keypad 1 and End 93 √ √ √ 4/101/104
90 5A Keypad 2 and Down Arrow 98 √ √ √ 4/101/104
91 5B Keypad 3 and PageDn 103 √ √ √ 4/101/104
92 5C Keypad 4 and Left Arrow 92 √ √ √ 4/101/104
93 5D Keypad 5 97 √ √ √ 4/101/104
94 5E Keypad 6 and Right Arrow 102 √ √ √ 4/101/104
95 5F Keypad 7 and Home 91 √ √ √ 4/101/104
96 60 Keypad 8 and Up Arrow 96 √ √ √ 4/101/104
97 61 Keypad 9 and PageUp 101 √ √ √ 4/101/104
98 62 Keypad 0 and Insert 99 √ √ √ 4/101/104
99 63 Keypad . and Delete 104 √ √ √ 4/101/104
100 64 Keyboard Non-US \ and |3;6 45 √ √ √ 4/101/104
101 65 Keyboard Application10 129 √ √ 104
102 66 Keyboard Power9 √ √
103 67 Keypad = √
104 68 Keyboard F13 √
105 69 Keyboard F14 √
106 6A Keyboard F15 √
107 6B Keyboard F16
108 6C Keyboard F17
109 6D Keyboard F18
110 6E Keyboard F19
111 6F Keyboard F20
112 70 Keyboard F21
113 71 Keyboard F22
114 72 Keyboard F23
115 73 Keyboard F24
116 74 Keyboard Execute √
117 75 Keyboard Help √
118 76 Keyboard Menu √
119 77 Keyboard Select √
120 78 Keyboard Stop √
121 79 Keyboard Again √
122 7A Keyboard Undo √
123 7B Keyboard Cut √
124 7C Keyboard Copy √
125 7D Keyboard Paste √
126 7E Keyboard Find √
127 7F Keyboard Mute √
128 80 Keyboard Volume Up √
129 81 Keyboard Volume Down √
130 82 Keyboard Locking Caps Lock12 √
131 83 Keyboard Locking Num Lock12 √
132 84 Keyboard Locking Scroll Lock12 √
133 85 Keypad Comma27 107
134 86 Keypad Equal Sign29
135 87 Keyboard International115,28 56
136 88 Keyboard International216
137 89 Keyboard International317
138 8A Keyboard International418
139 8B Keyboard International519
140 8C Keyboard International620
141 8D Keyboard International721
142 8E Keyboard International822
143 8F Keyboard International922
144 90 Keyboard LANG125
145 91 Keyboard LANG226
146 92 Keyboard LANG330
147 93 Keyboard LANG431
148 94 Keyboard LANG532
149 95 Keyboard LANG68
150 96 Keyboard LANG78
151 97 Keyboard LANG88
152 98 Keyboard LANG98
153 99 Keyboard Alternate Erase7
154 9A Keyboard SysReq/Attention1
155 9B Keyboard Cancel
156 9C Keyboard Clear
157 9D Keyboard Prior
158 9E Keyboard Return
159 9F Keyboard Separator
160 A0 Keyboard Out
161 A1 Keyboard Oper
162 A2 Keyboard Clear/Again
163 A3 Keyboard CrSel/Props
164 A4 Keyboard ExSel
165-175 A5-CF Reserved
176 B0 Keypad 00
177 B1 Keypad 000
178 B2 Thousands Separator 33
179 B3 Decimal Separator 33
180 B4 Currency Unit 34
181 B5 Currency Sub-unit 34
182 B6 Keypad (
183 B7 Keypad )
184 B8 Keypad {
185 B9 Keypad }
186 BA Keypad Tab
187 BB Keypad Backspace
188 BC Keypad A
189 BD Keypad B
190 BE Keypad C
191 BF Keypad D
192 C0 Keypad E
193 C1 Keypad F
194 C2 Keypad XOR
195 C3 Keypad ^
196 C4 Keypad %
197 C5 Keypad <
198 C6 Keypad >
199 C7 Keypad &
200 C8 Keypad &&
201 C9 Keypad |
202 CA Keypad ||
203 CB Keypad :
204 CC Keypad #
205 CD Keypad Space
206 CE Keypad @
207 CF Keypad !
208 D0 Keypad Memory Store
209 D1 Keypad Memory Recall
210 D2 Keypad Memory Clear
211 D3 Keypad Memory Add
212 D4 Keypad Memory Subtract
213 D5 Keypad Memory Multiply
214 D6 Keypad Memory Divide
215 D7 Keypad +/-
216 D8 Keypad Clear
217 D9 Keypad Clear Entry
218 DA Keypad Binary
219 DB Keypad Octal
220 DC Keypad Decimal
221 DD Keypad Hexadecimal
222-223 DE-DF Reserved
224 E0 Keyboard LeftControl 58 √ √ √ 4/101/104
225 E1 Keyboard LeftShift 44 √ √ √ 4/101/104
226 E2 Keyboard LeftAlt 60 √ √ √ 4/101/104
227 E3 Keyboard Left GUI10;23 127 √ √ √ 104
228 E4 Keyboard RightControl 64 √ √ √ 101/104
229 E5 Keyboard RightShift 57 √ √ √ 4/101/104
230 E6 Keyboard RightAlt 62 √ √ √ 101/104
231 E7 Keyboard Right GUI10;24 128 √ √ √ 104
232-65535 E8-FFFF Reserved 
*/

