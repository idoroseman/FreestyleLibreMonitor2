/*
NFC Communication with the Solutions Cubed, LLC BM019 
and an Arduino Uno.  The BM019 is a module that
carries ST Micro's CR95HF, a serial to NFC converter.

Wiring:
 Arduino          BM019
 IRQ: Pin 9       DIN: pin 2
 SS: pin 10       SS: pin 3
 MOSI: pin 11     MOSI: pin 5 
 MISO: pin 12     MISO: pin4
 SCK: pin 13      SCK: pin 6
 
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "secret.h"

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

#include "soc/spi_struct.h"
struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

#if defined (__AVR__) || defined(TEENSYDUINO)

const int SSPin = 10;  // Slave Select pin
const int IRQPin = 27;  // Sends wake-up pulse

#elif defined(ESP8266) || defined(ESP32)

const int SSPin = 4;  // Slave Select pin
const int IRQPin = 27;  // Sends wake-up pulse

#endif

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan
byte FirstRun = 1;
int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];
float glucoseReading;
byte TXBuffer[40];    // transmit buffer
byte RXBuffer[40];    // receive buffer
byte NFCState = 0;  // used to track NFC state
int count = 0;


#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

//---------------------------------------------------------------------------------------------------

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH);
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);

    Serial.begin(9600);
    Serial.println();
    Serial.println("CR95HF Demo");

    SPI.begin();
    spi_t * _spi;
    _spi= SPI.bus();
    _spi->dev->ctrl2.miso_delay_mode = 2;
    
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
}


//--------------------------------------------------------------------------------
//                                  ESP32
//--------------------------------------------------------------------------------

float getBatteryVolt() {
  float measuredvbat = analogRead(A13);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by reference voltage
  measuredvbat *= 1.1; // Multiply by adc reference voltage
  measuredvbat /= 4095; // adc range 

  return measuredvbat;
}

int getBatteryPercent() {
  // 100% = 4.2
  // empty = 3.4
  float volt = getBatteryVolt();
  float percent = ((volt-3.4)/0.8)*100.0;
  return min(100,max(0,(int)percent));
}

void goToSleep( int timeToSleep) {
  Serial.println("Going to sleep now");
  delay(100);
  Hibernate_Command();
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}


//---------------------------------------------------------------------------------------------------
//                                            CR95HF
//---------------------------------------------------------------------------------------------------

void CR95HF_Reset()
{

 
 // The CR95HF requires a wakeup pulse on its IRQ_IN pin
 // before it will select UART or SPI mode.  The IRQ_IN pin
 // is also the UART RX pin for DIN on the BM019 board.
 
    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
}

void CR95HF_Send(byte cmnd, byte len, ...)
{
  va_list argv;
  va_start(argv, len);
  // step 1 send the command
  digitalWrite(SSPin, LOW);
  delay(1);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(cmnd);  // IDN command
  SPI.transfer(len);  // length of data that follows is 0
  for (int i=0;i<len;i++)
  {
    byte val=va_arg(argv, int);
    SPI.transfer(val);  // length of data to follow
  }
  digitalWrite(SSPin, HIGH);
  delay(1);
  va_end(argv);
}

bool CR95HF_Receive() 
{
  // step 2, poll for data ready
  // data is ready when a read byte
  // has bit 3 set (ex:  B'0000 1000')
//  Serial.print("Polling");
  digitalWrite(SSPin, LOW);
  delay(1);
  RXBuffer[1] = 64;
  while((RXBuffer[0] != 8) && (RXBuffer[1] !=0))
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    //Serial.print(RXBuffer[0], HEX);
    //Serial.print(" ");
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    delay(10);
    RXBuffer[1] --;
    }
  digitalWrite(SSPin, HIGH);
  delay(1);
  if (RXBuffer[1] == 0)
    return 0;
  // step 3, read the data
  digitalWrite(SSPin, LOW);
  delay(1);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0) ;  // response code
  RXBuffer[1] = SPI.transfer(0) ;  // length of data
/*     
  Serial.print("Response Code ");
  Serial.print(RXBuffer[0], HEX);
  Serial.print(" Data len ");
  Serial.print(RXBuffer[1], HEX);
  Serial.println();
*/
  if (RXBuffer[1] >= sizeof(RXBuffer)-2)
    RXBuffer[1] = 0;
//  Serial.print("Data: ");  
  for (int i=0;i<RXBuffer[1];i++) {
      RXBuffer[i+2]=SPI.transfer(0) ;  // data
//      Serial.print(RXBuffer[i+2],HEX);
//      Serial.print(" ");
  }
  digitalWrite(SSPin, HIGH);
//  Serial.println(".");
  delay(1);
  return true;
}

//---------------------------------------------------------------------------------------------------
//                                  COMMANDS
//---------------------------------------------------------------------------------------------------
bool Hibernate_Command()
{
  CR95HF_Send(0x07, 14, 0x08, 0x04, 0x00 , 0x04 , 0x00, 0x18 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00);
  delay(10);
  CR95HF_Receive();
}

/* IDN_Command identifies the CR95HF connected to the Arduino.
This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display the CR95HF ID number and CRC code.  This rountine is 
not that useful in using the NFC functions, but is a good way to 
verify connections to the CR95HF. 
*/
bool IDN_Command()
 {
 byte i = 0;

  // step 1 send the command
  CR95HF_Send(0x01, 0);
  delay(10);
  CR95HF_Receive();

  if ((RXBuffer[0] == 0) && (RXBuffer[1] == 15))
  {  
    Serial.print("DEVICE ID: ");
    for(i=2; (RXBuffer[i] != '\0') && (i<(RXBuffer[1]));i++)
    {
      Serial.print(char(RXBuffer[i] ));
    }
    i++;
    Serial.print(" ");
    Serial.print("ROM CRC: ");
    Serial.print(RXBuffer[i],HEX);
    Serial.print(RXBuffer[i+1],HEX);
    Serial.println(" ");
    delay(1000);
    return true;
  }
  else
  {
    Serial.print("BAD RESPONSE TO IDN COMMAND!");
    Serial.print(RXBuffer[0]);
    Serial.print(RXBuffer[1]);
    Serial.println();
    return false;
  }
}

/* SetProtocol_Command programs the CR95HF for
ISO/IEC 15693 operation.

This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display successful programming. 
*/
bool SetProtocol_Command()
 {
 byte i = 0;
 
// step 1 send the command
/*
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
*/
  CR95HF_Send(0x02, 2, 0x01, 0x0D);
  delay(1);
  CR95HF_Receive();

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))
  {
     Serial.println("PROTOCOL SET-");  //
     return true;
  }
  else
  {
     Serial.println("BAD RESPONSE TO SET PROTOCOL");
     return false;
  }
  Serial.println(" ");
}

/* Inventory_Command chekcs to see if an RF
tag is in range of the BM019.

This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display the the RF tag's universal ID.  
*/
bool Inventory_Command()
 {
 byte i = 0;
/*
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
*/
  CR95HF_Send(0x04, 4, 0x26, 0x01, 0x00);
  delay(10);
  CR95HF_Receive();

  if ((RXBuffer[0] == 0x80) && (RXBuffer[1] == 13))
  {  
    Serial.print("TAG UID: ");
    for(i=11;i>=4;i--)
    {
      Serial.print(RXBuffer[i] >> 4,HEX);
      Serial.print(RXBuffer[i] & 0x0f,HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
    return true;
  }
  else
  {
    Serial.println("No tag in range");
    return false;
  }

}

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 8.5);
}

float ReadMemory_Command() {

 byte oneBlock[8];
 String hexPointer = "";
 String trendValues = "";
 String hexMinutes = "";
 String elapsedMinutes = "";
 float trendOneGlucose;
 float trendTwoGlucose;
 float currentGlucose;
 float shownGlucose;
 float averageGlucose = 0;
 int glucosePointer;
 int validTrendCounter = 0;
 float validTrend[16];
 byte readError = 0;
 int readTry;
  
 for ( int block = 3; block < 16; block++) {
  readTry = 0;
  do {
      readError = 0;   
      CR95HF_Send(0x04, 3, 0x02, 0x20, block);
      delay(1);
      CR95HF_Receive();
      if (RXBuffer[0] != 128)
         readError = 1;  
      
      for (int i = 0; i < 8; i++)
         oneBlock[i] = RXBuffer[i+3];
    
      char str[24];
      unsigned char * pin = oneBlock;
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for(; pin < oneBlock+8; pout+=2, pin++) {
          pout[0] = hex[(*pin>>4) & 0xF];
          pout[1] = hex[ *pin     & 0xF];
      }
      pout[0] = 0;
      if (!readError)       // is response code good?
      { 
        Serial.print(block / 10);
        Serial.print(block % 10);
        Serial.print(": ");
        Serial.println(str);
        trendValues += str;
      }
      readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );
  
 } // of blocks
 readTry = 0;
 do {
     readError = 0;  
     CR95HF_Send(0x04, 3, 0x02, 0x20, 39);
     delay(1);
     CR95HF_Receive();
     if (RXBuffer[0] != 128)
        readError = 1;  
      delay(1);
  
      for (int i = 0; i < 8; i++)
         oneBlock[i] = RXBuffer[i+3];
    
      char str[24];
      unsigned char * pin = oneBlock;
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for(; pin < oneBlock+8; pout+=2, pin++) {
         pout[0] = hex[(*pin>>4) & 0xF];
         pout[1] = hex[ *pin     & 0xF];
     }
     pout[0] = 0;
     if (!readError) {
        Serial.print(39);
        Serial.print(": ");
        Serial.println(str);
        elapsedMinutes += str;
     }
     readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );
      
  if (!readError)
    {
      hexMinutes = elapsedMinutes.substring(10,12) + elapsedMinutes.substring(8,10);
      hexPointer = trendValues.substring(4,6);
      sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
      glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);
             
      Serial.println("");
      Serial.print("Minutes Elapse: ");
      Serial.print(sensorMinutesElapse);
      Serial.println();
      Serial.print("Glucose pointer: ");
      Serial.print(glucosePointer);
      Serial.println("");
      
      int ii = 0;
      for (int i=8; i<=200; i+=12) {
        if (glucosePointer == ii)
        {
          if (glucosePointer == 0)
          {
            String trendNow = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendOne = trendValues.substring(178,180) + trendValues.substring(176,178);
            String trendTwo = trendValues.substring(166,168) + trendValues.substring(164,166);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));
            Serial.print("current: ");
            Serial.print(currentGlucose);
            Serial.print(" trend 1: ");
            Serial.print(trendOneGlucose);
            Serial.print(" trend 2: ");
            Serial.print(trendTwoGlucose);
            Serial.println();
            if (FirstRun == 1)
               lastGlucose = currentGlucose;
       
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else if (glucosePointer == 1)
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(190,192) + trendValues.substring(188,190);
            String trendTwo = trendValues.substring(178,180) + trendValues.substring(176,178);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));
            Serial.print("current: ");
            Serial.print(currentGlucose);
            Serial.print(" trend 1: ");
            Serial.print(trendOneGlucose);
            Serial.print(" trend 2: ");
            Serial.print(trendTwoGlucose);
            Serial.println();
            if (FirstRun == 1)
               lastGlucose = currentGlucose;
               
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
          else
          {
            String trendNow = trendValues.substring(i-10,i-8) + trendValues.substring(i-12,i-10);
            String trendOne = trendValues.substring(i-22,i-20) + trendValues.substring(i-24,i-22);
            String trendTwo = trendValues.substring(i-34,i-32) + trendValues.substring(i-36,i-34);
            currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));
            trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL ,16));
            trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL ,16));
            Serial.print("current: ");
            Serial.print(currentGlucose);
            Serial.print(" trend 1: ");
            Serial.print(trendOneGlucose);
            Serial.print(" trend 2: ");
            Serial.print(trendTwoGlucose);
            Serial.println();

            if (FirstRun == 1)
               lastGlucose = currentGlucose;
               
            if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
            {
               if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
                  currentGlucose = trendTwoGlucose;
               else
                  currentGlucose = trendOneGlucose;
            }
          }
        }  

        ii++;
      }
     
     for (int i=8, j=0; i<200; i+=12,j++) {
          String t = trendValues.substring(i+2,i+4) + trendValues.substring(i,i+2);
          trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL ,16));
       }

    for (int i=0; i<16; i++)
    {
      if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
         continue;
      else
      {
         validTrend[validTrendCounter] = trend[i];
         validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    { 
      for (int i=0; i < validTrendCounter; i++)
         averageGlucose += validTrend[i];
         
      averageGlucose = averageGlucose / validTrendCounter;
      
      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
         shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
         shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value 

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose; 

    
    //NFCReady = 2;
    FirstRun = 0;

    if (noDiffCount > 5)
      return 0;
    else  
      return shownGlucose;
    
    }
  else
    {
    Serial.print("Read Memory Block Command FAIL");
    //NFCReady = 0;
    readError = 0;
    }
    return 0;
 }

//---------------------------------------------------------------------------------------------------

void loop() {
  byte numSsid;
  int status = WL_IDLE_STATUS;

  glucoseReading = 0;
  NFCState = 0;
  
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Reset CR95HF");
  CR95HF_Reset();
  delay(100);
  if (IDN_Command())   // reads the CR95HF ID
  {
    if (SetProtocol_Command()) // ISO 15693 settings
      {
         for (int retry=0; retry<10; retry++)
         {
           digitalWrite(LED_BUILTIN, HIGH); 
           if (Inventory_Command()) 
           {
              glucoseReading = ReadMemory_Command();
              break;
           }
           digitalWrite(LED_BUILTIN, LOW);
           delay(1000);
         }

         Serial.print("Glucose Reading=");
         Serial.println(glucoseReading);
         if (glucoseReading < 10) // bad tag
           NFCState = -4;           
         if (glucoseReading == 0) // no tag
           NFCState = -3;
         Serial.println("15 minutes-trend: ");
         for (int i=0; i<16; i++)
          {
            Serial.print(trend[i]);
            Serial.print(" ");
          }
          Serial.println();
          Serial.print("Sensor lifetime: ");
          Serial.print(sensorMinutesElapse);
          Serial.print(" minutes elapsed");
          Serial.print(" (");
          Serial.print(sensorMinutesElapse / (24*60));
          Serial.print(" days)");
          Serial.println("");
      }
      else
        NFCState = -2;
    }
    else
      NFCState = -1;
  Serial.print("NFC State ");
  Serial.println(NFCState);
  Serial.print("Battery ");
  Serial.print(getBatteryVolt());
  Serial.print(" (");
  Serial.print(getBatteryPercent());
  Serial.print("%)");
  Serial.println();
  
  // send report
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.mode(WIFI_STA);
  status = WiFi.begin(ssid, key);
  int wifitimeout = 10;
  while (status != WL_CONNECTED)
  {
    Serial.print(".");
    if (wifitimeout == 0)
    {
      Serial.println("timed out");
      break;
    }
    wifitimeout--;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    status = WiFi.status();
  }
  Serial.println();
  digitalWrite(LED_BUILTIN, LOW);
    
  if (status == WL_CONNECTED)
  {
    Serial.println("start mqtt");
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    digitalWrite(LED_BUILTIN, HIGH);
    if (mqttClient.connect("myClientID")) {
      // connection succeeded
      digitalWrite(LED_BUILTIN, HIGH);
      char buffer[256];
      String(NFCState).toCharArray(buffer,10);
      mqttClient.publish("casamami/mamiguard/status", buffer);
      dtostrf(getBatteryPercent(), 0, 0, buffer);
      mqttClient.publish("casamami/mamiguard/battery/percent", buffer);
      dtostrf(getBatteryVolt(), 0, 2, buffer);
      mqttClient.publish("casamami/mamiguard/battery/volt", buffer);
      if (sensorMinutesElapse > 0)
      {
        dtostrf(sensorMinutesElapse, 0, 0, buffer);
        mqttClient.publish("casamami/mamiguard/lifetime", buffer);
      }
      bool rc;
      if (glucoseReading < 1.0)
        rc = mqttClient.publish("casamami/mamiguard/glucose", "---");
      else {
        String packet ="";
        packet += "[";
        for (int i=0; i<16; i++)
        {
          if (i>0)
          packet += ", ";
          packet += String((int)trend[i]);
        }
        packet += "]";
        packet.toCharArray(buffer,255);
        mqttClient.publish("casamami/mamiguard/trend", buffer);      

        dtostrf(glucoseReading, 0, 0, buffer);
        rc = mqttClient.publish("casamami/mamiguard/glucose", buffer);
      }
      if (rc) {
        Serial.println("publish ok ");
        mqttClient.disconnect();
        WiFi.disconnect();
      }
    } else {
      // connection failed
      // mqttClient.state() will provide more information on why it failed.
      Serial.print("MQTT Error ");
      Serial.println(mqttClient.state());
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  goToSleep(20*60);
}



