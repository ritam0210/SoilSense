#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#define TFT_CS 15    // TFT chip select pin
#define TFT_DC 2     // TFT DC or data/command pin
#define TFT_RST 12   // TFT reset pin

#define led 13
int ledState = LOW;
unsigned long previousMillis = 0;   // will store last time LED was updated

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);    //TFT initialisation
//--------------------------------------------------------------------------------------------------

float Temp= 0.0;

//SYSTEM 1
float Soil_1= 0.0;
float Soil_2= 0.0;
float Flow_1= 0;
float Qty_1= 0.0;
String Valve_1;

//SYSTEM 2
float Soil_3= 0.0;
float Soil_4= 0.0;
float Flow_2= 0;
float Qty_2= 0.0;
String Valve_2;

float Req_1= 40.0; 
float Req_2= 40.0;
float Area_1= 0.786;
float Area_2= 0.786;

//-----------------------------------Data from app----------------------------------------
//String type_1= "Rabi";
//String type_2= "Kharif";
//String name_1= "Peas";
//String name_2= "Apple";

//-----------------------------------------------------------
// Replace with your network credentials (STATION)
const char* ssid = "Ritam";
const char* password = "ritam0909";
//------------------------------------------------------------

//---------------------------FIREBASE-------------------------
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert Firebase project API Key
#define API_KEY "AIzaSyAG9_-Ca9R9wBuoZy0mgKd4i4Ev_d1K69c"
// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://soilsense-bc90c-default-rtdb.asia-southeast1.firebasedatabase.app/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis1 = 0;
unsigned long sendDataPrevMillis2 = 0;
bool signupOK = false;

//--------------------------------ESP NOW--------------------------------
// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x7B, 0x64, 0xD8};
//
//// Structure example to send data
//// Must match the receiver structure
//typedef struct struct_send 
//{
//  float req_1;
//  float req_2;
//} send_message;
//
//// Create a struct_message called sendData
//send_message sendData;

// ESPNOW Structure to receive data
// Must match the sender structure
typedef struct struct_message 
{
  float temp;
  float soil_1;
  float soil_2;
  float flow_1;
  float qty_1;
  String valve_1;
  float soil_3;
  float soil_4;
  float flow_2;
  float qty_2;
  String valve_2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
//{
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  blinkit();
}

//float temp;
//float soil_1;
//float soil_2;
//float flow_1;
//float qty_1;
//String valve_1;
//float soil_3;
//float soil_4;
//float flow_2;
//float qty_2;
//String valve_2;
//
//String name_1;
//String name_2;
//String type_1;
//String type_2;
//float Area_1 = 0.0;
//float Area_2 = 0.0;
//float Req_1 = 0.0;
//float Req_2 = 0.0;

//---------------------------------------------------------------------------------------------------

void fb_send()
{
  if (Firebase.ready() && signupOK)
  {
    if (Firebase.RTDB.setFloat(&fbdo, "Temperature", Temp))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    //*******************************************************************************************
    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Soil_1", Soil_1))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Soil_2", Soil_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Flow_1", Flow_1))
    Serial.println("PASSED");
      
    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Qty_1", Qty_1))
    Serial.println("PASSED");
      
    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setString(&fbdo, "System 1/Valve_1", Valve_1))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    //*******************************************************************************************
    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Soil_3", Soil_3))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Soil_4", Soil_4))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Flow_2", Flow_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Qty_2", Qty_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setString(&fbdo, "System 2/Valve_2", Valve_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
}

/*void fb_fetch()
{
  if (Firebase.RTDB.getFloat(&fbdo, "system-1-up/req")) 
  {   
    Req_1 = fbdo.floatData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }
  
  if (Firebase.RTDB.getFloat(&fbdo, "system-2-up/req")) 
  {   
    Req_2 = fbdo.floatData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

}*/
//---------------------------------------------------------------------------------------------------

//====================================FUNCTION FOR LED BLINKING====================================

void blinkit()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 750) 
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;  
    if(ledState == LOW)
    {
      ledState = HIGH;                        
    }    
    else
    {
      ledState = LOW;
    }
  }
  digitalWrite(led, ledState);
}

//===================================================================================================

//========================================DISPLAY FUNCTION===========================================

void updatedisplay()
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(2);
  tft.setCursor(70, 10);
  tft.println("SYSTEM-1");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 50);
  tft.print("CROP TYPE: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("Rabi");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.print("CROP NAME: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("Peas");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 110);
  tft.print("CROP AREA: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("0.786");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.print("REQ MOIST: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Req_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 170);
  tft.print("FLOW METER: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Flow_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 200);
  tft.print("WATER CAP: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Qty_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 230);
  tft.print("SENS-1 D: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(72.0);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 260);
  tft.print("SENS-2 D: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(92.0);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 290);
  tft.print("VALVE: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Valve_1);

  delay(1000);

  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(2);
  tft.setCursor(70, 10);
  tft.println("SYSTEM-2");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 50);
  tft.print("CROP TYPE: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("Rabi");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.print("CROP NAME: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("Beetroot");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 110);
  tft.print("CROP AREA: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("0.786");
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.print("REQ MOIST: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Req_2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 170);
  tft.print("FLOW METER: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Flow_2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 200);
  tft.print("WATER CAP: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Qty_2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 230);
  tft.print("SENS-1 D: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(82.0);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 260);
  tft.print("SENS-2 D: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(83.0);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 290);
  tft.print("VALVE: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(Valve_2);

  delay(1000);
}
//---------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  tft.begin();
  
  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println("Setting as a Wi-Fi Station..");
    delay(1000);
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
  //--------------------------------------------------------------------------------------------------
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
//  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
//  esp_now_peer_info_t peerInfo = {};
//  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//  peerInfo.encrypt = false;
//  
//  // Add peer        
//  if (esp_now_add_peer(&peerInfo) != ESP_OK)
//  {
//    Serial.println("Failed to add peer");
//    return;
//  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  //--------------------------------------------------------------------------------------------------

  //--------------------------------------FIREBASE-------------------------------------------------

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", ""))
  {
    Serial.println("ok");
    signupOK = true;
  }
  else
  {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop()
{
  Temp = myData.temp;
  Soil_1 = myData.soil_1;
  Soil_2 = myData.soil_2;
  Flow_1 = myData.flow_1;
  Qty_1 = myData.qty_1;
  Valve_1 = myData.valve_1;

  Soil_3 = myData.soil_3;
  Soil_4 = myData.soil_4;
  Flow_2 = myData.flow_2;
  Qty_2 = myData.qty_2;
  Valve_2 = myData.valve_2;
  
  // Send message via ESP-NOW
//  sendData.req_1 = Req_1;
//  sendData.req_2 = Req_2;
//  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
//   
//  if (result == ESP_OK) 
//  {
//    Serial.println("Sent with success");
//  }
//  else 
//  {
//    Serial.println("Error sending the data");
//  }
  delay(1000);
  //--------------------------------------------------------------------------------------------------

  fb_send();
  delay(300);
//  fb_fetch();

//  Serial.println(Req_1);
//  Serial.println(Req_2);
  updatedisplay();
  
}
