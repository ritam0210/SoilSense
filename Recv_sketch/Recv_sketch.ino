//Receiver Sketch
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include "time.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

#define TFT_CLK 18  // Define your SPI clock pin
#define TFT_MOSI 23 // Define your SPI MOSI pin
#define TFT_CS 15   // Define your TFT chip select pin
#define TFT_DC 2    // Define your TFT DC or data/command pin
#define TFT_RST 4   // Define your TFT reset pin

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

const int led = 16;
int ledState = LOW;   // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated
//-----------------------------------------------------------

//====================================ESP NOW====================================

// REPLACE WITH THE MAC Address of your receiver 
//uint8_t RXMACaddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //40:22:D8:7B:64:D8

//--------------------VARIABLE DECLARATION-------------------
float temp= 0.0;
float soil_1= 0.0;
float soil_2= 0.0;
float soil_3= 0.0;
float soil_4= 0.0;
float flow_1= 0.0;
float flow_2= 0.0;
float capacity_1= 0.0;
float capacity_2= 0.0;

//-----------------------------------Data from app----------------------------------------
float area_1= 0.0656;
float area_2= 0.0656;
float Req_1= 65.0;
float Req_2= 68.0;

String valve_1= "OFF";
String valve_2= "OFF";
String type_1= "Rabi";
String type_2= "Kharif";
String name_1= "Peas";
String name_2= "Apple";

//-----------------------------------Data from ESP----------------------------------------
float incomingtemp;
float incomingsoil_1;
float incomingsoil_2;
float incomingsoil_3;
float incomingsoil_4;
float incomingflow_1;
float incomingflow_2;
float incomingcapacity_1;
float incomingcapacity_2;
String incomingvalve_1;
String incomingvalve_2;

//--------------------------WIFI------------------------------
// Replace with your network credentials (STATION)
const char* ssid = "Ritam";
const char* password = "ritam0909";
//------------------------------------------------------------

//==================================ESP NOW SENDER STRUCTURE=========================================
// Variable to store if sending data was successful
//String success;
//
//typedef struct TxStruct
//{
//    float req_1;
//    float req_2;
//    
//}TxStruct;
//
//// Create a struct_message called myData
//TxStruct sendData;

//==================================ESP NOW RECEIVER STRUCTURE=======================================

typedef struct RxStruct
{
    float temp;
    float soil_1;
    float soil_2;
    float soil_3;
    float soil_4;
    float flow_1;
    float flow_2;
    float capacity_1;
    float capacity_2;
    String valve_1;
    String valve_2;
    
}RxStruct;

// Create a struct_message called myData
RxStruct receivedData;

//esp_now_peer_info_t peerInfo;

//==================================ESP NOW SENDER STRUCTURE=======================================

//// Callback when data is sent
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
//{
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  if (status ==0)
//  {
//    success = "Delivery Success :)";
//  }
//  else
//  {
//    success = "Delivery Fail :(";
//  }
//}

//=================================ESP NOW RECEIVER FUNCTION=========================================

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  blinkit();
  
  // Update the structures with the new incoming data
  incomingtemp = receivedData.temp;
  incomingsoil_1 = receivedData.soil_1;
  incomingsoil_2 = receivedData.soil_2;
  incomingsoil_3 = receivedData.soil_3;
  incomingsoil_4 = receivedData.soil_4;
  incomingflow_1 = receivedData.flow_1;
  incomingflow_2 = receivedData.flow_2;
  incomingcapacity_1 = receivedData.capacity_1;
  incomingcapacity_2 = receivedData.capacity_2;
  incomingvalve_1 = receivedData.valve_1;
  incomingvalve_2 = receivedData.valve_2;
}

//===================================================================================================

//==================================GOOGLE SHEET CREDENTIALS=========================================
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 0;
// Google script ID and required credentials
String GOOGLE_SCRIPT_ID = "AKfycbzHfGrXA0EDJBLYASBcCkLqJ-ACYUxN5NcI6RI2PWcRqyD6697XQIm9T5k2f8XgpT9Y";

//===================================================================================================

//====================================FUNCTIONS FOR LED BLINKING=====================================

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
  tft.println(type_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.print("CROP NAME: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(name_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 110);
  tft.print("CROP AREA: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(area_1);
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
  tft.println(incomingflow_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 200);
  tft.print("WATER CAP: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(incomingcapacity_1);
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
  tft.println("OFF");

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
  tft.println(type_2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.print("CROP NAME: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(name_2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 110);
  tft.print("CROP AREA: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(area_2);
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
  tft.println(incomingflow_1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 200);
  tft.print("WATER CAP: ");
  tft.setTextColor(ILI9341_YELLOW);
  tft.println(incomingcapacity_2);
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
  tft.println("OFF");

  delay(1000);

//  Serial.println(receivedData.temp);
//  Serial.println(receivedData.soil_1);
//  Serial.println(receivedData.soil_2);
//  Serial.println(receivedData.soil_3);
//  Serial.println(receivedData.soil_4);
//  Serial.println(receivedData.flow_1);
//  Serial.println(receivedData.flow_2);
//  Serial.println(receivedData.capacity_1);
//  Serial.println(receivedData.capacity_2);
//  Serial.println(receivedData.valve_1);
//  Serial.println(receivedData.valve_2);
//  Serial.println(==============================================================================================);

}
//==============================================================================================
void setup() 
{
  Serial.begin(115200);
  pinMode(led, OUTPUT);
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

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  //--------------------------------------------------------------------------------------------------
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

//  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
//  esp_now_register_send_cb(OnDataSent);
//  
//  // Register peer
//  memcpy(peerInfo.peer_addr, RXMACaddress, 6);
//  peerInfo.channel = 0;  
//  peerInfo.encrypt = false;
//  
//  // Add peer        
//  if (esp_now_add_peer(&peerInfo) != ESP_OK)
//  {
//    Serial.println("Failed to add peer");
//    return;
//  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  //--------------------------------------------------------------------------------------------------

  // Init and get the time
//  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
//  Serial.flush();
  //--------------------------------------------------------------------------------------------------
}

void loop() 
{
  // Set values to send
//  sendData.req_1 = Req_1;
//  sendData.req_2 = Req_2;
//
//  // Send message via ESP-NOW
//  esp_err_t result = esp_now_send(RXMACaddress, (uint8_t *) &sendData, sizeof(sendData));
//   
//  if (result == ESP_OK) 
//  {
//    Serial.println("Sent with success");
//  }
//  else 
//  {
//    Serial.println("Error sending the data");
//  }
//  
  //--------------------------------------------------------------------------------------------------

/*  if (WiFi.status() == WL_CONNECTED) 
  {
    static bool flag = false;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) 
    {
      Serial.println("Failed to obtain time");
      return;
    }

    String temp= String(receivedData.temp);
    String soil_1= String(receivedData.soil_1);
    String soil_2= String(receivedData.soil_2);
    String soil_3= String(receivedData.soil_3);
    String soil_4= String(receivedData.soil_4);
    String flow_1= String(receivedData.flow_1);
    String flow_2= String(receivedData.flow_2);
    String capacity_1= String(receivedData.capacity_1);
    String capacity_2= String(receivedData.capacity_2);

    String area_1= String(area_1);
    String area_2= String(area_2);
    String req_1= String(req_1);
    String req_2= String(req_2);   
    
    String Valve_1= String(receivedData.valve_1);
    String Valve_2= String(receivedData.valve_2);
    String Type_1= String(type_1);
    String Type_2= String(type_2);
    String Name_1= String(name_1);
    String Name_2= String(name_2);

    String urlFinal = "https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?temp=" + temp + "&type_1=" + Type_1 + "&name_1=" + Name_1 + 
    "&area_1=" + area_1 + "&req_moist_1=" + req_1 + "&soil_1=" + soil_1 + "&soil_2=" + soil_2 + "&flow_1=" + flow_1 + "&valve_1=" + Valve_1 + "&quant_1=" + capacity_1 + "&type_2=" + Type_2 + 
    "&name_2=" + Name_2 + "&area_2=" + area_2 + "&req_moist_2=" + req_2 + "&soil_3=" + soil_3 + "&soil_4=" + soil_4 + "&flow_2=" + flow_2 + "&valve_2=" + Valve_2 + "&quant_2=" + capacity_2;
    Serial.println("POST data to spreadsheet:");
    
//    Serial.println(urlFinal);
    HTTPClient http;
    http.begin(urlFinal.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET(); 
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
    String payload;
    if (httpCode > 0) 
    {
        payload = http.getString();
        Serial.println("Payload: "+payload);    
    }
    //---------------------------------------------------------------------
    http.end();
    unsigned long currentMillis = millis();
//    Serial.println(currentMillis);
  }*/
  updatedisplay();
  delay(500);
}
