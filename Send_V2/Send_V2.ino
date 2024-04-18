#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define mosfet 18
#define FlowSensor1  17
#define FlowSensor2  16
#define SoilSensor1  34
#define SoilSensor2  35
#define SoilSensor3  32
#define SoilSensor4  39

#define FlowRelay1   26
#define FlowRelay2   27
#define PumpRelay1   14
#define PumpRelay2   25
#define ValveRelay1  12
#define ValveRelay2  13

#define led 2
int ledState = LOW;
unsigned long previousMillis = 0;   // will store last time LED was updated

int valve1_state = LOW;
int valve2_state = LOW;

const int oneWireBus = 33;   //Temperature sensor
//================================================================================================

//===============================Temperature sensor declarations==================================
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature temp_sens(&oneWire);
float Temperature = 0.0;
//================================================================================================

//======================================Soil sensor declarations====================================
const int AirValue1 = 1850;   //2110
const int WaterValue1 = 950;  //805

const int AirValue2 = 1600;   //2110
const int WaterValue2 = 950;  //805

const int AirValue3 = 1550;   //2110
const int WaterValue3 = 780;  //805

const int AirValue4 = 1565;   //2110
const int WaterValue4 = 850;  //805

int m_val1 = 0;
int m_val2 = 0;
int m_val3 = 0;
int m_val4 = 0;

float soilMoistureValue1 = 0.0;
float soilMoistureValue2 = 0.0;
float soilMoistureValue3 = 0.0;
float soilMoistureValue4 = 0.0;

float soilmoisturepercent1 = 0.0;
float soilmoisturepercent2 = 0.0;
float soilmoisturepercent3 = 0.0;
float soilmoisturepercent4 = 0.0;

float moist_1 = 0.0;
float moist_2 = 0.0;
float moist_3 = 0.0;
float moist_4 = 0.0;
//================================================================================================

//====================================Flow sensor declarations====================================
long currentMillis1 = 0;
long previousMillis1 = 0;

long currentMillis2 = 0;
long previousMillis2 = 0;

int interval = 1000;
float calibrationFactor = 4.5;

volatile byte pulseCount1;
volatile byte pulseCount2;

byte pulse1Sec_1 = 0;
byte pulse1Sec_2 = 0;

float flowRate1;
float flowRate2;

unsigned int flowMilliLitres1;
unsigned long totalMilliLitres1;

unsigned int flowMilliLitres2;
unsigned long totalMilliLitres2;
//================================================================================================

//==============================DECLARING FUNCTIONS FOR LCD DISPLAY===============================
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x3F for a 16 chars and 2 line display //0x27 in node3
//================================================================================================

//================================================================================================
float Temp;
float Soil_1;
float Soil_2;
float Soil_3;
float Soil_4;
float Flow_1;
float Flow_2;
float Qty_1;
float Qty_2;
String Valve_1= "OFF";
String Valve_2= "OFF";

float Req_1 = 60.0;
float Req_2 = 70.0;
//================================================================================================

//------------------------------------ESP NOW------------------------------------

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5B, 0xFE, 0x48};

// SENDER STRUCTURE
// Must match the receiver structure
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
struct_message myData;    //send object

// RECEIVER STRUCTURE
// Must match the sender structure
//typedef struct struct_send 
//{
//  float req_1;
//  float req_2;
//} send_message;
//
//// Create a struct_message called sendData
//send_message sendData;    //recv object

// Insert your SSID
constexpr char WIFI_SSID[] = "Ritam";

int32_t getWiFiChannel(const char *ssid) 
{
  if (int32_t n = WiFi.scanNetworks()) 
  {
      for (uint8_t i=0; i<n; i++) 
      {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) 
          {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// callback when data is SENT
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is RECEIVED
//void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
//{
//  memcpy(&sendData, incomingData, sizeof(sendData));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
//  blinkit();
//}
//================================================================================================

//==================================TEMP FUNCTION===========================================
float calc_temp()
{
  temp_sens.requestTemperatures(); 
  float temperatureC = temp_sens.getTempCByIndex(0);
  Temperature = temperatureC;

  return Temperature;
}
  
//================================================================================================

//==============================MOISTURE FUNCTIONS=========================================
float calc_moisture1()
{
  m_val1 = analogRead(SoilSensor1);
  soilMoistureValue1 = (m_val1*3300)/4096;
//  Serial.println(soilMoistureValue1);
  
  soilmoisturepercent1 = map(soilMoistureValue1, AirValue1, WaterValue1, 0, 100);
  if(soilmoisturepercent1 > 100.0)
  {
    moist_1 = 100.0;
//    Serial.println("100 %");
  }
  else if(soilmoisturepercent1 <0.0)
  {
    moist_1 = 0.0;
//    Serial.println("0 %");
  }
  else if(soilmoisturepercent1 >=0.0 && soilmoisturepercent1 <= 100.0)
  {
    moist_1 = soilmoisturepercent1;
//    Serial.print(soilmoisturepercent);
//    Serial.println("%");
  }
  
  return moist_1;
}

float calc_moisture2()
{
  m_val2 = analogRead(SoilSensor2);
  soilMoistureValue2 = (m_val2*3300)/4096;
  
  soilmoisturepercent2 = map(soilMoistureValue2, AirValue2, WaterValue2, 0, 100);
  if(soilmoisturepercent2 > 100.0)
  {
    moist_2 = 100.0;
  }
  else if(soilmoisturepercent2 <0.0)
  {
    moist_2 = 0.0;
  }
  else if(soilmoisturepercent2 >=0.0 && soilmoisturepercent2 <= 100.0)
  {
    moist_2 = soilmoisturepercent2;
  }
  
  return moist_2;
}

float calc_moisture3()
{
  m_val3 = analogRead(SoilSensor3);
  soilMoistureValue3 = (m_val3*3300)/4096;
  
  soilmoisturepercent3 = map(soilMoistureValue3, AirValue3, WaterValue3, 0, 100);
  if(soilmoisturepercent3 > 100.0)
  {
    moist_3 = 100.0;
  }
  else if(soilmoisturepercent3 <0.0)
  {
    moist_3 = 0.0;
  }
  else if(soilmoisturepercent3 >=0.0 && soilmoisturepercent3 <= 100.0)
  {
    moist_3 = soilmoisturepercent3;
  }
  
  return moist_3;
}

float calc_moisture4()
{
  m_val4 = analogRead(SoilSensor4);
  soilMoistureValue4 = (m_val4*3300)/4096;
  
  soilmoisturepercent4 = map(soilMoistureValue4, AirValue4, WaterValue4, 0, 100);
  if(soilmoisturepercent4 > 100.0)
  {
    moist_4 = 100.0;
  }
  else if(soilmoisturepercent4 <0.0)
  {
    moist_4 = 0.0;
  }
  else if(soilmoisturepercent4 >=0.0 && soilmoisturepercent4 <= 100.0)
  {
    moist_4 = soilmoisturepercent4;
  }
  
  return moist_4;
}
//================================================================================================

//========================================FLOW SENSOR FUNCTIONS===================================
void IRAM_ATTR pulseCounter1()
{
  pulseCount1++;
}

void IRAM_ATTR pulseCounter2()
{
  pulseCount2++;
}

float calc_flow1()
{
  currentMillis1 = millis();
  if ((currentMillis1 - previousMillis1 > interval) && (Valve_1 == "ON"))
  {
    pulse1Sec_1 = pulseCount1;
    pulseCount1 = 0;
    
    flowRate1 = ((1000.0 / (millis() - previousMillis1)) * pulse1Sec_1) / calibrationFactor;
    previousMillis1 = millis();  
  }
  else if(Valve_1 == "OFF")
  {
    flowRate1 = 0.0;
  }
  return flowRate1;
}

float calc_flow2()
{
  currentMillis2 = millis();
  if ((currentMillis2 - previousMillis2 > interval) && (Valve_2 == "ON"))
  {
    pulse1Sec_2 = pulseCount2;
    pulseCount2 = 0;
    
    flowRate2 = ((1000.0 / (millis() - previousMillis2)) * pulse1Sec_2) / calibrationFactor;
    previousMillis2 = millis();
  }
  else if(Valve_2 == "OFF")
  {
    flowRate2 = 0.0;
  }
  return flowRate2;
}

unsigned long calc_totalflow1()
{
  flowMilliLitres1 = (flowRate1 / 60) * 1000;

  // Add the millilitres passed in this second to the cumulative total
  totalMilliLitres1 += flowMilliLitres1;

  return totalMilliLitres1;
}

unsigned long calc_totalflow2()
{
  flowMilliLitres2 = (flowRate2 / 60) * 1000;

  // Add the millilitres passed in this second to the cumulative total
  totalMilliLitres2 += flowMilliLitres2;

  return totalMilliLitres2;
}
//=================================================================================================

//====================================FUNCTION FOR RELAY CTRL======================================

void relay1()
{
  float moist_scale_1 = (Soil_1 + Soil_2)/2;

  if((moist_scale_1 > 10.0) && (moist_scale_1 <= Req_1))
  {
    digitalWrite(mosfet, HIGH);
    delay(200);
    digitalWrite(PumpRelay1, LOW);
    digitalWrite(ValveRelay1, LOW);
    delay(200);
    digitalWrite(FlowRelay1, LOW);
    attachInterrupt(digitalPinToInterrupt(FlowSensor1), pulseCounter1, FALLING);
    Valve_1 = "ON";
  }
  else
  {
    digitalWrite(PumpRelay1, HIGH);
    digitalWrite(ValveRelay1, HIGH);
    digitalWrite(mosfet, LOW);
    detachInterrupt(FlowSensor1);
    digitalWrite(FlowRelay1, HIGH);
    Valve_1 = "OFF";
  }
}

void relay2()
{
  float moist_scale_2 = (Soil_3 + Soil_4)/2;

  if((moist_scale_2 > 10.0) && (moist_scale_2 <= Req_2))
  {
    digitalWrite(mosfet, HIGH);
    delay(200);
    digitalWrite(PumpRelay2, LOW);
    digitalWrite(ValveRelay2, LOW);
    delay(200);
    digitalWrite(FlowRelay2, LOW);
    attachInterrupt(digitalPinToInterrupt(FlowSensor2), pulseCounter2, FALLING);
    Valve_2 = "ON";
  }
  else
  {
    digitalWrite(PumpRelay2, HIGH);
    digitalWrite(ValveRelay2, HIGH);
    digitalWrite(mosfet, LOW);
    detachInterrupt(FlowSensor1);
    digitalWrite(FlowRelay2, HIGH);
    Valve_2 = "OFF";
  }
}

//=================================================================================================

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

//====================================FUNCTION FOR DISPLAY===========================================

void updatedisplay()
{
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("SYSTEM 1");
  lcd.setCursor(0, 1);
  lcd.print("Req moisture: ");
  lcd.print(Req_1);
  lcd.setCursor(0, 2);
  lcd.print("Soil_1: ");
  lcd.print(Soil_1);
  lcd.setCursor(0, 3);
  lcd.print("Soil_2: ");
  lcd.print(Soil_2);
  delay(500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Flow sensor: ");
  lcd.print(Flow_1);
  lcd.setCursor(0, 1);
  lcd.print("Water flown: ");
  lcd.print(Qty_1);
  lcd.setCursor(0, 2);
  lcd.print("Valve State: ");
  lcd.print(Valve_1);
  lcd.setCursor(0, 3);
  lcd.print("Pump: ");
  lcd.print(Valve_1);
  delay(500);

  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("SYSTEM 2");
  lcd.setCursor(0, 1);
  lcd.print("Req moisture: ");
  lcd.print(Req_2);
  lcd.setCursor(0, 2);
  lcd.print("Soil_1: ");
  lcd.print(Soil_3);
  lcd.setCursor(0, 3);
  lcd.print("Soil_2: ");
  lcd.print(Soil_4);
  delay(500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Flow sensor: ");
  lcd.print(Flow_2);
  lcd.setCursor(0, 1);
  lcd.print("Water flown: ");
  lcd.print(Qty_2);
  lcd.setCursor(0, 2);
  lcd.print("Valve State: ");
  lcd.print(Valve_2);
  lcd.setCursor(0, 3);
  lcd.print("Pump: ");
  lcd.print(Valve_2);
  delay(500);
}

//===================================================================================================

void setup() 
{
  Serial.begin(115200);
  lcd.init();       
  lcd.backlight();
  lcd.clear();
  temp_sens.begin();

  pinMode(mosfet, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(FlowRelay1, OUTPUT);
  pinMode(FlowRelay2, OUTPUT);
  pinMode(PumpRelay1, OUTPUT);
  pinMode(PumpRelay2, OUTPUT);
  pinMode(ValveRelay1, OUTPUT);
  pinMode(ValveRelay2, OUTPUT);

  digitalWrite(mosfet, LOW);
  digitalWrite(led, LOW);
  digitalWrite(FlowRelay1, HIGH);
  digitalWrite(FlowRelay2, HIGH);
  digitalWrite(PumpRelay1, HIGH);
  digitalWrite(PumpRelay2, HIGH);
  digitalWrite(ValveRelay1, HIGH);
  digitalWrite(ValveRelay2, HIGH);
  
  //---------------------------------Flow sensor setup------------------------------------
  pinMode(FlowSensor1, INPUT_PULLUP);
  pinMode(FlowSensor2, INPUT_PULLUP);

  pulseCount1 = 0;
  flowRate1 = 0.0;
  flowMilliLitres1 = 0;
  totalMilliLitres1 = 0;
  previousMillis1 = 0;

  pulseCount2 = 0;
  flowRate2 = 0.0;
  flowMilliLitres2 = 0;
  totalMilliLitres2 = 0;
  previousMillis2 = 0;
  
//  attachInterrupt(digitalPinToInterrupt(FlowSensor1), pulseCounter1, FALLING);
//  attachInterrupt(digitalPinToInterrupt(FlowSensor2), pulseCounter2, FALLING);
  //-----------------------------------------------------------------------------------

  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after
  //--------------------------------------------------------------------------------------------------

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to get recv packet info
//  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{
  Temp = calc_temp();
  Soil_1 = calc_moisture1();
  Soil_2 = calc_moisture2();
  Soil_3 = calc_moisture3();
  Soil_4 = calc_moisture4();
    
//  Req_1 = sendData.req_1;
//  Req_2 = sendData.req_2;

//  Serial.print("Req_1 = ");
//  Serial.println(Req_1);
//  Serial.print("Req_2 = ");
//  Serial.println(Req_2);

  relay1();
  relay2();

  Flow_1 = calc_flow1();
  Flow_2 = calc_flow2();
  Qty_1 = calc_totalflow1();
  Qty_2 = calc_totalflow2();
  
  // Set values to send
  myData.temp = Temp;
  myData.soil_1 = Soil_1;
  myData.soil_2 = Soil_2;
  myData.flow_1 = Flow_1;
  myData.qty_1 = Qty_1;
  myData.valve_1 = Valve_1;

  myData.soil_3 = Soil_3;
  myData.soil_4 = Soil_4;
  myData.flow_2 = Flow_2;
  myData.qty_2 = Qty_2;
  myData.valve_2 = Valve_2;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
  }
  else 
  {
    Serial.println("Error sending the data");
  }
  
  updatedisplay();
//  delay(1000);
}
