//=====================================DECLARING LIBRARIES AND PINS========================================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define FlowSensor1  17
#define FlowSensor2  16
//#define TempSensor   39
#define SoilSensor1  34
#define SoilSensor2  35
#define SoilSensor3  32
#define SoilSensor4  33

#define FlowRelay1   26
#define FlowRelay2   27
#define PumpRelay    14
#define ValveRelay1  12
#define ValveRelay2  13

#define led 2
int ledState = LOW;
unsigned long previousMillis = 0;   // will store last time LED was updated

int valve1_state = LOW;
int valve2_state = LOW;

const int oneWireBus = 39; 
//================================================================================================

//================================================================================================
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature temp_sens(&oneWire);
float Temperature = 0.0;
//================================================================================================

//======================================Soil sensor declarations====================================
const int AirValue1 = 2050;   //2110
const int WaterValue1 = 805;  //805

//const int AirValue2 = 2050;   //2110
//const int WaterValue2 = 805;  //805

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
float Capacity_1;
float Capacity_2;
String Valve_1= "OFF";
String Valve_2= "OFF";

float Req_1 = 65.0;
float Req_2 = 68.0;

// Variable to store if sending data was successful
String success;

//-------------------------------------------------------------------------------------
uint8_t RxMACaddress[] = {0x40, 0x22, 0xD8, 0x7B, 0x64, 0xD8};    //70:B8:F6:5B:FE:48

//-------------------------------------------------------------------------------------
//==================================ESP NOW RECEIVER STRUCTURE=======================================
//
//typedef struct RxStruct
//{
//    float req_1;
//    float req_2;
//    
//}RxStruct;
//
////Create a struct_message called myData
//RxStruct recvData;

//==================================ESP NOW SENDER STRUCTURE=======================================

//Structure example to send data
//Must match the receiver structure
typedef struct TxStruct
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
    
}TxStruct;

//Create a struct_message called myData
TxStruct sendData;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
//void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//  memcpy(&recvData, incomingData, sizeof(recvData));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
//  Req_1 = recvData.req_1;
//  Req_2 = recvData.req_2;
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
  m_val1 = analogRead(SoilSensor1);  //put Sensor insert into soil
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
  m_val2 = analogRead(SoilSensor2);  //put Sensor insert into soil
  soilMoistureValue2 = (m_val2*3300)/4096;
  
  soilmoisturepercent2 = map(soilMoistureValue2, AirValue1, WaterValue1, 0, 100);
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
  m_val3 = analogRead(SoilSensor3);  //put Sensor insert into soil
  soilMoistureValue3 = (m_val3*3300)/4096;
  
  soilmoisturepercent3 = map(soilMoistureValue3, AirValue1, WaterValue1, 0, 100);
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
  m_val4 = analogRead(SoilSensor4);  //put Sensor insert into soil
  soilMoistureValue4 = (m_val4*3300)/4096;
  
  soilmoisturepercent4 = map(soilMoistureValue4, AirValue1, WaterValue1, 0, 100);
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
  if (currentMillis1 - previousMillis1 > interval) 
  {
    pulse1Sec_1 = pulseCount1;
    pulseCount1 = 0;
    
    flowRate1 = ((1000.0 / (millis() - previousMillis1)) * pulse1Sec_1) / calibrationFactor;
    previousMillis1 = millis();  
  }
  return flowRate1;
}

float calc_flow2()
{
  currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 > interval) 
  {
    pulse1Sec_2 = pulseCount2;
    pulseCount2 = 0;
    
    flowRate2 = ((1000.0 / (millis() - previousMillis2)) * pulse1Sec_2) / calibrationFactor;
    previousMillis2 = millis();
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
void relay_trig_high1()
{
  digitalWrite(FlowRelay1, HIGH);
  digitalWrite(PumpRelay, HIGH);
  digitalWrite(ValveRelay1, HIGH);
}

void relay_trig_high2()
{
  digitalWrite(FlowRelay2, HIGH);
  digitalWrite(PumpRelay, HIGH);
  digitalWrite(ValveRelay2, HIGH);
}

void relay_trig_low()
{
  digitalWrite(FlowRelay1, LOW);
  digitalWrite(FlowRelay2, LOW);
  digitalWrite(PumpRelay, LOW);
  digitalWrite(ValveRelay1, LOW);
  digitalWrite(ValveRelay2, LOW);
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
  lcd.print("      SYSTEM 1      ");
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
  lcd.print(Capacity_1);
  lcd.setCursor(0, 2);
  lcd.print("Valve State: ");
  lcd.print(Valve_1);
  lcd.setCursor(0, 3);
  lcd.print("Pump: ");
  lcd.print("OFF");
  delay(500);

  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("      SYSTEM 2      ");
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
  lcd.print(Capacity_2);
  lcd.setCursor(0, 2);
  lcd.print("Valve State: ");
  lcd.print(Valve_2);
  lcd.setCursor(0, 3);
  lcd.print("Pump: ");
  lcd.print("OFF");
  delay(500);
}

//===================================================================================================

void setup() 
{
  Serial.begin(115200);
  lcd.init();       
  lcd.backlight();            // Make sure backlight is on
  lcd.clear();
  temp_sens.begin();
  
  pinMode(led, OUTPUT);
  pinMode(FlowRelay1, OUTPUT);
  pinMode(FlowRelay2, OUTPUT);
  pinMode(PumpRelay, OUTPUT);
  pinMode(ValveRelay1, OUTPUT);
  pinMode(ValveRelay2, OUTPUT);

  digitalWrite(led, LOW);
  digitalWrite(FlowRelay1, LOW);
  digitalWrite(FlowRelay2, LOW);
  digitalWrite(PumpRelay, LOW);
  digitalWrite(ValveRelay1, LOW);
  digitalWrite(ValveRelay2, LOW);
  
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
  
  attachInterrupt(digitalPinToInterrupt(FlowSensor1), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FlowSensor2), pulseCounter2, FALLING);
  //-----------------------------------------------------------------------------------

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, RxMACaddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() 
{
  Temp = calc_temp();
  Soil_1 = calc_moisture1();
  Soil_2 = calc_moisture2();
  Soil_3 = calc_moisture3();
  Soil_4 = calc_moisture4();
  Flow_1 = calc_flow1();
  Flow_2 = calc_flow2();
  Capacity_1 = calc_totalflow1();
  Capacity_2 = calc_totalflow2();
//  Valve_1 = valve1_state;
//  Valve_2 = valve2_state;

  if(Soil_1 <= Req_1 || Soil_2 <= Req_1)
  {
    relay_trig_high1();
    Valve_1 = "ON";
  }
  else if(Soil_1 > Req_1 || Soil_2 > Req_2)
  {
    relay_trig_low();
    Valve_1 = "OFF";
  }

  if(Soil_3 <= Req_2 || Soil_4 <= Req_2)
  {
    relay_trig_high2();
    Valve_2 = "ON";
  }
  else if(Soil_3 > Req_2 || Soil_4 > Req_2)
  {
    relay_trig_low();
    Valve_2 = "OFF";
  }
  
  //-----------------------------------------------------------------------------------
  // Set values to send
  sendData.temp = Temp;
  sendData.soil_1 = Soil_1;
  sendData.soil_2 = Soil_2;
  sendData.soil_3 = Soil_3;
  sendData.soil_4 = Soil_4;
  sendData.flow_1 = Flow_1;
  sendData.flow_2 = Flow_2;
  sendData.capacity_1 = Capacity_1;
  sendData.capacity_2 = Capacity_2;
  sendData.valve_1 = Valve_1;
  sendData.valve_2 = Valve_2;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(RxMACaddress, (uint8_t *) &sendData, sizeof(sendData));
   
  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
    blinkit();
  }
  else 
  {
    Serial.println("Error sending the data");
  }
  updatedisplay();
  delay(1000);
}
