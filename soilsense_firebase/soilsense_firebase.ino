#include <WiFi.h>
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Ritam"
#define WIFI_PASSWORD "ritam0909"

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

String name_1;
String name_2;
String type_1;
String type_2;
float Area_1 = 0.0;
float Area_2 = 0.0;
float Req_1 = 0.0;
float Req_2 = 0.0;

//---------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

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
  temp = random(0.1,100.0);
  soil_1 = random(0,100);
  soil_2 = random(0,100);
  flow_1 = random(0.0,12.5);
  qty_1 = random(1.1,89.9);
  valve_1 = "OFF";
  soil_3 = random(0,100);
  soil_4 = random(0,100);
  flow_2 = random(0.0,12.5);
  qty_2 = random(1.1,89.9);
  valve_2 = "OFF";
  delay(100);

  fb_send();
  delay(500);
  fb_fetch();

  Serial.println(name_1);
  Serial.println(type_1);
  Serial.println(Area_1);
  Serial.println(Req_1);

  Serial.println(name_2);
  Serial.println(type_2);
  Serial.println(Area_2);
  Serial.println(Req_2);
  
  //--------------------------------------------------------------------------------------------------------

}

void fb_send()
{
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis1 > 4000 || sendDataPrevMillis1 == 0))
  {
    sendDataPrevMillis1 = millis();
    
    if (Firebase.RTDB.setFloat(&fbdo, "Temperature", temp))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    //*******************************************************************************************
    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Soil_1", soil_1))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Soil_2", soil_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Flow_1", flow_1))
    Serial.println("PASSED");
      
    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 1/Qty_1", qty_1))
    Serial.println("PASSED");
      
    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setString(&fbdo, "System 1/Valve_1", valve_1))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    //*******************************************************************************************
    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Soil_3", soil_3))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Soil_4", soil_4))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Flow_2", flow_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setFloat(&fbdo, "System 2/Qty_2", qty_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (Firebase.RTDB.setString(&fbdo, "System 2/Valve_2", valve_2))
    Serial.println("PASSED");

    else 
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
}

void fb_fetch()
{
  if (Firebase.RTDB.getFloat(&fbdo, "system-1-up/Area")) 
  {   
    Area_1 = fbdo.floatData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getFloat(&fbdo, "system-2-up/Area")) 
  {   
    Area_2 = fbdo.floatData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

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

  if (Firebase.RTDB.getString(&fbdo, "system-1-up/CropName")) 
  {   
    name_1 = fbdo.stringData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getString(&fbdo, "system-2-up/CropName")) 
  {   
    name_2 = fbdo.stringData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getString(&fbdo, "system-1-up/CropType")) 
  {   
    type_1 = fbdo.stringData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.RTDB.getString(&fbdo, "system-2-up/CropType")) 
  {   
    type_2 = fbdo.stringData();
  }
  else 
  {
    Serial.println("FETCH FAILED");
    Serial.println(fbdo.errorReason());
  }

}
