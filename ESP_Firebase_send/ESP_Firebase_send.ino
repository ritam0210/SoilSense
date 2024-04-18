#include <WiFi.h>
#include <FirebaseESP32.h>

#define FIREBASE_HOST "https://soilsense-bc90c-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyAG9_-Ca9R9wBuoZy0mgKd4i4Ev_d1K69c"

#define WIFI_SSID "TIG_NT"
#define WIFI_PASSWORD "Tig@1566"

//Define FirebaseESP32 data object
FirebaseData firebaseData;
FirebaseJson json;

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

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  /*
  This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
  Firewall that allows only GET and POST requests.
  
  Firebase.enableClassicRequest(firebaseData, true);
  */

  //String path = "/data";
  

  Serial.println("------------------------------------");
  Serial.println("Connected...");
  
}

void loop()
{
  int Sdata = random(0,4096) ;
  String Sstring = "OFF";
  Serial.println(Sdata); 
  delay(100); 
  
  json.set("/Soil_sensor1", Sdata-80);
  json.set("/Soil_sensor2", Sdata);
  json.set("/Valve_state", Sstring);
  json.set("/Flow_sensor", Sdata);
  Firebase.updateNode(firebaseData,"/System 1",json);

  json.set("/Soil_sensor1", Sdata-100);
  json.set("/Soil_sensor2", Sdata-40);
  json.set("/Valve_state", Sstring);
  json.set("/Flow_sensor", Sdata);
  Firebase.updateNode(firebaseData,"/System 2",json);

}
