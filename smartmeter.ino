#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>


const int voltagePin = 34;
const float ADC_REF = 3.3;
const int ADC_RES = 4095;
const int sampleCount = 1000;
const int powerthreshold = 800;//in watt
const unsigned long runtimelimit = 12UL * 60 * 60 * 1000; // 12 hours in ms
const unsigned long postInterval = 2UL * 60 * 1000;

// Load runtime tracking
unsigned long load1starttime = 0;
unsigned long load2starttime = 0;
unsigned long load3starttime = 0;
unsigned long lastposttime = 0;

//calibration offset
float zeroOffset1 = 0;
float zeroOffset2 = 0;
float zeroOffset3 = 0;
float zeroVoltageOffset = 0;

//relay pin
const int relay1 = 25;
const int relay2 = 26;
const int relay3 = 27;

// current sensor initiallization;
const int Load1 = 32;
const int Load2 = 33;
const int Load3 = 35;

const float HallSensor_sensitivity = 0.04;
const float ZERO_CURRENT_VOLTAGE = 2.5;

const float voltageSensorScaling = 300.0;

//wifi credentials
const char* ssid = "DESKTOP-E0R8OTB8659";
const char* password="dHS89952";

//Url data
const char* powerURL = "https://smartmeter-project.onrender.com/api/Meter/create";
const char* relayURL ="https://smartmeter-project.onrender.com/api/RelayOptions";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(32, ADC_11db);
  analogSetPinAttenuation(33, ADC_11db);
  analogSetPinAttenuation(35, ADC_11db);
  // delay(100);
  Serial.println("Calibrating zero offsets.... ");
  zeroOffset1 = calibrateZeroCurrent(Load1);
  zeroOffset2 = calibrateZeroCurrent(Load2);
  zeroOffset3 = calibrateZeroCurrent(Load3);

  //intializing the wifi
  WiFi.begin(ssid, password);
  Serial.begin(115200);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print("..");
  }
    Serial.println("connected");
    delay(5000);

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);

}

//fuction to read RMS AC current
float readACCurrent(int pin, float zeroOffset) {
  long sumsq = 0;
  int samples = 500; // enough to capture several AC cycles at 50Hz
  for (int i = 0; i < samples; i++) {
    float voltage = (analogRead(pin) * ADC_REF) / ADC_RES;
    Serial.print("voltage: ");
    Serial.println(voltage);
    Serial.print("zeroOffset: ");
    Serial.println(zeroOffset);
    float centered = voltage - zeroOffset;
    float current = centered / HallSensor_sensitivity;
    sumsq += current * current;
    delayMicroseconds(200); // adjust sampling speed
  }
  float rms = sqrt(sumsq / samples);
  
  // Deadband filter to remove tiny noise readings
  if (fabs(rms) < 0.03) rms = 0;
  
  return rms;
}

//function to calibrate each load at startup
float calibrateZeroCurrent(int pin) {
  long sum = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  float avgRaw = sum / (float)samples;
  return (avgRaw * ADC_REF) / ADC_RES;
}



void sendPowerdata(float power1, float power2, float power3){
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;
    http.begin(powerURL);
    http.addHeader("Content-Type", "application/json; charset=UTF-8");

    DynamicJsonDocument doc(1024);
    doc["power1"] = power1;
    doc["power2"] = power2;
    doc["power3"] = power3;
    doc["timestamp"] = millis();

    String payload;
    serializeJson(doc, payload);
    Serial.println("Sending payload: " + payload);

    int httpResponseCode = http.POST(payload);
     if (httpResponseCode > 0) {
      Serial.print("POST Response: ");
      Serial.println(httpResponseCode);
      Serial.println(http.getString());
    } else {
      Serial.print("POST Error: ");
      Serial.println(http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  }else{
    Serial.println("WiFi not connected");
  }
}

void checkRelayControl() {
  if(WiFi.status() == WL_CONNECTED){
    HTTPClient http;
    http.begin(relayURL);

    int httpResponseCode = http.GET();
    Serial.print("HTTP Response Code: ");
    Serial.println(httpResponseCode);

    if(httpResponseCode == 200){
      String Response = http.getString();
      Serial.println("Raw response: " + Response);
      
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, Response);

    if(!error){
      JsonObject obj = doc[0];
      int relay1state = obj["relay1"];
      int relay2state = obj["relay2"];
      int relay3state = obj["relay3"];

      digitalWrite(relay1, relay1state);
      digitalWrite(relay2, relay2state);
      digitalWrite(relay3, relay3state);
      Serial.print("relay1 state: ");
      Serial.println(relay1state);

      Serial.print("relay2 state: ");
      Serial.println(relay2state);

      Serial.print("relay3 state: ");
      Serial.println(relay3state);
    }else{
      Serial.print("GET ERROR: ");
      Serial.println(httpResponseCode);
    }
    http.end();
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float sumsq = 0;

  for(int i=0; i < sampleCount; i++){
    int raw = analogRead(voltagePin);
    float voltage = (raw * ADC_REF) / ADC_RES;
    float  centered = voltage - (ADC_REF/2);
    sumsq += centered * centered;
    delay(1);

  }

  float rms = sqrt(sumsq/sampleCount);
  float mainVoltage = rms * voltageSensorScaling;


  //reading each current values from the load
  float load1 = readACCurrent(Load1, zeroOffset1);
  float load2 = readACCurrent(Load2, zeroOffset2);
  float load3 = readACCurrent(Load3, zeroOffset3);


  //checking whether the loads are on
  bool load1on = (load1 > 0.3);
  bool load2on = (load2 > 0.3);
  bool load3on = (load3 > 0.3);

  //tracking the time;
  unsigned long now = millis();
  if(load1on && load1starttime == 0) load1starttime = now;
  if(load2on && load2starttime == 0) load2starttime = now;
  if(load3on && load3starttime == 0) load3starttime = now;

  if(!load1on) load1starttime = 0;
  if(!load2on) load2starttime = 0;
  if(!load3on) load3starttime = 0;

  //power computations
  float loadpower1 = mainVoltage * load1;
  float loadpower2 = mainVoltage * load2;
  float loadpower3 = mainVoltage * load3;


  
  float loadlist[] = {loadpower1, loadpower2, loadpower3};
  int listsize = sizeof(loadlist)/sizeof(loadlist[0]);

  String loadtype = "";
  int inductivecount = 0;
  for(int i=0; i<listsize; i++){
    int value = loadlist[i];
    if(value > 30){
      loadtype += "resistive";
    }else if(value > 500){

      loadtype += "inductive";
      if(load1on) inductivecount++;
      if(load2on) inductivecount++;
      if(load3on) inductivecount++;

    }else{
      loadtype += "low power";
    }
  }

  if(inductivecount >= 2){
    if(loadtype == "inductive" && load1on && loadpower1>powerthreshold && (now - load1starttime >= runtimelimit))
    digitalWrite(relay1, HIGH);
  }

  if(inductivecount >= 2){
    if(loadtype == "inductive" && load2on && loadpower2>powerthreshold && (now - load2starttime >= runtimelimit))
    digitalWrite(relay2, HIGH);
  }

  if(inductivecount >= 2){
    if(loadtype == "inductive" && load3on && loadpower3>powerthreshold && (now - load3starttime >= runtimelimit))
    digitalWrite(relay3, HIGH);
  }

  //send data every 2omins
  if(millis() - lastposttime >= postInterval){
    sendPowerdata(loadpower1, loadpower2, loadpower3);
    lastposttime = millis();
  }

  //remote control command every 1sec
  static unsigned long lastchecktime = 0;
  if(millis() - lastchecktime >= 1000){
    checkRelayControl();
    lastchecktime = millis();
  }

  
  Serial.print("Ac Voltage(rms): ");
  Serial.print(mainVoltage);
  Serial.println("v");

  //current sensor(load) printout
  
  Serial.print("Load1 current: ");
  Serial.print(load1);
  Serial.println(" A");

  Serial.print("Load2 current: ");
  Serial.print(load2);
  Serial.println(" A");

  Serial.print("Load3 current");
  Serial.print(load3);
  Serial.println(" A");

  Serial.println("-----------------------");


  delay(5000);

}
