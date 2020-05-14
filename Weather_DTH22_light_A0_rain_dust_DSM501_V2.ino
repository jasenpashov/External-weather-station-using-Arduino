/* 

https://github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/simple_server/simple_server.ino
https://raw.githubusercontent.com/RuiSantosdotme/Random-Nerd-Tutorials/master/Projects/ESP8266/ESP8266_SPIFFS/ESP8266_SPIFFS.ino
https://create.arduino.cc/projecthub/mircemk/ppd42ns-arduino-air-quality-monitor-in-3d-printed-enclosure-f42573
http://wiki.seeedstudio.com/Grove-Dust_Sensor/

 * dust pin 
Pin  Description
1 : COMMON(GND)
2 : OUTPUT(P2)
3 : INPUT(5VDC 90mA)
4 : OUTPUT(P1)
5 : INPUT(T1)･･･FOR THRESHOLD FOR [

*/
#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <SparkFunTSL2561.h>
#include <DHT.h> //DHT sensor library

#include <DNSServer.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)

#include <ESPAsyncTCP.h>
#endif
#include "ESPAsyncWebServer.h"

extern "C" {
  #include <osapi.h>
  #include <os_type.h>
}

#include "config.h"
//const char* ssid      = "";
const char* ssid      = "";
const char* password  = "";
const char* server_my = "";

WiFiUDP ntpUDP;

#define DHTPIN D3
DHT dht(DHTPIN,DHT22,15);
SFE_TSL2561 light; //SCL->D1 |  SDA->D2
IPAddress ip;
long timeInterval ;
int mysql_channel  = 25;// Външе


long intervalTransmitWeather = 0;
int lastTime =0; 
// Global variables:
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds

//Set variables for PM10 and PM2,5 readings
 unsigned long starttime;
 unsigned long sampletime_ms = 60000; // TIME BETWEEN MEASURES AND UPDATES

 unsigned long triggerOnP2;
 unsigned long triggerOffP2;
 unsigned long pulseLengthP2;
 unsigned long durationP2;
 boolean valP2 = HIGH;
 boolean triggerP2 = false;
 float ratioP2 = 0;
 float countP2;
 float concLarge;

 unsigned long triggerOnP1;
 unsigned long triggerOffP1;
 unsigned long pulseLengthP1;
 unsigned long durationP1;
 boolean valP1 = HIGH;
 boolean triggerP1 = false;
 float ratioP1 = 0;
 float countP1;
 float concSmall;

int pm25  = D6;// Small ( pm2.5) PM2,5 P1
int pm10  = D5; // Large ( pm10 )  PM10 P2
int rain_pin = D7;
int UVsensorIn = A0; //Output from the sensor


float pm25_avg   = 0; 
float pm10_avg   = 0; 
float srPM10_avg = 0;
float srPM25_avg = 0;
float t_avg      = 0; 
float h_avg      = 0; 
float UV_avg     = 0; 
int   lux_avg    = 0;
int   rain_detected = 0;
boolean bIsRaining = false;
int count_loop  = 0;
int send_data   =  0;




AsyncWebServer server(80);
const char* PARAM_MESSAGE = "message";

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}


void setup() {
  Serial.begin(9600);
  delay(10);
  dht.begin();
  pinMode(pm25, INPUT);   //  PM2,5 P1
  pinMode(pm10, INPUT);  //  PM10 P2
  pinMode(rain_pin, INPUT);
  pinMode(UVsensorIn, INPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    return;
  }
  Serial.println();
  Serial.print("IP Address : ");
  Serial.println(WiFi.localIP());
  ip = WiFi.localIP();  

//------------ server ----------------------------
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      AsyncResponseStream *response = request->beginResponseStream("text/html");
      response->print("<!DOCTYPE html><html><head><title>Weather DTH AsyncWebServer</title>");
      response->print("<link rel='stylesheet' href='https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/css/bootstrap.min.css' integrity='sha384-Vkoo8x4CGsO3+Hhxv8T/Q5PaXtkKtu6ug5TOeNV6gBiFeWPGFN9MuhOf23Q9Ifjh' crossorigin='anonymous'>");
      response->print("</head><body>");
      response->print("<script src='https://code.jquery.com/jquery-3.4.1.slim.min.js' integrity='sha384-J6qa4849blE2+poT4WnyKhv5vZF5SrPo0iEjwBvKU7imGFAV0wwj1yYfoRSJoZ+n' crossorigin='anonymous'></script>");
      response->print("<script src='https://cdn.jsdelivr.net/npm/popper.js@1.16.0/dist/umd/popper.min.js' integrity='sha384-Q6E9RHvbIyZFJoft+2mJbHaEWldlvI9IOYy5n3zV9zzTtmI3UksdQRVvoxMfooAo' crossorigin='anonymous'></script>");
      response->print("<script src='https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/js/bootstrap.min.js' integrity='sha384-wfSDF2E50Y2D1uUdj0O3uMBJnjuUD4Ih7YwaYd1iqfktj0Uod8GCExl3Og8ifwB6' crossorigin='anonymous'></script>");
      response->print("<H1>Weather DTH AsyncWebServer</H1>");
      response->print("<div class='container-fluid'>");
      response->printf("<p>You were trying to reach: http://%s%s</p>", request->host().c_str(), request->url().c_str());
      response->print("<p><a href='temperature'>temperature: ");
      response->print(getTemperature().c_str());
      response->print("</a></p>");
      response->print("<p><a href='humidity'>humidity: ");
      response->print(getHumidity().c_str());
      response->print("</a></p>");
      response->print(getState().c_str());
      response->print("<p><a href='sendWeather'>send Weather to my server</a></p>");
      response->print("</div>");
      response->print("</body></html>");
      request->send(response);
    });
    
    server.on("/sendWeather", HTTP_GET, [](AsyncWebServerRequest *request){
        send_data =1;
        request->send(200, "text/plain", "send weather");
    });

    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", getTemperature().c_str());
    });
    
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", getHumidity().c_str());
    });

    // Send a GET request to <IP>/set?power=<message>
    server.on("/set", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam("power")) {
            message = request->getParam("power")->value();
        } else {
            message = "No power sent";
        }
        request->send(200, "text/plain", "set Power: " + message);
        //setStart setStop  setAuto
        if(message=="on"){
          setStart();
        }
        if(message=="off"){
          setStop();
        }
        if(message=="auto"){
          setAuto();
        }
    });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message);
    });

    // Send a POST request to <IP>/post with a form field message set to <message>
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
    });

    server.onNotFound(notFound);
    server.begin();

  light.begin();
  unsigned char ID;
  if (light.getID(ID))
  {
    Serial.print("Got factory ID: 0X");
    Serial.print(ID,HEX);
    Serial.println(", should be 0X5X");
  } else
  {
    byte error = light.getError();
    printError(error);
  }
  gain = 0;
  unsigned char time = 2;
  Serial.println("Set timing...");
  light.setTiming(gain,time,ms);

  Serial.println("Powerup...");
  light.setPowerUp(); 

//------------ server ----------------------------

intervalTransmitWeather = calcTimeStop(millis(), 20);

}

void loop()
{ 
  long duration;
  long currentMillis = millis();  
  measure();

  if( currentMillis >= intervalTransmitWeather || send_data == 1){
    pm25_avg  = pm25_avg/count_loop;
    pm10_avg  = pm10_avg/count_loop;
    t_avg     = t_avg/count_loop;
    h_avg     = h_avg/count_loop;
    lux_avg   = lux_avg/count_loop;
    UV_avg    = UV_avg/count_loop;
    srPM10_avg = srPM10_avg/count_loop;
    srPM25_avg = srPM25_avg/count_loop;

    sendWeather(t_avg, h_avg, UV_avg, lux_avg, rain_detected, pm25_avg, pm10_avg);
    intervalTransmitWeather = calcTimeStop(currentMillis, 20);
    Serial.println("sendWeather");
    Serial.print(intervalTransmitWeather);
    Serial.print("||");
    Serial.print(currentMillis);

    count_loop    = 0;
    pm25_avg      = 0;
    pm10_avg      = 0;
    t_avg         = 0;
    h_avg         = 0;
    lux_avg       = 0;
    UV_avg        = 0;
    rain_detected = 0;
    send_data     = 0;
    srPM10_avg    = 0;
    srPM25_avg    = 0;
  }
   
  delay(100);
}//loop

int calcTimeStop(long currentMillis, int periodMinute){
    const unsigned long oneSecond = 1000;  //the value is a number of milliseconds, ie 1 second
    long rz = currentMillis+(oneSecond*(periodMinute*60));
    return rz;
}

void measure(){
  
     valP1 = digitalRead(pm25); // Small ( pm2.5)
     valP2 = digitalRead(pm10); // Large ( pm10 )
  
    
    //--------PM2,5-------------
    
    if(valP1 == LOW && triggerP1 == false){
     triggerP1 = true;
     triggerOnP1 = micros();
     }
    
    if (valP1 == HIGH && triggerP1 == true){
     triggerOffP1 = micros();
     pulseLengthP1 = triggerOffP1 - triggerOnP1;
     durationP1 = durationP1 + pulseLengthP1;
     triggerP1 = false;
     }
    
    //-----------PM10------------
    
    if(valP2 == LOW && triggerP2 == false){
     triggerP2 = true;
     triggerOnP2 = micros();
     }
    
    if (valP2 == HIGH && triggerP2 == true){
     triggerOffP2 = micros();
     pulseLengthP2 = triggerOffP2 - triggerOnP2;
     durationP2 = durationP2 + pulseLengthP2;
     triggerP2 = false;
     }
    
    //----------Calcolo-----------
    
    if ((millis() - starttime) > sampletime_ms) {
    
    // Integer percentage 0=>100
     ratioP1 = durationP1/(sampletime_ms*10.0); // Integer percentage 0=>100
     ratioP2 = durationP2/(sampletime_ms*10.0);
     countP1 = 1.1*pow(ratioP1,3)-3.8*pow(ratioP1,2)+520*ratioP1+0.62;
     countP2 = 1.1*pow(ratioP2,3)-3.8*pow(ratioP2,2)+520*ratioP2+0.62;
     float PM10count = countP2;
     float PM25count = countP1 - countP2;
    
    //PM10 count to mass concentration conversion
     double r10 = 2.6*pow(10,-6);
     double pi = 3.14159;
     double vol10 = (4/3)*pi*pow(r10,3);
     double density = 1.65*pow(10,12);
     double mass10 = density*vol10;
     double K = 3531.5;
     concLarge = (PM10count)*K*mass10;
    
    //PM2.5 count to mass concentration conversion
     double r25 = 0.44*pow(10,-6);
     double vol25 = (4/3)*pi*pow(r25,3);
     double mass25 = density*vol25;
     concSmall = (PM25count)*K*mass25;
    
    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float uv = UVLight();
    double lux = getLUX();
    
    bIsRaining = !(digitalRead(rain_pin));
    if(bIsRaining) rain_detected = 1;
    
      if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
      }
    //----Debug of Values on Serial Port----
     pm25_avg   = pm25_avg  + ratioP1; 
     pm10_avg   = pm10_avg  + ratioP2; 
     t_avg      = t_avg + t;
     h_avg      = h_avg + h;
     lux_avg    = lux_avg + lux;
     UV_avg     = UV_avg + uv;

     Serial.print("PM10Conc: ");
     Serial.print(ratioP2);
     Serial.print("ug/m3 , PM2.5Conc: ");
     Serial.print(ratioP1);
     Serial.println(" ug/m3");
     Serial.print("Temperature: ");
     Serial.println(t);
     Serial.print("Degrees Celcius Humidity: ");
     Serial.println(h);
     Serial.print("Lux: ");
     Serial.println(lux);
     Serial.print("UV Intensity(mW/cm^2): ");
     Serial.println(uv);
     Serial.print("Rain detected: ");
     Serial.println(bIsRaining);
     Serial.print("Rain_detected: ");
     Serial.println(rain_detected);
  
     count_loop++;
     Serial.print("count_loop: ");
     Serial.println(count_loop);
    //Reset Values
     durationP1 = 0;
     durationP2 = 0;
     starttime = millis();
    }
 }

double Light (int RawADC0)
{
  double Vout=RawADC0*0.0048828125;
  int lux=500/(10*((5-Vout)/Vout));//use this equation if the LDR is in the upper part of the divider
  //int lux=(2500/Vout-500)/10;
  return lux;
}

double getLUX()
{
  // Wait between measurements before retrieving the result
  // (You can also configure the sensor to issue an interrupt
  // when measurements are complete)
  
  // This sketch uses the TSL2561's built-in integration timer.
  // You can also perform your own manual integration timing
  // by setting "time" to 3 (manual) in setTiming(),
  // then performing a manualStart() and a manualStop() as in the below
  // commented statements:
  
  // ms = 1000;
  // light.manualStart();
  delay(ms);
  // light.manualStop();
  
  // Once integration is complete, we'll retrieve the data.
  
  // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are needed for lux calculations.
  
  // Retrieve the data from the device:

  unsigned int data0, data1;
  
  if (light.getData(data0,data1))
  {
    // getData() returned true, communication was successful
    
    Serial.print("data0: ");
    Serial.print(data0);
    Serial.print(" data1: ");
    Serial.print(data1);
  
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.
    
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor
  
    double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(gain,ms,data0,data1,lux);
    
    // Print out the results:
  
    Serial.print(" lux: ");
    Serial.print(lux);
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
    return (lux);
  }
  else
  {
    // getData() returned false because of an I2C error, inform the user.

    byte error = light.getError();
    printError(error);
  }
}

double UVLight (){
   int uvLevel = averageAnalogRead(UVsensorIn);
 
  float outputVoltage = 3.3 * uvLevel/1024;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
 
  Serial.print(" UV Intensity: ");
  Serial.print(uvIntensity);
  Serial.print(" mW/cm^2"); 
  Serial.println(); 
  delay(200);
  return uvIntensity;
}

int averageAnalogRead(int pinToRead){
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);  
 
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setStart() {
 Serial.println("Set Start");

}

void setStop() {
  Serial.println("Set Stop");

}

void setAuto(){
  Serial.println("Auto Power Start");

}

void setInterval(int inputParam){
    Serial.println(inputParam);

}

String getTemperature() {
  float temperature = dht.readTemperature();
  Serial.println(temperature);
  return String(temperature);
}

String getHumidity() {
  float humidity = dht.readHumidity();
  Serial.println(humidity);
  return String(humidity);
}

String getState() {
  String mess ="";
        mess +="<p>Temperature: ";
        mess +=t_avg; 
        mess +="</p>";
        mess +="Humidity: ";
        mess +=h_avg; 
        mess +="</p>";
        mess +="<p>";
        mess +="P 2.5: ";
        mess +=pm25_avg;
        mess +="</p>";
        mess +="<p>";
        mess +="P 10: ";
        mess +=pm10_avg;
        mess +="</p>";
        mess +="<p>";
        mess +="Lux ";
        mess +=lux_avg;
        mess +="</p>";
        mess +="<p>";
        mess +="UV Intensity (mW/cm^2): ";
        mess +=UV_avg;
        mess +="</p>";

        mess +="<p>Count loop: ";
        mess +=count_loop; 
        mess +="</p>";
        mess +="</p>";
        
  return String(mess);
}

void printError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error)
  {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}

void sendWeather(float t,float h, float UV_avg, float lux_avg, int rain, float pm25_avg, float pm10_avg){
  WiFiClient client;
  
  if (client.connect(server_my,80)) {
      Serial.println("POST: ");
      Serial.print("pm25_avg: ");
      Serial.print(pm25_avg);
      Serial.println(" pm10_avg: ");
      Serial.print(pm10_avg);
      Serial.println("Send to Local!!!");


      String postStr = "apiKey";
      postStr +="&field1=";
      postStr += String(t);
      postStr +="&field2=";
      postStr += String(h);
      postStr +="&field7=";
      postStr += String(lux_avg);
      postStr +="&field8=";
      postStr += String(rain);
      postStr +="&field10=";
      postStr += String(UV_avg); 
      postStr +="&field14=";
      postStr += String(pm25_avg);
      postStr +="&field15=";
      postStr += String(pm10_avg);
      postStr +="&field_measure1=1";
      postStr +="&field_measure2=2";
      postStr +="&field_measure7=7";
      postStr +="&field_measure8=8";
      postStr +="&field_measure10=10";
      postStr +="&field_measure14=14";
      postStr +="&field_measure15=15";
      
      postStr +="&channel=";
      postStr += String(mysql_channel);
      postStr +="&ip=";
      postStr += String(ip.toString());
      Serial.println(postStr);

      client.print(String("GET /iot/update_iot.php?"+postStr) + " HTTP/1.1\r\n" +
                   "Host: " + server_my + "\r\n" + 
                   "Connection: close\r\n\r\n");
      delay(10);
  }
  client.stop();
}

