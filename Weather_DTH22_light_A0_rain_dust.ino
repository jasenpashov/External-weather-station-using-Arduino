#include <DHT.h>
#include <ESP8266WiFi.h>
/*
 * dust pin 
Pin  Description
1 : COMMON(GND)
2 : OUTPUT(P2)
3 : INPUT(5VDC 90mA)
4 : OUTPUT(P1)
*/

IPAddress ip;
int mysql_channel     = 25;
const char* ssid      = "";
const char* password  = "";
const char* server_my = "";
#define DHTPIN D1 // what pin we're connected to

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

int pm25  = D5;// Small ( pm2.5)
int pm10  = D6; // Large ( pm10 )
int light_sensor = A0;
int rain_pin = D2;


float pm25_avg   = 0; 
float pm10_avg   = 0; 
float t_avg      = 0; 
float h_avg      = 0; 
int   lux_avg    = 0;
int   rain_detected = 0;
boolean bIsRaining = false;
int count_loop  = 0;
int period      = 15;
int send_data   =  0;
DHT dht(DHTPIN,DHT22,15);
WiFiClient client;
WiFiServer server(80); 

void setup() {
 pinMode(pm25, INPUT);   //  PM2,5 P1
 pinMode(pm10, INPUT);  //  PM10 P2
 pinMode(rain_pin, INPUT);
 
 Serial.begin(115200);
 
 delay(10);
 WiFi.begin(ssid, password);
 
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  server.begin();
  Serial.println("Server started");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  ip = WiFi.localIP();
  Serial.println(ip.toString());
  dht.begin();
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
    float lux = int(Light(analogRead(light_sensor)));  
    
    
    bIsRaining = !(digitalRead(rain_pin));
    if(bIsRaining) rain_detected = 1;
    
      if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
      }
    //----Debug of Values on Serial Port----
     pm25_avg   = pm25_avg  + concSmall; 
     pm10_avg   = pm10_avg  + concLarge; 
     t_avg      = t_avg + t;
     h_avg      = h_avg + h;
     lux_avg    = lux_avg + lux;
     
     Serial.print("PM10Conc: ");
     Serial.print(concLarge);
     Serial.print("ug/m3 , PM2.5Conc: ");
     Serial.print(concSmall);
     Serial.println(" ug/m3");
     Serial.print("Temperature: ");
     Serial.println(t);
     Serial.print("Degrees Celcius Humidity: ");
     Serial.println(h);
     Serial.print("Lux: ");
     Serial.println(lux);
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
void loop() {
   

   if( count_loop  ==  period || send_data ==1 ){
        pm25_avg  = pm25_avg/period;
        pm10_avg  = pm10_avg/period;
        t_avg      = t_avg/period;
        h_avg      = h_avg/period;
        lux_avg    = lux_avg/period;
        
        sendInput(t_avg, h_avg, lux_avg, rain_detected, pm25_avg, pm10_avg);
          count_loop    = 0;
          pm25_avg      = 0;
          pm10_avg      = 0;
          t_avg         = 0;
          h_avg         = 0;
          lux_avg       = 0;
          rain_detected = 0;
          send_data     = 0;
   }

measure();

WiFiClient client = server.available();
    if (!client) {
    return;
    }
    
    // Wait until the client sends some data
    Serial.println("new client");
    while(!client.available()){
    delay(1);
    }
    
    // Read the first line of the request
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();
    if (request.indexOf("/send_data=1") != -1) {
    send_data = 1;
  }


    // Return the response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println(""); // do not forget this one
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    
    client.println("<br><h3>");
    client.println("Click <a href=\"/send_data=1\">here</a> send data<br>");
    client.println("<br></h3>");
    client.println("</html>");
    
    delay(1);
    Serial.println("Client disonnected");
    Serial.println("");
    
    Serial.println("Waiting...");
    delay(1);

  // Recall of the measure program
 }

int sendInput(float t, float h, float lux_avg, int rain, float pm25_avg,  float pm10_avg){  
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
      postStr +="&field14=";
      postStr += String(pm25_avg);
      postStr +="&field15=";
      postStr += String(pm10_avg);
      postStr +="&field_measure1=1";
      postStr +="&field_measure2=2";      
      postStr +="&field_measure7=7";
      postStr +="&field_measure8=8";
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

 
