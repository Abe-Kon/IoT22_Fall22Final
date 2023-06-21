/**
 * Author @ Abena Osei-Akoto
 * made use of the BasicHTTPClient.ino example
 *
 *  Created and edited: 11.22-12.22
 *
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#include "index.h"
#include "humidity.h"
/*  */
#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT22 
#define USE_SERIAL Serial

WiFiMulti wifiMulti;

/*
const char* ca = \ 
"-----BEGIN CERTIFICATE-----\n" \  
"MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\n" \  
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \  
"DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\n" \  
"SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\n" \  
"GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\n" \  
"AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\n" \  
"q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\n" \  
"SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\n" \  
"Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\n" \  
"a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\n" \  
"/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\n" \  
"AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\n" \  
"CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\n" \  
"bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\n" \  
"c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\n" \  
"VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\n" \  
"ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\n" \  
"MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\n" \  
"Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\n" \  
"AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\n" \  
"uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\n" \  
"wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\n" \  
"X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\n" \  
"PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\n" \  
"KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n" \  
"-----END CERTIFICATE-----\n";
*/



//Pin defs
int ldrPin= 34;
int trigger_pin = 5;
int echo_pin   = 18;
int ledpin= 23;
int relaypin= 21;
float hum;

//EEPROM
#define EEPROM_SIZE 1

//PWM Parameters
int freq= 5000;
int ledChannel=0;
int resolution= 8;
int dutyCycle;

//reading params
float ldrval, distance;
int ledState =LOW;

//Timers
unsigned long previousTimeldr = millis();
long timeIntervalldr = 1000;

unsigned long previousTimeUltra = millis();
long timeIntervalUltra = 2000;

unsigned long previousTimeDHT = millis();
long timeIntervalDHT = 10000;

unsigned long previousTimeLed = millis();
long timeIntervalLed = 200;

DHT dht(DHTPIN, DHTTYPE);
const char* ssid = "Ttbi";
const char* password = "yaakonadu1";

WebServer server(80);

//function implemention
void handleRoot();
void handleNotFound();

void handleRoot() {
   server.send(200, "text/html",page);
}
void hpage() {
   server.send(200, "text/html",page1);
}
void humidityy(){
    //Check WiFi connection status
  String message = "Humidity: ";
  message += hum;
  message += "\n\n ";
   server.send(200, "text/html", message);
}
void start(){
  digitalWrite(relaypin,LOW);
}
void stop(){
  digitalWrite(relaypin, HIGH);  
}
void Getldr(){
    //Check WiFi connection status
  
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;      
      //String serverPath = "http://example.com/index.html";
    String serverPath = "http://192.168.208.242/IoT/Abena_final/getLDR.php";
    // String serverPath = serverName;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());  //alt use char host[] = "example.com";
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.println("-----===------====+++");
        Serial.println(serverPath);
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
        server.send(200, "text/html", payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
}
void Getultra(){
    //Check WiFi connection status
  
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;      
      //String serverPath = "http://example.com/index.html";
    String serverPath = "http://192.168.208.242/IoT/Abena_final/getUltra.php";
    // String serverPath = serverName;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());  //alt use char host[] = "example.com";
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.println("-----===------====+++");
        Serial.println(serverPath);
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
        server.send(200, "text/html", payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
}
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup() {
    dht.begin();
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
    pinMode(ledpin, OUTPUT);
    pinMode(relaypin, OUTPUT);

    USE_SERIAL.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid,password);
    EEPROM.begin(EEPROM_SIZE);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

      // wifiMulti.addAP("Ttbi", "yaakonadu1");
    // wifiMulti.addAP("DUFIE-HOSTEL", "Duf1e@9723");
    wifiMulti.addAP(ssid, password);
     while(WiFi.status() != WL_CONNECTED){
    delay(500);
     }
    USE_SERIAL.println("Connected to WiFi network with IP Address: ");
    USE_SERIAL.println(WiFi.localIP());
    if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");}

    ArduinoOTA.setHostname("AbsESP32");
    ArduinoOTA.setPassword("pass");
    ArduinoOTA.begin();
    server.on("/", handleRoot);
    server.on("/ldr", Getldr);
    server.on("/ultra", Getultra);
    server.on("/start", start);
    server.on("/stop", stop);
    server.on("/hval", hpage);
    server.on("/humidityy", humidityy);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
    digitalWrite(relaypin,HIGH);

}

void loop() {
    server.handleClient();
    delay(2);
    unsigned long currentTime = millis();
        // LED
          if(currentTime - previousTimeLed > timeIntervalLed) {
            previousTimeLed = currentTime;
            if (ledState == HIGH) {
              ledState = LOW;
              digitalWrite(ledpin, ledState);
            }
            else {
              ledState = HIGH;
              digitalWrite(ledpin, ledState);
            }
            
          }    
    // wait for WiFi connection
    if((wifiMulti.run() == WL_CONNECTED)) {
          // LDR    
        if(currentTime - previousTimeldr > timeIntervalldr) {
         previousTimeldr = currentTime;
        ldrval= analogRead(ldrPin);}
    if(currentTime - previousTimeUltra > timeIntervalUltra) {
        previousTimeUltra = currentTime;
        digitalWrite(trigger_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin, LOW);

        long duration = pulseIn(echo_pin, HIGH);
        distance = (duration / 2) / 29.09;
    }
    //  // DHT 
  if(currentTime - previousTimeDHT > timeIntervalDHT) {
    previousTimeDHT = currentTime;
    USE_SERIAL.printf("DHTxx test!\n");
     // Reading temperature or humidity takes about 250 milliseconds!
    float humidity = dht.readHumidity();
    // USE_SERIAL.printf("%f\n",humidity);
    EEPROM.write(0, humidity);
    EEPROM.commit();
    USE_SERIAL.printf("level saved in flash memory");
    hum= EEPROM.read(0);
     USE_SERIAL.printf("%f\n",hum);
  }
 
        HTTPClient http;

        USE_SERIAL.print("[HTTP] begin...\n");
        // configure traged server and url
        //http.begin("https://www.howsmyssl.com/a/check", ca); //HTTPS
        http.begin("http://192.168.208.242/IoT/Abena_final/final_insert.php?ldr="+String(ldrval)+"&distance="+String(distance)); //HTTP

        USE_SERIAL.print("[HTTP] GET...\n");
        // start connection and send HTTP header
        int httpCode = http.GET();

        // httpCode will be negative on error
        if(httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            USE_SERIAL.printf("[HTTP] GET... code: %d\n", httpCode);

            // file found at server
            if(httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
                USE_SERIAL.println(payload);
            }
        } else {
            USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
    }
    ArduinoOTA.handle();
    delay(1000);
}


