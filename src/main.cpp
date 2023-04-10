#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "HX711.h"
#include <PubSubClient.h>
#include "main.h"

const char* host = "192.168.0.59";
const int httpPort = 80;

#define UDPDEBUG 1
#ifdef UDPDEBUG
WiFiUDP udp;
const char * udpAddress = "192.168.0.34";
const int udpPort = 19814;
#endif

const char* MQTT_BROKER = "192.168.0.46";
WiFiClient espClient;
PubSubClient client(espClient);

uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = D6; //A2;
const int LOADCELL_SCK_PIN = D5; // A3;

// wiring OLED D1=SCL, D2=SDA
// Taster D7
//bool ButtonWasClicked = false;

const int Buzzer =D7;

//#define Buddy  // definition in platformio.ini, wählen env aus!!!
// nicht für Timmi nehmen, dafür Timmi WS2812

#ifdef Matti
  #define Katze "ScaleMatti"
  const float kalibrierung = 21100.F; //19306.F;

  // 19306.F  dann 84.4 kg anzeige als 93.3 kg

  // 21100.F  84.68 bei Matti
  // 21100 F  84.45 bei Buddy
  // 23000 F  84.10 bei Mika
  
  
  #include <SPI.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define 	SSD1306_WHITE   1
  
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

 // const int interruptPin = D7;

/*
void ICACHE_RAM_ATTR handleInterrupt() {
  ButtonWasClicked= true;
}
*/
#endif

#ifdef Mika
#define Katze "ScaleMika"
const float kalibrierung = 23000.F; //21100.F;
#endif

#ifdef Buddy
#define Katze "ScaleBuddy"
const float kalibrierung = 21100.F;
#endif


#ifdef Timmi
#define Katze "ScaleTimmi"
const float kalibrierung = -21600.F;
// 82kg anzeige als
// -22400 = 74kg
// -21400 = 84
// 21100 = 86
// 21600 = 82
#endif

#define HOST_NAME Katze

char logString[200];
float Gewicht=0;
float AltGewicht=0;
int Messungen=0;
int TaraCounter = 0;

HX711 scale;


void RemoteSetup() {

   Serial.println("**** Setup: initializing ...");

    // WiFi connection

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(WIFI_SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  
}


void setup() {
  Serial.begin(115200);
  Serial.println("CatDogScale");
  RemoteSetup();
   client.setServer(MQTT_BROKER, 1883);
   client.setCallback(MQTT_callback);

  #ifdef Matti
  Serial.println("Init Display");
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(("SSD1306 allocation failed"));
        delay(3000);
  }
  display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  
  display.setTextSize(3); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.print(("Matti"));
  display.display();
  #endif

    delay(1000);

   //WIFI_Connect();  // in remotedebug
  ArduinoOTA.setHostname(Katze);
  
  ArduinoOTA
    .onStart([]() {
     //  rdebugVln("Start updating");
       #ifdef Matti
        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 50);
        display.print(("update"));
        display.display();
       #endif 
    });
    ArduinoOTA.onEnd([]() {
    //   rdebugVln("End updating");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //  sprintf(logString, "Progress: %u", (progress / (total / 100)));
    //  rdebugVln("%s",logString);
    });
    ArduinoOTA.onError([](ota_error_t error) {
      /*
      debugV("Error[%d]", error);
      if (error == OTA_AUTH_ERROR)  rdebugVln("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)  rdebugVln("Begin Failed");
      else if (error == OTA_CONNECT_ERROR)  rdebugVln("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)  rdebugVln("Receive Failed");
      else if (error == OTA_END_ERROR)  rdebugVln("End Failed");
      */
    });

  ArduinoOTA.begin();  

  //long rssi = WiFi.RSSI();
  //sprintf(logString, " %d", rssi);

    #ifdef Matti
      display.clearDisplay();
      display.setTextSize(1); // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(10, 20);
      display.println(WiFi.localIP());
      display.display();

      //pinMode(interruptPin, INPUT_PULLUP);
      //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);

  #endif

  Serial.println("Init scale");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_scale(kalibrierung); // (223.F);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare(20);				        // reset the scale to 0

  Serial.println("Scale after reset "+scale.read());

#ifdef Matti
  display.clearDisplay();
  display.setTextSize(3); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.print(("Matti"));
  display.display();
#endif

  delay(500);
  UDBDebug(String(Katze)+ " started");
}

void loop() {
    float Gelesen=0;
  
  if (WiFi.status() != WL_CONNECTED)
    {
      WIFI_Connect();
    }
    
  ArduinoOTA.handle();

  if (!client.connected()) {  // MQTT
        while (!client.connected()) {
            client.connect(Katze,MQTT_User,MQTT_Pass);
            delay(100);
            UDBDebug(String(Katze)+" reconnect mqtt");
        }
    }
  client.loop();

#ifdef Matti
  //if (ButtonWasClicked) 
  //  RealhandleInterrupt();
#endif
  
  Gelesen = scale.get_units(10);


  #ifdef Buddy
    if (Gelesen>9) {
      Serial.println("Buzzer");
      for (int i=1000;i<=2000;i++) {
      tone(Buzzer, i); // Im Hauptteil wird nun mit dem Befehl "tone ( x , y )" ein Ton abgegeben.
      delay(1);
      } 
      
      noTone(Buzzer); // Der Ton wird abgeschaltet
    }
  #endif
  

    if ((Gelesen <= (AltGewicht + 0.1)) && (Gelesen >= (AltGewicht - 0.1)) ) {  // gleicher Wert erneut gelesen
      
      if (!((Gelesen <= (Gewicht + 0.1)) && (Gelesen >= (Gewicht - 0.1)) )) {  // Wert noch nicht gesendet
               #ifdef Matti
                  display.clearDisplay();
                  int val_int;
                  int val_fra;
                  val_int = (int) Gelesen;   // compute the integer part of the float 
                  val_fra = round ((Gelesen - (float)val_int) * 10);   // compute 3 decimal places (and convert it to int)

                  sprintf(logString, "%d.%d", val_int, val_fra);
                  display.setTextSize(4); // Draw 2X-scale text
                  display.setTextColor(SSD1306_WHITE);
                  display.setCursor(10, 20);
                  if (val_int>2)
                    display.print(logString);
                  else
                    display.print("   ");
                  display.display();
              #endif
        
              Gewicht = Gelesen;
              SendeStatus(Gelesen, 1, Gelesen); 
              TaraCounter = 0;
      }
     }
     else {
      AltGewicht = Gelesen;
     }

#ifdef Timmi
  if ((Gelesen <= 2.0) && (Gelesen >= -4.8 ))  // war 3.8
#else  
  if ((Gelesen <= 2.0) && (Gelesen >= -3.8 )) 
#endif  
  {
      if (++TaraCounter > 1800) {
        TaraCounter = 0;
       // rdebugVln("Reset Tara");
        scale.tare(20);  
        Gelesen = scale.get_units(10);
        SendeStatus(Gewicht, 3, Gelesen);
                #ifdef Matti
                  display.clearDisplay();
                  display.setTextSize(4); // Draw 2X-scale text
                  display.setTextColor(SSD1306_WHITE);
                  display.setCursor(10, 20);
                  display.print("   ");
                  display.display();
              #endif       
      }
  }



  Messungen++;

  if (Messungen > 30) {
    // eine Minute
    if (Gewicht > 5)
      SendeStatus(Gewicht, 0, Gelesen);
    Messungen = 0;
  }
  mydelay(1000);
  
}

void mydelay(long thedelay) {
unsigned long start = millis();

  while ((start+thedelay)>millis()) {
   // Debug.handle();
    delay(1);
  }
}




// #############################################################
void WIFI_Connect()
{
  WiFi.disconnect();
  Serial.println("Booting Sketch...");
  WiFi.mode(WIFI_STA);
  WiFi.hostname(Katze);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
    // Wait for connection
  for (int i = 0; i < 25; i++)
  {
    if ( WiFi.status() != WL_CONNECTED ) {
      Serial.print ( "." );
      delay ( 500 );
    }
  }

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

}

// #############################################################

void SendeStatus(float Gewicht, int warum, float Gelesen) {

  MQTT_Send("HomeServer/Tiere/"+String(Katze), String(Gewicht));
  UDBDebug("HomeServer/Tiere/"+String(Katze)+" " +String(Gewicht));

  #ifdef Buddy
    if (Gewicht>9) {
      Serial.println("Buzzer");
      tone(Buzzer, 1000); // Im Hauptteil wird nun mit dem Befehl "tone ( x , y )" ein Ton abgegeben.
      delay(1000); // mit einer Dauer von 1 Sekunde
      
      noTone(Buzzer); // Der Ton wird abgeschaltet
      delay(100); // Der Lautsprecher bleibt eine Sekunde aus
      
      tone(Buzzer, 2000); // Im Hauptteil wird nun mit dem Befehl "tone ( x , y )" ein Ton abgegeben.
      delay(1000); // mit einer Dauer von 1 Sekunde
      
      noTone(Buzzer); // Der Ton wird abgeschaltet
    }
  #endif
  
}


void MQTT_callback(char* topic, byte* payload, unsigned int length) {
  UDBDebug("MQTT Callback "+String(topic));
}

void MQTT_Send(String topic, String value) {
    Serial.println("MQTT " +String(topic)+" "+value) ;
    if (!client.publish(topic.c_str(), value.c_str(), true)) {
       UDBDebug("MQTT error");  
    };
      UDBDebug(String(topic)+" - "+value);
}

void MQTT_Send(char const * topic, String value) {
    Serial.println("MQTT " +String(topic)+" "+value) ;
    if (!client.publish(topic, value.c_str(), true)) {
       UDBDebug("MQTT error");  
    };
      UDBDebug(String(topic)+" - "+value);
}

void MQTT_Send(char const * topic, float value) {
    char buffer[10];
    snprintf(buffer, 10, "%f", value);
    MQTT_Send(topic, buffer);
}

void MQTT_Send(char const * topic, int16_t value) {
    char buffer[10];
    snprintf(buffer, 10, "%d", value);
    MQTT_Send(topic, buffer);
}

void MQTT_Send(char const * topic, long value) {
    char buffer[10];
    snprintf(buffer, 10, "%ld", value);
    MQTT_Send(topic, buffer);
}

void UDBDebug(String message) {
#ifdef UDPDEBUG
  udp.beginPacket(udpAddress, udpPort);
  udp.write((const uint8_t* ) message.c_str(), (size_t) message.length());
  udp.endPacket();
#endif  
}