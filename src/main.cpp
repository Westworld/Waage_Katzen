#include "EspMQTTClient.h"
#include <WiFiUdp.h>
#include "HX711.h"
#include "main.h"

#define UDPDEBUG 1
#ifdef UDPDEBUG
WiFiUDP udp;
const char * udpAddress = "192.168.0.95";
const int udpPort = 19814;
#endif

uint32_t mLastTime = 0;
uint32_t mTimeSeconds = 0;

float GewichtMittel[11];
int GewichtAnzahl=0;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = D6; //A2;
const int LOADCELL_SCK_PIN = D5; // A3;

// wiring OLED D1=SCL, D2=SDA
// Taster D7
//bool ButtonWasClicked = false;

const int Buzzer =D7;

//#define Buddy  // definition in platformio.ini, wählen env aus!!!
// nicht für Timmi nehmen, dafür Timmi WS2812

#ifdef testcat
  #define Katze "testcat"
  #define Hostname "Scaletest"

  float KatzeGewichtStart = 6.5;
float KatzeGewichtEnde = 7.2;
const float kalibrierung = 23000.F; //21100.F;
#endif

#ifdef Matti
  #define Katze "Matti"
  #define Hostname "ScaleMatti"
  const float kalibrierung = 21100.F; //19306.F;
  float KatzeGewichtStart = 6.8;
  float KatzeGewichtEnde = 7.8;

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
#define Katze "Mika"
#define Hostname "ScaleMika"
float KatzeGewichtStart = 6.5;
float KatzeGewichtEnde = 7.2;
const float kalibrierung = 23000.F; //21100.F;
#endif

#ifdef Buddy
#define Katze "Buddy"
#define Hostname "ScaleBuddy"
float KatzeGewichtStart = 6.0;
float KatzeGewichtEnde = 6.9;
const float kalibrierung = 21100.F;
#endif


#ifdef Timmi
#define Katze "Timmi"
#define Hostname "ScaleTimmi"
const float kalibrierung = -21600.F;
// 82kg anzeige als
// -22400 = 74kg
// -21400 = 84
// 21100 = 86
// 21600 = 82
#endif

char logString[200];
float Gewicht=0;
float AltGewicht=0;
int Messungen=0;
int TaraCounter = 0;

bool StartupDone=false;

HX711 scale;

EspMQTTClient client(
  WIFI_SSID,
  WIFI_PASS,
  "192.168.0.46",  // MQTT Broker server ip
  "Enzel",   // Can be omitted if not needed
  "hausen",   // Can be omitted if not needed
  Katze,     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient

void onConnectionEstablished()
{
  Serial.println(String(Katze)+ " started");
  client.publish("mytopic/test", String(Katze)+ " started"); // You can activate the retain flag by setting the third parameter to true
 

 StartupDone = true;

UDBDebug(String(Katze)+ " started");

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
  display.setCursor(10, 10);
  display.print(("Matti"));

      display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(10, 25);
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

}




void setup() {
  Serial.begin(115200);
  Serial.println("CatDogScale");

  client.enableDebuggingMessages(); 
  client.enableOTA("", 8266); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.setOnConnectionEstablishedCallback(onConnectionEstablished); 
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
 
}




void loop() {
    float Gelesen=0;
  
    client.loop();


  if (StartupDone) {
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
        
      }
      delay(20);
}

  float BerechneDurchschnitt(float neu) {
    if (GewichtAnzahl>9) {
      for (int i = 0; i<9; i++) {
        GewichtMittel[i] = GewichtMittel[i+1];
      }
      GewichtAnzahl--;
    }

    GewichtMittel[GewichtAnzahl++] = neu;

    float mittel = 0;
    for (int i = 0; i<GewichtAnzahl; i++) {
        mittel += GewichtMittel[i];
    }
    return (mittel / GewichtAnzahl);  
  }


  // #############################################################

  void SendeStatus(float Gewicht, int warum, float Gelesen) {
    UDBDebug("Waage "+String(Katze)+" " +String(Gewicht));

    float sende = roundf(Gewicht * 100) / 100;
    if (sende > 2)
      //MQTT_Send("display/Gewicht", sende);
      UDBDebug(String(Katze)+" gewogen: "+String(Gelesen));

    if ((sende > KatzeGewichtStart) && (sende < KatzeGewichtEnde)) {
      sende = BerechneDurchschnitt(Gewicht);
      sende = roundf(sende * 100) / 100;
      //UDBDebug(String(Katze)+" gewogen: "+String(Gelesen)+" Durchschnitt: "+String(sende));
      MQTT_Send("HomeServer/Tiere/"+String(Katze), String(sende));
    }  

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
    client.publish(topic, value);
   UDBDebug(String(topic)+" - "+value);
}

void MQTT_Send(char const * topic, String value) {
    Serial.println("MQTT " +String(topic)+" "+value) ;
    client.publish(topic, value.c_str());

    UDBDebug(String(topic)+" - "+value);
}

void MQTT_Send(char const * topic, float value) {
    char buffer[20];
    snprintf(buffer, 20, "%f", value);
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