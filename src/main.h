void RealhandleInterrupt();
void SendeStatus(float Gewicht, int warum, float Gelesen);
void WIFI_Connect();
void mydelay(long thedelay);
void UDBDebug(String message);
void MQTT_Send(char const * topic, String value);
void MQTT_Send(char const * topic, float value); 
void MQTT_Send(char const * topic, int16_t value);
void MQTT_Send(char const * topic, long value);
void MQTT_callback(char* topic, byte* payload, unsigned int length);