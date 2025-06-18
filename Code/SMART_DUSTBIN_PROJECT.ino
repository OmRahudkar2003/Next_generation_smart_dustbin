#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include "DHT.h"
#include <SimpleTimer.h>
#include <wifi_provisioning/manager.h>

// Set Defalt Values
#define DEFAULT_RELAY_MODE true
#define DEFAULT_Temperature 0
#define DEFAULT_Humidity 0

// BLE Credentils
const char *service_name = "TTPVT";
const char *pop = "TTPVT@321";

// GPIO
static uint8_t gpio_reset = 0;
static uint8_t DHTPIN = 19;
static uint8_t relay = 21;
static uint8_t LDR_PIN      = 39;

static uint8_t WIFI_LED   = 2;
static uint8_t PIR_PIN    = 4;
static uint8_t BUZZER_PIN = 22;

boolean PIR_STATE_NEW     = LOW;  // current state
boolean PIR_STATE_OLD     = LOW;  // previous state

boolean BUZZER_STATE        = false;
unsigned long buzzer_timer  = 0;

bool relay_state = true;
bool wifi_connected = 0;
bool SECURITY_STATE = false;
uint32_t chipId = 0;
float ldrVal  = 0;
DHT dht(DHTPIN, DHT11);

SimpleTimer Timer;

//------------------------------------------- Declaring Devices -----------------------------------------------------//
char device1[] = "SecuritySwitch";
//The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
static Switch SecuritySwitch(device1, NULL);
static TemperatureSensor temperature("Temperature");
static TemperatureSensor humidity("Humidity");
static Switch my_switch("Relay", &relay);
static TemperatureSensor ldr("AIR QUALITY");

void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("\nConnected to Wi-Fi!\n");
      wifi_connected = 1;
      delay(500);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV: {
        Serial.println("\nReceived Wi-Fi credentials");
        Serial.print("\tSSID : ");
        Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid);
        Serial.print("\tPassword : ");
        Serial.println((char const *) sys_event->event_info.prov_cred_recv.password);
        break;
      }
    case ARDUINO_EVENT_PROV_INIT:
      wifi_prov_mgr_disable_auto_stop(10000);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("Stopping Provisioning!!!");
      wifi_prov_mgr_stop_provisioning();
      break;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
  const char *device_name = device->getDeviceName();
  Serial.println(device_name);
  const char *param_name = param->getParamName();

  if(strcmp(device_name, device1) == 0) {
    
    Serial.printf("SecuritySwitch = %s\n", val.val.b? "true" : "false");
    
    if(strcmp(param_name, "Power") == 0) {
        Serial.printf("Received value = %s for %s - %s\n", 
                      val.val.b? "true" : "false", device_name, param_name);
        SECURITY_STATE = val.val.b;
        param->updateAndReport(val);
        //________________________________________________
        if(SECURITY_STATE == true){
         esp_rmaker_raise_alert("Security is ON");
        }
        else{
          esp_rmaker_raise_alert("Security is OFF");
        }
        //________________________________________________
        
    }
  }

  if (strcmp(device_name, "Relay") == 0)
  {
    if (strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      relay_state = val.val.b;
      (relay_state == false) ? digitalWrite(relay, LOW) : digitalWrite(relay, HIGH);
      param->updateAndReport(val);
    }
  }
}


void setup()
{

  Serial.begin(115200);

  // Configure the input GPIOs
  pinMode(gpio_reset, INPUT);
   pinMode(WIFI_LED, OUTPUT);
   digitalWrite(WIFI_LED, LOW);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, DEFAULT_RELAY_MODE);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  //Beginning Sensor
  dht.begin();

  //------------------------------------------- Declaring Node -----------------------------------------------------//
  Node my_node;
  my_node = RMaker.initNode("SMART DUSTBIN SYSTEM");///// PROJECT NAME

  //Standard switch device
  my_switch.addCb(write_callback);
  SecuritySwitch.addCb(write_callback);
  //------------------------------------------- Adding Devices in Node -----------------------------------------------------//
  my_node.addDevice(temperature);
  my_node.addDevice(humidity);
  my_node.addDevice(my_switch);
  my_node.addDevice(SecuritySwitch);
  my_node.addDevice(ldr);


  //This is optional
  RMaker.enableOTA(OTA_USING_PARAMS);
  //If you want to enable scheduling, set time zone for your region using setTimeZone().
  //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
  // RMaker.setTimeZone("Asia/Shanghai");
  // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
  RMaker.enableTZService();
  RMaker.enableSchedule();

 for(int i=0; i<17; i=i+8) {
  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
 }

  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  // Timer for Sending Sensor's Data
  //Timer.setInterval(5000);

  WiFi.onEvent(sysProvEvent);

#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
SecuritySwitch.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);

}


void loop()
{

  if (Timer.isReady() && wifi_connected) {                    // Check is ready a second timer
    Serial.println("Sending Sensor's Data");
    Send_Sensor();
    delay (2000);
    //Timer.reset();                        // Reset a second timer
  }



  //-----------------------------------------------------------  Logic to Reset RainMaker

  // Read GPIO0 (external button to reset device
  if (digitalRead(gpio_reset) == LOW) { //Push button pressed
    Serial.printf("Reset Button Pressed!\n");
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000) {
      // If key pressed for more than 10secs, reset all
      Serial.printf("Reset to factory.\n");
      wifi_connected = 0;
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = 0;
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
  delay(100);

  if (WiFi.status() != WL_CONNECTED){
    //Serial.println("WiFi Not Connected");
    digitalWrite(WIFI_LED, LOW);
  }
  else{
    //Serial.println("WiFi Connected");
    digitalWrite(WIFI_LED, HIGH);
  }
  detectMotion();
  controlBuzzer();
}
 void readSensor() {

  ldrVal = map(analogRead(LDR_PIN), 400, 4200, 0, 100);
  Serial.print("LDR - "); Serial.println(ldrVal);
 }


  
void detectMotion() {
  if(SECURITY_STATE == true) {
    PIR_STATE_OLD = PIR_STATE_NEW; // store old state
    PIR_STATE_NEW = digitalRead(PIR_PIN); //read new state
    //------------------------------------------------------------------------
    if(PIR_STATE_OLD == HIGH && PIR_STATE_NEW == LOW) {
      Serial.println("Motion detected!");
      esp_rmaker_raise_alert("Security Alert!\nFIRE IS DETECTED.");
      digitalWrite(BUZZER_PIN, HIGH);
      BUZZER_STATE = true;
      buzzer_timer = millis();
    }
    //------------------------------------------------------------------------
  }
}

void controlBuzzer(){
  if (BUZZER_STATE == true) {
    if (millis() - buzzer_timer > 5000) {
      digitalWrite(BUZZER_PIN, LOW);
      BUZZER_STATE = false;
      buzzer_timer = 0;
    }
  }
}
void Send_Sensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  readSensor();

  Serial.print("Temperature - "); Serial.println(t);
  Serial.print("Humidity - "); Serial.println(h);

  temperature.updateAndReportParam("Temperature", t);
  humidity.updateAndReportParam("Temperature", h);
  ldr.updateAndReportParam("Temperature", ldrVal);
}
