#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define TEMP_PIN 18
#define TDS_PIN 34
#define TURB_PIN 35
#define OXGN_PIN 32
#define WATER_FLOW 27
#define LCD_ADDR 0x27

#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
#define SUB_TEMP "ras/temperature"
#define SUB_TDS "ras/tds"
#define SUB_TURB "ras/turbidity"
#define SUB_OXGN "ras/oxygen"
#define SUB_FLOW "ras/flow"

OneWire oneWire(TEMP_PIN);
DallasTemperature DS18B20(&oneWire);

LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

float turbidityValue = 0;

int minDOValue = 0;
int maxDOValue = 1023;  // Adjust this based on your DO sensor's specifications
float minDOConcentration = 0.0;  // Adjust this based on your DO sensor's specifications
float maxDOConcentration = 5.9, doValue = 0;

const char* ssid = "Buttered Crayfish";
const char* password = "!T06MYTA9G2G";
const char* mqtt_server = "test.mosquitto.org";

char str_temp[10];
char str_tds[10];
char str_turb[10];
char str_oxgn[10];
char str_flow[10];

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(SUB_TEMP);
      client.subscribe(SUB_TDS);
      client.subscribe(SUB_TURB);
      client.subscribe(SUB_OXGN);
      client.subscribe(SUB_FLOW);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

void setup() {
  pinMode(TDS_PIN, INPUT);
  pinMode(OXGN_PIN, INPUT);
  pinMode(WATER_FLOW, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(WATER_FLOW), pulseCounter, FALLING);

  Wire.begin();
  DS18B20.begin();
  lcd.begin(20, 4);
  lcd.backlight();

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    
    pulse1Sec = pulseCount;
    pulseCount = 0;

    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    flowMilliLitres = (flowRate / 60) * 1000;

    totalMilliLitres += flowMilliLitres;

    dtostrf(flowRate, 4, 2, str_flow);
    Serial.print("Flow rate: ");
    Serial.print(flowRate,0);
    Serial.println("L/min");
    client.publish(SUB_FLOW, str_flow);
  }

  if (now - lastMsg > 2000) {

    lastMsg = now;

    DS18B20.requestTemperatures();
    float tempC = DS18B20.getTempCByIndex(0);

    static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      // for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      //   analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      // averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      // float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      // float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      // tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value

      tdsValue = readTDS();

      int rawDOValue = analogRead(OXGN_PIN);
      doValue = map(rawDOValue, minDOValue, maxDOValue, minDOConcentration, maxDOConcentration);

      int rawTurbValue = analogRead(TURB_PIN);

      dtostrf(tempC, 4, 2, str_temp);
      Serial.print("Temperature:");
      Serial.print(tempC,0);
      Serial.println(" C");
      client.publish(SUB_TEMP, str_temp);

      dtostrf(tdsValue, 4, 2, str_tds);
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      client.publish(SUB_TDS, str_tds);

      dtostrf(rawTurbValue, 4, 2, str_turb);
      Serial.print("Turbidity:");
      Serial.print(rawTurbValue,0);
      Serial.println(" NTU");
      client.publish(SUB_TURB, str_turb);

      dtostrf(doValue, 4, 2, str_oxgn);
      Serial.print("DO:");
      Serial.print(doValue,0);
      Serial.println(" mg/L");
      client.publish(SUB_OXGN, str_oxgn);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TEMP:");
      lcd.print(tempC);
      lcd.print(" ");
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("TDS:");
      lcd.print(tdsValue, 0);
      lcd.print(" ppm");

      lcd.setCursor(0, 2);
      lcd.print("TURB: ");
      lcd.print(rawTurbValue);
      lcd.print("NTU");

      lcd.setCursor(0, 3);
      lcd.print("OXGN: ");
      lcd.print(doValue, 2);
      lcd.print("mg/L");
   }
  }
}

float readTDS() {
  // Replace this with your actual TDS sensor reading code
  // For example, use analogRead to read the sensor value and convert to TDS using calibration formula
  int sensorValue = analogRead(TDS_PIN);

  // Replace the following line with your TDS calibration formula
  float tds = map(sensorValue, 0, 1382, 0, 1000);

  return tds; // Return the calculated TDS value
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
