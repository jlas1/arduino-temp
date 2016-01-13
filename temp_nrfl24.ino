/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <MySigningAtsha204Soft.h>
#include <MySigningAtsha204.h>

#define MY_DEBUG
//#define MY_SIGNING_FEATURE
#define MY_VERIFICATION_TIMEOUT_MS 5000
//#define MY_SECURE_NODE_WHITELISTING
#define MY_SIGNING_REQUEST_SIGNATURES
//#define MY_ATSHA204_PIN 17 // A3 - pin where ATSHA204 is attached
#define MY_RANDOMSEED_PIN 7 // A7 - Pin used for random generation (do not connect anything to this)
#define MY_HMAC_KEY 0x22,0x75,0x1f,0x88,0x58,0xab,0x56,0xca,0x61,0x06,0x9d,0x7b,0x23,0xcc,0xe1,0xb0,0x28,0x67,0x71,0xd9,0x59,0xda,0xc6,0x95,0x51,0x3d,0xd9,0xbb,0x74,0x7f,0xee,0xeb
uint8_t soft_serial[SHA204_SERIAL_SZ] = {0x11,0x26,0xa5,0x1f,0x7b,0x0d,0x1e,0xff,0xa0};
//Signature
MySigningAtsha204Soft signer(true);  // Select SW ATSHA signing backend

// NRFRF24L01 radio driver (set low transmit power by default) 
MyTransportNRF24 transport(RF24_CE_PIN, RF24_CS_PIN, RF24_PA_LEVEL);
//MyTransportRFM69 transport;

// Hardware profile 
MyHwATMega328 hw;

#define DEBUG
#define TEMP_ID 0
#define BVOLT_ID 1
#define LED_POWERUP_COUNT 6
#define LED_DELAY 100

//DS18B20 configuration
#define ONE_WIRE_BUS 4 // Pin where dallas sensor is connected 
#define ONE_WIRE_VCC 5 // Pin where the one wire power is controlled


//for 2xAA nimh low discharge
#define VMIN 2.2
#define VMAX 2.8
#define VDELTA 0.6

//MySensor gw (transport, hw , signer);
MySensor gw (transport, hw);

float Vbatt,lastTemperature=-127,temperature=-127,deltatemp;
unsigned int BattValue,Batt,nosend=1000;
unsigned long present=5760;
boolean ack = false;
boolean receivedConfig = false;
boolean metric; 

unsigned long tempwakeupTime = 1000;
unsigned long sensorreadTime = 8000;
unsigned long SLEEP_TIME = (30000 - tempwakeupTime - sensorreadTime*2); // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

//Mysensors Messages initialization
MyMessage msgT(TEMP_ID,V_TEMP);
MyMessage msgB(BVOLT_ID,V_VOLTAGE);

void setup()  
{  
  Serial.begin(19200);
  Serial.println("begin");
  pinMode(2,INPUT);
  // disable internal pull-up
  digitalWrite(2,LOW);
  pinMode(3,INPUT);
  digitalWrite (3, LOW);  
  pinMode(6,INPUT);
  digitalWrite (6, LOW); 
  pinMode(7,INPUT);
  digitalWrite (7, LOW); 
  pinMode(8,INPUT);
  digitalWrite (8, LOW); 
  pinMode(ONE_WIRE_VCC,OUTPUT);
  digitalWrite (ONE_WIRE_VCC, HIGH);

  //blink LED on power up
  pinMode(13,OUTPUT);
  for (int i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (13, HIGH);
    delay(LED_DELAY);
    digitalWrite (13, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }
  
  // Startup up the OneWire library
  sensors.begin();
  sensors.setWaitForConversion(false);
  
  analogReference(DEFAULT);
  //Accept messages, auto id, no relay
  gw.begin(incomingMessage, 249, false);
}

void loop() 
{
  Serial.print(".");
  //Check for messages
  gw.process(); 

  //present at beggining and every day
  if (present > 2880) {
    gwPresent ();
    present = 1;
  }

  // power up sensor
  Serial.println("waking sensor");
  digitalWrite (ONE_WIRE_VCC, HIGH);
  gw.sleep(tempwakeupTime);

  // Fetch temperatures from Dallas sensors  
  sensors.requestTemperatures();
  gw.sleep(sensorreadTime);
  // Fetch
  temperature = ((metric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)));
  //Fetch again
  sensors.requestTemperatures();
  gw.sleep(sensorreadTime);
  temperature = (temperature + (metric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)))/2.0;
  // disable sensor
  Serial.println("sleeping sensor");
  digitalWrite (ONE_WIRE_VCC, LOW);
  //print Temperature
  Serial.print("Metric:");
  Serial.print(metric);
  Serial.print(" temperature: ");
  Serial.println(temperature);
  
  // Only send data if no error
  if (temperature != -127.00 && temperature != 85.00) {
    deltatemp = temperature - lastTemperature;
    if (abs(deltatemp) < 0.2) {
      nosend ++;
      //debug message
      Serial.println("Aproximatelly the same Temperature, skipping send");
    } else {
      lastTemperature = temperature;
      nosend = 1000;
    }
  } else {
    Serial.println("Error reading temperature");
  }

  //only sends values if they have changed or if it didn't send for 120 cycles (1 hour)
  if (nosend > 119) {
    sendValues();  
    nosend = 0;  
  }

  present++;
  gw.sleep(SLEEP_TIME);
} 

void readBattery () {
  //Battery meter
  BattValue = analogRead(A3);
  gw.wait(10);
  BattValue = analogRead(A3);
  gw.wait(10);
  Serial.print("analog 1 ");
  Serial.print(BattValue);
  BattValue = (BattValue + analogRead(A3))/2;
  Serial.print(", 2 ");
  Serial.println(BattValue);
  Vbatt= (float)BattValue * 0.003363075;
  Batt= (Vbatt-VMIN)*100.0/(VDELTA);
  if (Batt > 100) {
    Batt = 100;
  } else if (Batt < 0) {
    Batt = 0;
  }  
  //print battery status
  Serial.print("battery: ");
  Serial.print(Batt);
  Serial.print(", ");
  Serial.println(Vbatt);
}

void sendValues () {
  //print debug message
  Serial.println("Sending Values");
  //send values
  readBattery();
  gw.wait(100);
  gw.sendBatteryLevel(Batt, ack);
  gw.wait(100);
  gw.send(msgT.set(temperature,2));
  gw.wait(100);
  gw.send(msgB.set(Vbatt,2));
  gw.wait(100);
}

void gwPresent () {
  gw.wait(100);
  gw.sendSketchInfo("Temperature Sensor", "1.2");  
  gw.wait(100);
  gw.present(TEMP_ID, S_TEMP, "Temperature");  
  gw.wait(100);
  gw.present(BVOLT_ID, S_MULTIMETER, "Battery Voltage");  
  gw.wait(100);
  metric = gw.getConfig().isMetric;
  gw.wait(100);
}

/*
int repeat = 0;
boolean sendOK = false;
int repeatdelay = 0;
 
void loop()
{
  resend((msg.set(OPEN)), 5);
delay(500);

}*/

void resend(MyMessage &msg, int repeats)
{
  int repeat = 1;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    if (gw.send(msg)) {
      sendOK = true;
    } else {
      sendOK = false;
      Serial.print("FEHLER ");
      Serial.println(repeat);
      repeatdelay += 250;
    } repeat++; delay(repeatdelay);
  }
}

void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
 
 
  if (message.isAck()) {
     Serial.println("This is an ack from gateway");
  } else {
     Serial.print("Incoming change for sensor:");
     Serial.print(message.sensor);
     Serial.print(".\n");
  }
}

