// This #include statement was automatically added by the Particle IDE.
#include "MQTT/MQTT.h"
#include <application.h>
#include <spark_wiring_i2c.h>

// SI1132 I2C address is 0x60(96)
#define Addr 0x60

double visible = 0.0, ir = 0.0, uv = 0.0;
int response = 0;

// Configure the incoming signal pin for the moisture sensor.
int sensorPin = A0;

// Configure the periodic power output pin
int powerOut = D0;  

// Configure the Diagnostic LED
int diagnosticLed = D7;

// Unique identifier for the device
String deviceId="/garden/aisle/1/";

// Unique identifier for individual sensors
String moistureSensor = deviceId + "moisture";
String visibleLightSensor = deviceId + "visible-light";
String infraRedSensor = deviceId + "infra-red";
String ultraVioletSensor = deviceId + "ultra-violet";


// Configure the IP address of the MQTT Client 
byte server[] = { 192,168,117,86 };

// Iniialize the MQTT Client for port 1833
MQTT client(server, 1883, NULL);

// This gets executed  before the loop defined below. 
void setup() {

  //Set the pin mode to output
  pinMode(diagnosticLed, OUTPUT);

  // Configure the input pin for sensor input signals
  pinMode(sensorPin, INPUT);

  // Configure the output pin for sending power to the sensor.
  pinMode(powerOut, OUTPUT); 

  // Configure the serial interface for debugging.
  Serial.begin(9600);

  Serial.println("Connecting to MQTT server ");

  // Connect the the spark MQTT service.
  client.connect("sparkclient");
  
  Serial.println("Setting up light sensor");

  setupLightSensor();

}


// This is the loop that gets repeatedly called after the setup code has completed. 
void loop() {
    
  blinkDiagnostic(1, 100);    
    
  // Turn on the power to the sebnsor by seting the output pin to high. 
  digitalWrite(powerOut, HIGH);    
    
  // Read a value from the signal line
  int sensorValue = analogRead(sensorPin);

  // Send the moisture reading for this device to the particle cloud service
  bool result = Particle.publish(moistureSensor, String(sensorValue));    
  Serial.println("Publish of Sensor value " + String(sensorValue) + " to the Particle Cloud returned " + String(result));


  // Check if the MQTT client is connected  
  if (! client.isConnected()) {
      // Display three diagnostic led pulses to notify that we are unable to connect to the MQTT server.
      blinkDiagnostic(3, 100);

      // Attempt or Re-attempt a connect the the spark MQTT service.
      client.connect("sparkclient");
  }

  // Send the moisture reading for this device to the MQTT service
  bool mqttResult = client.publish(moistureSensor, String(sensorValue));
  Serial.println("Publish of Sensor value " + String(sensorValue) + " to the MQTT server returned " + String(mqttResult));


  // Publish the IR and UV readings
  publishLightSensorReadings();


  // Turn the power on the sensor back off to prevent the sensor from degrading
  // and to conserve battery life. 
  digitalWrite(powerOut, LOW);    

  // Pause for 60 seconds until the next reading
  Serial.println("Sleeping");
  System.sleep(SLEEP_MODE_DEEP, 180);
  //System.sleep(240000);
  
  //delay(60000);
  
  Serial.println("Awake");


}

//The function that handles the event from IFTTT
void blinkDiagnostic(int numberOfBlinks, int duration){
  int cnt = 1;

  while(cnt <= numberOfBlinks) {
    // We'll turn the LED on
    digitalWrite(diagnosticLed, HIGH);
  
    Serial.println("=== Turn On");

    delay(duration);
    
    // Then we'll turn it off...
    digitalWrite(diagnosticLed, LOW);

    Serial.println("=== Turn Off");

    delay(duration);

    cnt += 1;
    
    Serial.println("=== Count " + String(cnt));
  }

}

void setupLightSensor() {
  Serial.println("Setting up light sensor");    

// Particle.variable("i2cdevice", "SI1132");
//   Particle.variable("visible", visible);
//   Particle.variable("uv", uv);
//   Particle.variable("ir", ir);
  
  // Initialise I2C communication as MASTER
  Wire.begin();
  // Initialise Serial Communication, Baud rate = 9600
//  Serial.begin(9600);

  // Enable UVindex measurement coefficients
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-1 register
  Wire.write(0x13);
  // Default value
  Wire.write(0x29);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-2 register
  Wire.write(0x14);
  // Default value
  Wire.write(0x89);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-3 register
  Wire.write(0x15);
  // Default value
  Wire.write(0x02);
  // Stop I2C Transmission
  Wire.endTransmission();



  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COFF-4 register
  Wire.write(0x16);
  // Default value
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();


  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Enable uv, Visible, IR
  Wire.write(0xF0);
  // Stop I2C Transmission
  Wire.endTransmission();


  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select command register
  Wire.write(0x18);
  // Select CHLIST register in RAM
  Wire.write(0x01 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select HW_KEY register
  Wire.write(0x07);
  // Default value
  Wire.write(0x17);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Small IR photodiode
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADCMUX register in RAM
  Wire.write(0x0E | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set ADC Clock divided / 1
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADC_GAIN register in RAM
  Wire.write(0x1E | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set 511 ADC Clock
  Wire.write(0x70);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  /// Select ALS_IR_ADC_COUNTER register in RAM
  Wire.write(0x1D | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set ADC Clock divided / 1
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
 // Select ALS_VIS_ADC_GAIN register in RAM
  Wire.write(0x11 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // High Signal Range
  Wire.write(0x20);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_IR_ADC_MISC register in RAM
  Wire.write(0x1F | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // Set 511 ADC Clock
  Wire.write(0x70);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_VIS_ADC_COUNTER register in RAM
  Wire.write(0x10 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_WR register
  Wire.write(0x17);
  // High Signal Range
  Wire.write(0x20);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Select ALS_VIS_ADC_MISC register in RAM
  Wire.write(0x12 | 0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(10);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select PARAM_RD register
  Wire.write(0x2E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);
  // Read 1 byte of data
  response = Wire.read();
  delay(300);

}


void publishLightSensorReadings() {
  unsigned int data[4];
  
  Serial.println("Publishing Light Sensor Readings");
  
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select COMMAND register
  Wire.write(0x18);
  // Start ALS conversion
  Wire.write(0x0E);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x22);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  // Request 4 byte of data
  Wire.requestFrom(Addr, 4);

  // Read 4 bytes of data
  // visible lsb, visible msb, ir lsb, ir msb
  if (Wire.available() == 4)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
  }

  visible = (data[1] * 256.0 + data[0]);
  ir = (data[3] * 256.0 + data[2]);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x2C);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);

  // Read 2 bytes of data
  // uv lsb, uv msb
  if (Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  
  // Convert the data
  uv = (data[1] * 256.0 + data[0]);

  //Pulbish the visible light values to the cloud and MQTT
  Particle.publish(visibleLightSensor, String(visible));
  client.publish(visibleLightSensor, String(visible));
  delay(1000);

  //Pulbish the infra-red values to the cloud and MQTT
  Particle.publish(infraRedSensor, String(ir));
  client.publish(infraRedSensor, String(ir));
  delay(1000);

  Particle.publish(ultraVioletSensor, String(uv));
  client.publish(ultraVioletSensor, String(uv));

  delay(1000);
}



