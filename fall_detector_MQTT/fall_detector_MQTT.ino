// second version of fall detector, this using MQTT and remote inference
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// WiFi Config 
const char* WIFI_SSID = "Silvas";
const char* WIFI_PASS = "ladyboy_edi_2025"; // remember to delete this embarrassing pwd when sending to teacher

// Example HiveMQ Cloud endpoint (cluster URL)
const char* MQTT_HOST = "maqiatto.com";
const uint16_t MQTT_PORT = 1883; // tcp port
const char* MQTT_USER = "netosilvan78@gmail.com";
const char* MQTT_PASSW = "y4NW1jgJi3b7";
const char* MQTT_TOPIC = "netosilvan78@gmail.com/imu/esp32/readings";

// ====== Globals ======
WiFiClient net;
PubSubClient mqtt(net);
//---------------

Adafruit_MPU6050 mpu;
const int sampleRate = 50; 
/*
  Sample Rate:
  How often the IMU measures motion per second, in Hz. Higher Hz captures faster motion but costs more power,
  storage, and processing. Human activity and gait can be usually captured on a range 50-200 Hz
*/ 

const int windowSize = 300; 
/* 
   Window Size:
   For how long will the IMU continuously capture signals before processing them. We've chosen
   4 seconds at 50Hz, this specific window size is needed due to the shape of training data the
   model is used to: a floating point 2-dimensional matrix input[300][4], in which the the 4-sized 
   dimension represents pitch | roll | yaw | timestep
*/ 

float accelBuffer[windowSize][4]; // x, y, z, timestamp
float gyroBuffer[windowSize][4]; // x, y, z, timestamp --> we wont need this on this version
int bufferIndex = 0;
bool bufferFilled = false;
/*
  Circular Buffers for storing sensor readings:
  A good data structure for streaming data, constant-time push/pop and bounded memory.
  This structure is a fixed-size array that treats the end as connected to the beginning.
  You write at a head index and read from a tail index. When full, you can either overwrite
  or reject writes until dropping the whole data.
*/


// Fall detection state
bool fallDetected = false;
unsigned long fallTimestamp = 0; 
const float fallThreshold = 0.7; // confidence threshold for considering a fall

// feature extraction helpers: TODO: make sure to reproduce the normalization process from training
float calculateMean(float* values, int size);
float calculateStd(float* values, int size, float mean);
float calculateMax(float* values, int size);

void ensureWifi();
void ensureMqtt();
void callback(char* topic, byte* payload, unsigned int length);
void publishWindow();


float ax, ay, az, baseAx, baseAy, baseAz;
void calibrateSensor(); 
/*
  calibrateSensor():
  We'll use this function to calibrate our IMU, this is done by taking 10 different readings
   and averaging them. This average will be used to calculate our calibration constants (baseAx, 
   baseAy and baseAz), which will modify every subsequent reading. 
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // put here other peripherals
  Wire.begin();

  ensureWifi();

  // TLS setup
	//net.setCACert(CA_CERT); // verify server cert
	// If your broker requires client certs, also setCertificate and setPrivateKey.

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(callback);
  mqtt.setBufferSize(18192);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // TODO: add some led here to show that we have initalized
    while (1) {
      delay(10);
    }
  }

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // ±500 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // 21 Hz bandwidth
  calibrateSensor();
  Serial.println("");

  ensureMqtt();

}

void loop() {
  // put your main code here, to run repeatedly:
  ensureWifi();
  if(!mqtt.connected()) ensureMqtt();
  mqtt.loop();

  static unsigned long lastSampleTime = 0;
  unsigned long currentTime = millis();

  // sample at our target range
  if(currentTime - lastSampleTime >= (1000 / sampleRate)) {
    lastSampleTime = currentTime;

    // get new sensor readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Store in circular buffer
    accelBuffer[bufferIndex][0] = a.acceleration.x;
    accelBuffer[bufferIndex][1] = a.acceleration.y;
    accelBuffer[bufferIndex][2] = a.acceleration.z;
    accelBuffer[bufferIndex][3] = 0.0;  // TODO: make a running timer until the window closes
    gyroBuffer[bufferIndex][0] = g.gyro.x;
    gyroBuffer[bufferIndex][1] = g.gyro.y;
    gyroBuffer[bufferIndex][2] = g.gyro.z;
    gyroBuffer[bufferIndex][3] = 0.0;

    bufferIndex = (bufferIndex + 1) % windowSize; // will only be 0 when bufferIndex reaches 300
    if(bufferIndex == 0)
    {
      bufferFilled = true;
      // TODO: reset the running counter here
    }

    // Process window when buffer is full
    if (bufferFilled && bufferIndex % 10 == 0) { // Check every 0.2s (10 samples at 50Hz)
      Serial.println("Window filled");
      publishBatch();
      bufferFilled = false;
      //processSensorWindow();
    }

  }

}

float calculateMean(float* values, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += values[i];
  }
  return sum / size;
}

float calculateStd(float* values, int size, float mean) {
  float sumSquaredDiff = 0;
  for (int i = 0; i < size; i++) {
    float diff = values[i] - mean;
    sumSquaredDiff += diff * diff;
  }
  return sqrt(sumSquaredDiff / size);
}

float calculateMax(float* values, int size) {
  float max = values[0];
  for (int i = 1; i < size; i++) {
    if (values[i] > max) {
      max = values[i];
    }
  }
  return max;
}

void calibrateSensor()
{
  float totX, totY, totZ;
  sensors_event_t a, g, temp;
  
  for (int i = 0; i < 10; i++) {
    mpu.getEvent(&a, &g, &temp);
    totX = totX + a.acceleration.x;
    totY = totY + a.acceleration.y;
    totZ = totZ + a.acceleration.z;
  }
  baseAx = totX / 10;
  baseAy = totY / 10;
  baseAz = totZ / 10;

}

void ensureWifi() {
	WiFi.mode(WIFI_STA);
	if (WiFi.status() == WL_CONNECTED)return;
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to: ");
    Serial.print(WIFI_SSID);
    Serial.println(""); 
    delay(300); 
  }
}

void ensureMqtt() {
	while (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
		String cid = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Attempting to connect user ");
    Serial.print(cid);
    Serial.println(" to broker");
		if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASSW)){
      Serial.println("Connected to broker!");
      break;
    } 
    else {
      Serial.print("failed, rc=");
      Serial.println(mqtt.state());
		  delay(1000);
    }
    
	}
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}

void publishBatch() { // TODO: the AI suggestion of creating a struct for readings is better, we should implement it
  // Build JSON array
  //mqtt.publish(MQTT_TOPIC, "sanity publish", false);
  
  String payload = "";
  for (int i = 0; i < windowSize; i++) {
    //if (i > 0) payload += ",";
    payload += String(accelBuffer[i][0], 6);
    payload += ", ";
    payload += String(accelBuffer[i][1], 6);
    payload += ", ";
    payload += String(accelBuffer[i][2], 6);
    payload += ", ";
    payload += accelBuffer[i][3];
    payload += "\n";

    /*payload += "{\"t_ms\":";
    payload += accelBuffer[i][3];
    payload += ",\"ax\":";
    payload += String(accelBuffer[i][0], 6);
    payload += ",\"ay\":";
    payload += String(accelBuffer[i][1], 6);
    payload += ",\"az\":";
    payload += String(accelBuffer[i][2], 6);
    payload += ",\"gx\":";
    payload += String(gyroBuffer[i][0], 6);
    payload += ",\"gy\":";
    payload += String(gyroBuffer[i][1], 6);
    payload += ",\"gz\":";
    payload += String(gyroBuffer[i][2], 6);
    payload += "}"; */
  }
  //payload += "]";

  Serial.println(payload.c_str());
  mqtt.publish(MQTT_TOPIC, payload.c_str(), false);
  bufferIndex = 0; // Reset buffer
  Serial.println("Window Published");
}



