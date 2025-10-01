// first version of fall detector using LSTM-based model for inference
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "model.h" // generated TFLite model header
#include <tflm_esp32.h>
#include "eloquent_tinyml.h"
//#include "EloquentTinyML.h"
//#include "eloquent_tinyml/tensorflow.h"

// global vars
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

// Model parameters
constexpr int inputNumber = 1200; // windowsize * 4 (3 accel + 1 timestamp)
constexpr int outputNumber = 1; // fall vs not-fall
constexpr int tensorArenaSize = 32 * 1024; // an area of working memory for the model
constexpr int TF_NUM_OPS = 16; // not too sure what this param means


Eloquent::TF::Sequential<TF_NUM_OPS, tensorArenaSize> tf;

// TFLite globals
/*namespace{
const tflite::Model* fallModel = nullptr; // pointer to the model
tflite::ErrorReporter* tflErrorReporter =  nullptr; // pointer to the error reporter
constexpr int tensorArenaSize = 102 * 1024; // an area of working memory for the model
uint8_t tensorArena[tensorArenaSize]; 
TfLiteTensor* input = nullptr; // pointer to input tensor
TfLiteTensor* output = nullptr; // pointer to output tensor
tflite::MicroInterpreter* interpreter = nullptr;  // pointer to interpreter
} // namespace

*/

// Fall detection state
bool fallDetected = false;
unsigned long fallTimestamp = 0; // TODO: register timestamps in circular buffer
const float fallThreshold = 0.7; // confidence threshold for considering a fall

// feature extraction helpers: TODO: make sure to reproduce the normalization process from training
float calculateMean(float* values, int size);
float calculateStd(float* values, int size, float mean);
float calculateMax(float* values, int size);

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

 

  // Initialize Error reporter
  /*static tflite::MicroErrorReporter micro_error_reporter; // define a micro_Error_reporter instance
  tflErrorReporter = &micro_error_reporter; // create an errorReporter ptr and assign it to micro_error_reporter

  // Load TFLite model
  fallModel = tflite::GetModel(model);
  if (fallModel->version() != TFLITE_SCHEMA_VERSION) 
  {
      tflErrorReporter->Report(
      "Model provided is schema version %d not equal "
      "to supported version %d.",
      fallModel->version(), TFLITE_SCHEMA_VERSION);
      return;
  }*/


}



void loop() {
  // put your main code here, to run repeatedly:

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
