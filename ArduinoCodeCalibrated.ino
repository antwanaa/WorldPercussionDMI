// #include "Adafruit_AHRS_Mahony.h"
#include "Adafruit_AHRS_Madgwick.h"
// #include <MadgwickAHRS.h>

#include <Arduino_LSM9DS1.h>

#include "mbed.h"
mbed::Timer t;        // Timer for IMU polling frequency
mbed::Timer tPrint;   // Timer for gyroscope values printing
mbed::Timer tButton;  // Timer for button debouncing
mbed::Timer tFilter;  // Timer for Hit event filtering

bool DEBUG_ = false;

const float HIT_THRESHOLD = 2.0f;       // Minimum acceleration to trigger a hit
const long SHORT_WAIT_DURATION = 10000; // Duration to wait between each printing line (in debug)
long WAIT_VALUE = SHORT_WAIT_DURATION;  // Initialize the wait value to the short wait duration
long LONG_WAIT_DURATION = 1500000;      // Duration to wait between hit detection and printing another line (debug)
const float DRUM_HALF_WIDTH = 20.0f;    // Drum (detection zone) half width in degrees

const float SAMPLE_FREQ = 20.0f;                   // IMU Sampling frequency
const long SAMPLE_PERIOD = 1000000 / SAMPLE_FREQ;  // IMU Sample period in microseconds

const long BUTTON_DEBOUNCE = 300000;  // Button input ignored during this duration following a press event

// Zone Mapping
//  3   1   5
//  4   2   6

const int TOP_LEFT = 3;
const int TOP_CENTER = 1;
const int TOP_RIGHT = 5;
const int BOTTOM_LEFT = 4;
const int BOTTOM_CENTER = 2;
const int BOTTOM_RIGHT = 6;

int INSTRUMENT = 1;             // Instrument ID
const int NUM_INSTRUMENTS = 4;  // Number of instruments

const int buttonPin = 13;  // the number of the pushbutton pin
bool buttonState = false;

// Note: This sketch is a WORK IN PROGRESS

// // Offsets applied to raw x/y/z mag values
// float mag_offsets[3] = { 2.45F, -4.55F, -26.93F };

// // Soft iron error compensation matrix
// float mag_softiron_matrix[3][3] = { { 0.961, -0.001, 0.025 },
//                                     { 0.001, 0.886, 0.015 },
//                                     { 0.025, 0.015, 1.176 } };

// float mag_field_strength = 44.12F;

// // Offsets applied to raw x/y/z mag values
// float mag_offsets[3] = { 36.086073f, 10.799801f, 7.783771f };

// // Soft iron error compensation matrix YES
// float mag_softiron_matrix[3][3] = { { 1.253387f, 0.040156f, 0.020214f },
//                                     { 0.040156f, 1.308894f, -0.007654f },
//                                     { 0.020214f, -0.007654f, 1.199946f } };
// // Soft iron error compensation matrix temp
// float mag_softiron_matrix[3][3] = { { 0.798846f, -0.024588f, -0.013614f },
//                                     { -0.024588f, 0.764789f, 0.005292f },
//                                     { -0.013614f, 0.005292f, 0.833634f } };

// Offsets applied to raw x/y/z mag values NO BREADBOARD
float mag_offsets[3] = { 15.025947f, 20.525517f, 7.783771f };

// Soft iron error compensation matrix YES
float mag_softiron_matrix[3][3] = { { 1.227704f, 0.039757f, 0.016049f },
                                    { 0.039757f, 1.224314f, -0.009543f },
                                    { 0.016049f, -0.009543f, 1.293139f } };

float mag_field_strength = 53.2633f;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
// float rawToDPS = 0.00875F;
// float dpsToRad = 0.017453293F;
// float gyro_zero_offsets[3] = { 175.0F * rawToDPS * dpsToRad,
//                                -729.0F * rawToDPS* dpsToRad,
//                                101.0F * rawToDPS* dpsToRad };
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3] = { 2.43f,
                               1.46f,
                              //  -1.17f };
                              -1.20f };


// Mahony is lighter weight as a filter and should be used on slower systems
// Adafruit_Mahony filter;
Adafruit_Madgwick filter;
// Madgwick filter;

void setup() {
  // Serial.begin(115200);
  Serial.begin(9600);

  pinMode(buttonPin, INPUT);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while (!Serial);

  Serial.println("Fusion Example");
  Serial.println("");

  // Initialize the sensors.
  if (!IMU.begin()) {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while (1)
      ;
  }

  // Run filter at 20Hz (Magnetometer runs at 19Hz)
  filter.begin(SAMPLE_FREQ);

  // Wait for the connection to complete
  delay(1000);

  //Start the various timers used in the program
  t.start();
  tPrint.start();
  tButton.start();
  tFilter.start();
}

void loop(void) {
  buttonState = digitalRead(buttonPin);
  if ((buttonState == HIGH) && (tButton.elapsed_time().count() > BUTTON_DEBOUNCE)) {
    if (DEBUG_) Serial.println("HIGH");
    tButton.reset();
    INSTRUMENT += 1;
    if (INSTRUMENT > NUM_INSTRUMENTS) {
      INSTRUMENT = 1;
    }
  } else {
    // if(DEBUG_) Serial.println("LOW");
  }

  float GyrX, GyrY, GyrZ, AccelX, AccelY, AccelZ, magx, magy, magz;

  // Get new data samples
  if (t.elapsed_time().count() >= SAMPLE_PERIOD) {
    t.stop();
    t.reset();
    t.start();
    if (IMU.magneticFieldAvailable()) {
      if (IMU.accelerationAvailable()) {
        if (IMU.gyroscopeAvailable()) {
          IMU.readMagneticField(magx, magy, magz);
          IMU.readAcceleration(AccelX, AccelY, AccelZ);
          IMU.readGyroscope(GyrX, GyrY, GyrZ);


          // Apply mag offset compensation (base values in uTesla)
          float x = magx - mag_offsets[0];
          float y = magy - mag_offsets[1];
          float z = magz - mag_offsets[2];

          // Apply mag soft iron error compensation
          float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
          float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
          float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
          my = -my;

          // Apply gyro zero-rate error compensation
          float gx = (GyrY - gyro_zero_offsets[1]);
          float gy = (GyrX - gyro_zero_offsets[0]);
          float gz = GyrZ - gyro_zero_offsets[2];

          // Apply compensation for the accelerometer
          float ax = getCorrectedValue(AccelY, 'y');
          float ay = getCorrectedValue(AccelX, 'x');
          float az = getCorrectedValue(AccelZ, 'z');

          filter.updateIMU(gx, gy, gz,
                           ax, ay, az);


          // delay(96);
          detectHit(ax, ay, az);
        }
      }
    }
  }
  
  if (DEBUG_ && tPrint.elapsed_time().count() >= WAIT_VALUE) {
    tPrint.stop();
    tPrint.reset();
    WAIT_VALUE = SHORT_WAIT_DURATION;
    tPrint.start();
    // Print the orientation filter output
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
    // Serial.print(millis());
    // Serial.print(" - Orientation: ");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(heading);
    Serial.print("\t");
    Serial.print(roll);
    // Serial.print("\t\t");
    Serial.println();
  }
}

void detectHit(float ax, float ay, float az) {
  float accelerationNorm;
  accelerationNorm = sqrtf((powf(ax, 2.0f) + powf(ay, 2.0f) + powf(az, 2.0f)));
  if (accelerationNorm >= HIT_THRESHOLD && tFilter.elapsed_time().count() >= 200000) {
    tFilter.reset();
    if (DEBUG_) Serial.println("=============== HIT DETECTED ===================");
    // Print the orientation filter output
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();

    if ((roll <= 60.0f) && (roll > 0.0f)) {
      if (DEBUG_) Serial.print("=== HIGH and ");
      if ((heading <= 180.0f + 3 * DRUM_HALF_WIDTH) && (heading > 180.0f + DRUM_HALF_WIDTH)) {
        if (DEBUG_) Serial.println("LEFT ===");
        Serial.print(INSTRUMENT);
        Serial.print(" ");
        Serial.println(TOP_LEFT);
      } else if ((heading <= 180.0f + DRUM_HALF_WIDTH) && (heading > 180.0f - DRUM_HALF_WIDTH)) {
        if (DEBUG_) Serial.println("CENTER ===");
        Serial.print(INSTRUMENT);
        Serial.print(" ");
        Serial.println(TOP_CENTER);
      } else if ((heading <= 180.0f - DRUM_HALF_WIDTH) && (heading > 180.0f - 3 * DRUM_HALF_WIDTH)) {
        if (DEBUG_) Serial.println("RIGHT ===");
        Serial.print(INSTRUMENT);
        Serial.print(" ");
        Serial.println(TOP_RIGHT);
      } else {
        if (DEBUG_) Serial.println("undefined region! ===");
      }
    } else if ((roll <= 0.0f) && (roll > -60.0f)) {
      if (DEBUG_) Serial.print("=== LOW and ");
      if ((heading <= 180.0f + 3 * DRUM_HALF_WIDTH) && (heading > 180.0f + DRUM_HALF_WIDTH)) {
        if (DEBUG_) Serial.println("LEFT ===");
        Serial.print(INSTRUMENT);
        Serial.print(" ");
        Serial.println(BOTTOM_LEFT);
      } else if ((heading <= 180.0f + DRUM_HALF_WIDTH) && (heading > 180.0f - DRUM_HALF_WIDTH)) {
        if (DEBUG_) Serial.println("CENTER ===");
        Serial.print(INSTRUMENT);
        Serial.print(" ");
        Serial.println(BOTTOM_CENTER);
      } else if ((heading <= 180.0f - DRUM_HALF_WIDTH) && (heading > 180.0f - 3 * DRUM_HALF_WIDTH)) {
        if (DEBUG_) Serial.println("RIGHT ===");
        Serial.print(INSTRUMENT);
        Serial.print(" ");
        Serial.println(BOTTOM_RIGHT);
      } else {
        if (DEBUG_) Serial.println("undefined region! ===");
      }
    } else {
      if (DEBUG_) Serial.print("undefined region! ");
    }


    WAIT_VALUE = LONG_WAIT_DURATION;
    tPrint.reset();
  }
}

// function to compute a corrected value for the accelerometer
float getCorrectedValue(float val, char axis) {
  float correctedValue = 0;
  if (axis == 'x') {
    correctedValue = (((val - (-1.01)) * 2.0) / 1.99) - 1.0;
  } else if (axis == 'y') {
    correctedValue = (((val - (-1.02)) * 2.0) / 1.99) - 1.0;
  } else if (axis == 'z') {
    correctedValue = (((val - (-1.02)) * 2.0) / 1.99) - 1.0;
  }
  return correctedValue;
}
