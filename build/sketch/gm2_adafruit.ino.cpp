#line 1 "C:\\Users\\scorp\\Documents\\GitHub\\GalaxyMouse-V2\\gm2_adafruit\\gm2_adafruit.ino"
#include <Arduino.h>
#include "libraries/Adafruit_MPU6050/Adafruit_MPU6050.h"
#include <Wire.h>
#include "libraries/Adafruit_Unified_Sensor/Adafruit_Sensor.h"
#include "libraries/Adafruit_HMC5883_Unified/Adafruit_HMC5883_U.h"
#include "config.h"
#include "SpaceMouseHID.h"
#include <float.h>
#include <EEPROM.h>
#include "libraries/Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    14

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 12

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

unsigned long lastTime = 0;
unsigned long lastTimeAcc = 0;
const unsigned long gyroDelay = 10;
const unsigned long accelerometerDelay = 10;

float gyroX = 0, gyroY = 0, gyroZ = 0;
float accX = 0, accY = 0, accZ = 0;
const float gyroXerror = 0.1, gyroYerror = 0.1, gyroZerror = 0.1;

float magX_offset = 0, magY_offset = 0, magZ_offset = 0;

unsigned long startTime = 0;
bool isTiming = false;

unsigned long autoCalibStartTime = 0;
bool autoCalibTiming = false;
float lastMagX = 0, lastMagY = 0, lastMagZ = 0;
float lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;
const unsigned long autoCalibDuration = 1000; // 1 seconds for magnetometer auto-calibration

bool isCalibrating = false;

unsigned long eepromCalibStartTime = 0;
bool eepromCalibTiming = false;

//float min_x = -100;
//float min_y = -90; 
//float min_z = -220; 
//float max_x = 120;
//float max_y = 130; 
//float max_z = 115; 

float min_x, max_x, min_y, max_y, min_z, max_z;

float min_yaw = -.13;
float min_pitch = -.13;
float min_roll = -.13;
float max_yaw = .13;
float max_pitch = .13;
float max_roll = .13;


const int sensorPin0 = A2; // Pin connected to resistor
const int sensorPin1 = A0; // Pin connected to resistor
const int sensorPin2 = A10; // Pin connected to resistor
const int sensorPin3 = A9; // Pin connected to resistor
const int sensorPin4 = A8; // Pin connected to resistor
const int sensorPin5 = A6; // Pin connected to resistor
const int sensorPin6 = A1; // Pin connected to resistor
//const int sensorPin7 = A7; // Pin connected to resistor

const int groundPin1 = A3; // Pin connected to GND


float lastGyroXValue = 0;
float lastGyroYValue = 0;
float lastGyroZValue = 0;
unsigned long lastGyroXTime = 0;
unsigned long lastGyroYTime = 0;
unsigned long lastGyroZTime = 0; // Add this line
const unsigned long resetDelay = 10; // .01 second


/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

uint8_t previousKeyState[10] = {0}; // Add this line to store previous key states

void setup(void) {

  Serial.begin(115200);
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(100);
  staticColor(strip.Color(255, 255, 255), 150); // Set initial color to white


  pinMode(groundPin1, OUTPUT);
  digitalWrite(groundPin1, LOW); // Set ground pin to GND


  /* Initialise the magnetometer */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 */
    Serial.println("Ooops, not HMC5883 detected ... Check your wiring!");
    while(1);
  }

  /* Adjust gain to handle stronger magnetic fields */
  mag.setMagGain(HMC5883_MAGGAIN_8_1); 
              /* HMC5883_MAGGAIN_1_3 (default)
                 HMC5883_MAGGAIN_1_9
                 HMC5883_MAGGAIN_2_5
                 HMC5883_MAGGAIN_4_0
                 HMC5883_MAGGAIN_4_7
                 HMC5883_MAGGAIN_5_6
                 HMC5883_MAGGAIN_8_1 */


  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  calibrateMagnetometer();


  readMinMaxValuesFromEEPROM();


}

uint8_t keyState[10]; 




void loop(void) 
{
  static unsigned long lastLedUpdate = 0;
  static bool toggleColor = false; // Add this line to toggle between red and white

  // Check if keyState[0] has changed
  if (keyState[0] != previousKeyState[0]) {
    if (keyState[0] == 0) {
      staticColor(strip.Color(255, 255, 255), 150); // White
    } else if (keyState[0] == 1) {
      staticColor(strip.Color(0, 255, 0), 150); // Green
    }
    previousKeyState[0] = keyState[0]; // Update previousKeyState
  }

  
  /* Get a new sensor event */ 
  sensors_event_t event; 

  mag.getEvent(&event);

  mpu.getEvent(&a, &g, &temp);

  float magX = event.magnetic.x - magX_offset;
  float magY = event.magnetic.y - magY_offset;
  float magZ = event.magnetic.z - magZ_offset;

  // Bound the magnetometer values by the min and max values
  if (magX < min_x) magX = min_x;
  if (magX > max_x) magX = max_x;
  if (magY < min_y) magY = min_y;
  if (magY > max_y) magY = max_y;
  if (magZ < min_z) magZ = min_z;
  if (magZ > max_z) magZ = max_z;

  if ((millis() - lastTime) > gyroDelay) {
    // Print Gyro Readings to the Serial Monitor
    getGyroReadings();
    lastTime = millis();
  }

  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Print Accelerometer Readings to the Serial Monitor
    getAccReadings();
    lastTimeAcc = millis();
  }


//-----------------------------------------------------gyro damping-----------------------------------------------------


  // Check if keyState[0] is 0 and set gyro values to 0
  if (keyState[0] == 0) {
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
  }

  // Apply damping factor to gyroX
  if (gyroX != 0) {
    float dampingFactorX = 0.005 * (1 - abs(gyroX) / 0.05); // Adjust the damping factor as needed
    gyroX -= gyroX * dampingFactorX;
    // Prevent gyroX from going over the maximum or minimum values
    if (gyroX > 0.13) gyroX = 0.13;
    if (gyroX < -0.13) gyroX = -0.13;
  }

  // Apply damping factor to gyroY
  if (gyroY != 0) {
    float dampingFactorY = 0.005 * (1 - abs(gyroY) / 0.05); // Adjust the damping factor as needed
    gyroY -= gyroY * dampingFactorY;
    // Prevent gyroY from going over the maximum or minimum values
    if (gyroY > 0.13) gyroY = 0.13;
    if (gyroY < -0.13) gyroY = -0.13;
  }

  // Apply damping factor to gyroZ
  if (gyroZ != 0) {
    float dampingFactorZ = 0.005 * (1 - abs(gyroZ) / 0.05); // Adjust the damping factor as needed
    gyroZ -= gyroZ * dampingFactorZ;
    // Prevent gyroZ from going over the maximum or minimum values
    if (gyroZ > 0.13) gyroZ = 0.13;
    if (gyroZ < -0.13) gyroZ = -0.13;
  }

  
//-----------------------------------------------------buttons-----------------------------------------------------


  int sensorValue0 = analogRead(sensorPin0); // Read voltage
  float voltage0 = sensorValue0 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage0 < 4.75) {
    keyState[0] = 1; // Set button status
  } else {
    keyState[0] = 0;
  }

  int sensorValue1 = analogRead(sensorPin1); // Read voltage
  float voltage1 = sensorValue1 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage1 < 2) {
    keyState[1] = 1; // Set button status
  } else {
    keyState[1] = 0;
  }

  int sensorValue2 = analogRead(sensorPin2); // Read voltage
  float voltage2 = sensorValue2 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage2 < 2) {
    keyState[2] = 1; // Set button status
  } else {
    keyState[2] = 0;
  }

  int sensorValue3 = analogRead(sensorPin3); // Read voltage
  float voltage3 = sensorValue3 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage3 < 2) {
    keyState[3] = 1; // Set button status
  } else {
    keyState[3] = 0;
  }

  int sensorValue4 = analogRead(sensorPin4); // Read voltage
  float voltage4 = sensorValue4 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage4 < 2) {
    keyState[4] = 1; // Set button status
  } else {
    keyState[4] = 0;
  }

  int sensorValue5 = analogRead(sensorPin5); // Read voltage
  float voltage5 = sensorValue5 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage5 < 2) {
    keyState[5] = 1; // Set button status
  } else {
    keyState[5] = 0;
  }

  int sensorValue6 = analogRead(sensorPin6); // Read voltage
  float voltage6 = sensorValue6 * (5.0 / 1023.0); // Convert to voltage (5V system)
  if (voltage6 < 2) {
    keyState[6] = 1; // Set button status
  } else {
    keyState[6] = 0;
  }


  //int sensorValue7 = analogRead(sensorPin7); // Read voltage
  //float voltage7 = sensorValue7 * (5.0 / 1023.0); // Convert to voltage (5V system)
  //if (voltage7 < 2) {
  //  keyState[7] = 1; // Set button status
  //} else {
  //  keyState[7] = 0;
  //}




//-----------------------------------------------------print values-----------------------------------------------------



  //Serial.print("    X: "); print_sign(magX);
  //Serial.print("    Y: "); print_sign(magY);
  //Serial.print("    Z: "); print_sign(magZ);
  //
  //Serial.print("GyroX: "); Serial.print(gyroX);
  //Serial.print("GyroY: "); Serial.print(gyroY);
  //Serial.print("GyroZ: "); Serial.print(gyroZ);



//-----------------------------------------------------mapping-----------------------------------------------------


  long map_x = map(magX, min_x, max_x, -350, 350);
  long map_y = map(magY, min_y, max_y, -350, 350);
  long map_z = map(magZ, min_z, max_z, -350, 350);
  long map_yaw = map(gyroZ * 10000, min_yaw * 10000, max_yaw * 10000, -350, 350);
  long map_pitch = map(gyroY * 10000, min_pitch * 10000, max_pitch * 10000, -350, 350);
  long map_roll = map(gyroX * 10000, min_roll * 10000, max_roll * 10000, -350, 350);


//------------------------------------------offset mapping for better neutral 0-------------------------------------

  if (abs(max_x) > abs(min_x)) {
    map_x += (700 / (abs(min_x) + abs(max_x))) * (abs(max_x) - ((abs(min_x) + abs(max_x)) / 2));
  } else {
    map_x -= (700 / (abs(min_x) + abs(max_x))) * (abs(min_x) - ((abs(min_x) + abs(max_x)) / 2));
  }

  if (abs(max_y) > abs(min_y)) {
    map_y += (700 / (abs(min_y) + abs(max_y))) * (abs(max_y) - ((abs(min_y) + abs(max_y)) / 2));
  } else {
    map_y -= (700 / (abs(min_y) + abs(max_y))) * (abs(min_y) - ((abs(min_y) + abs(max_y)) / 2));
  }

  if (abs(max_z) > abs(min_z)) {
    map_z += (700 / (abs(min_z) + abs(max_z))) * (abs(max_z) - ((abs(min_z) + abs(max_z)) / 2));
  } else {
    map_z -= (700 / (abs(min_z) + abs(max_z))) * (abs(min_z) - ((abs(min_z) + abs(max_z)) / 2));
  }

  if (abs(max_yaw) > abs(min_yaw)) {
    map_yaw += (700 / (abs(min_yaw) + abs(max_yaw))) * (abs(max_yaw) - ((abs(min_yaw) + abs(max_yaw)) / 2));
  } else {
    map_yaw -= (700 / (abs(min_yaw) + abs(max_yaw))) * (abs(min_yaw) - ((abs(min_yaw) + abs(max_yaw)) / 2));
  }

  if (abs(max_pitch) > abs(min_pitch)) {
    map_pitch += (700 / (abs(min_pitch) + abs(max_pitch))) * (abs(max_pitch) - ((abs(min_pitch) + abs(max_pitch)) / 2));
  } else {
    map_pitch -= (700 / (abs(min_pitch) + abs(max_pitch))) * (abs(min_pitch) - ((abs(min_pitch) + abs(max_pitch)) / 2));
  }

  if (abs(max_roll) > abs(min_roll)) {
    map_roll += (700 / (abs(min_roll) + abs(max_roll))) * (abs(max_roll) - ((abs(min_roll) + abs(max_roll)) / 2));
  } else {
    map_roll -= (700 / (abs(min_roll) + abs(max_roll))) * (abs(min_roll) - ((abs(min_roll) + abs(max_roll)) / 2));
  }


//-----------------------------------------------------print mapped values-----------------------------------------------------


  //Serial.print(" |   Map X: "); print_sign(map_x);
  //Serial.print(" |   Map Y: "); print_sign(map_y);
  //Serial.print(" |   Map Z: "); print_sign(map_z);
  //Serial.print(" |   Map Yaw: "); print_sign(map_yaw);
  //Serial.print(" |   Map Pitch: "); print_sign(map_pitch);
  //Serial.print(" |   Map Roll: "); print_sign(map_roll);


//-------------------------------------program does not work when inverting directions, do it here-----------------------------------------------------



  //map_x = -map_x;
  map_y = -map_y;
  map_z = -map_z;
  //map_yaw = -map_yaw;
  //map_pitch = -map_pitch;
  //map_roll = -map_roll;



//-----------------------------------------------------apply deadzone-----------------------------------------------------


  if (map_x       > -40 && map_x      < 40) map_x      = 0;
  if (map_y       > -40 && map_y      < 40) map_y      = 0;
  if (map_z       > -60 && map_z      < 60) map_z      = 0;
  if (map_yaw     > -90 && map_yaw    < 90) map_yaw    = 0;
  if (map_pitch   > -80 && map_pitch  < 80) map_pitch  = 0;
  if (map_roll    > -80 && map_roll   < 80) map_roll   = 0;
  

//-------------------------print deadzone-----------------------------------------------------


  //Serial.print(" | dead X: ");   print_sign   (map_x);
  //Serial.print(" | dead Y: ");   print_sign   (map_y);
  //Serial.print(" | dead Z: ");   print_sign   (map_z);
  //Serial.print(" | Yaw: ");      print_sign   (map_yaw);
  //Serial.print(" | Pitch: ");    print_sign   (map_pitch);
  //Serial.print(" | Roll: ");     print_sign   (map_roll);



//-----------------------------------------------------print buttons-----------------------------------------------------


  //Serial.print(" | Button0: "); print_sign(voltage0);
  //Serial.print(" | Button1: "); print_sign(voltage1);
  //Serial.print(" | Button2: "); print_sign(voltage2);
  //Serial.print(" | Button3: "); print_sign(voltage3);
  //Serial.print(" | Button4: "); print_sign(voltage4);
  //Serial.print(" | Button5: "); print_sign(voltage5);
  //Serial.print(" | Button6: "); print_sign(voltage6);
  //Serial.print(" | Button7: "); print_sign(voltage7);

  //Serial.print(" | Button1: "); print_sign(keyState[0]);
  //Serial.print(" | Button2: "); print_sign(keyState[1]);
  //Serial.print(" | Button3: "); print_sign(keyState[2]);
  //Serial.print(" | Button4: "); print_sign(keyState[3]);
  //Serial.print(" | Button5: "); print_sign(keyState[4]);
  //Serial.print(" | Button6: "); print_sign(keyState[5]);
  //Serial.print(" | Button7: "); print_sign(keyState[6]);
  //Serial.print(" | Button8: "); print_sign(keyState[7]);



//----------------------------------------------------gyro resets and works only if circular button pressed-----------------------------------------------------
 
  resetGyroValues();
  


 

//-----------------------------------------auto calibrate magnet xy if the mouse has been moved-----------------------------------------------------
   
  
  autoCalibrate(magX, magY, magZ, gyroX, gyroY, gyroZ);

  
//-----------------------------------------------------send values to spacemouse-----------------------------------------------------
  
  
  SpaceMouseHID.send_command((int16_t)map_roll, (int16_t)map_pitch, (int16_t)map_yaw, (int16_t)map_x, (int16_t)map_y, (int16_t)map_z, keyState, 2);
  


  // Check if button 1 and 5 are pressed for 4 seconds
  if (keyState[1] == 1 && keyState[5] == 1) {
    if (!eepromCalibTiming) {
      eepromCalibStartTime = millis();
      eepromCalibTiming = true;
      colorWipe(strip.Color(255, 0, 0), 4000); // Red
    } else if (millis() - eepromCalibStartTime > 4000) {
      calibrateMinMaxValues();
      isCalibrating = true;
      fadeRedInOut();
    }
  } else {
    if (eepromCalibTiming) {
      if (isCalibrating) {
        saveMinMaxValuesToEEPROM();
        isCalibrating = false;
        strip.show(); // Turn off LEDs after calibration
        // Print the min and max values saved to EEPROM
        Serial.print("Saved Min X: "); Serial.println(min_x);
        Serial.print("Saved Max X: "); Serial.println(max_x);
        Serial.print("Saved Min Y: "); Serial.println(min_y);
        Serial.print("Saved Max Y: "); Serial.println(max_y);
        Serial.print("Saved Min Z: "); Serial.println(min_z);
        Serial.print("Saved Max Z: "); Serial.println(max_z);
      }
      eepromCalibTiming = false;
    } else if (keyState[0] == 0) {
      staticColor(strip.Color(255, 255, 255), 150); // Immediately return to white
    }
  }

  if (isCalibrating) {
    // Show the magnet values while moving the sensor
    Serial.print(" | Current Mag X: "); print_sign(magX);
    Serial.print(" | Current Mag Y: "); print_sign(magY);
    Serial.print(" | Current Mag Z: "); print_sign(magZ);
  }

  Serial.println();

  delay(10);
}


void getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  float gyroZ_temp = g.gyro.z;

  if(abs(gyroZ_temp) > gyroZerror) {
    //Serial.print("    gyroZ_temp"); print_sign(gyroZ_temp);
    gyroZ += gyroZ_temp / 50.00;
  }

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp / 50.00;
  }


  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp / 50.00;
  }

}


void getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  //Serial.print("   AccX: "); Serial.print  (accX);
  //Serial.print("   AccY: "); Serial.print  (accY);
  //Serial.print("AccZ: "); Serial.println(accZ);
}


void calibrateMagnetometer() {
  float magX_sum = 0, magY_sum = 0, magZ_sum = 0;
  const int calibrationSamples = 100;

  Serial.println("setting the neutral values of magnetometer...");
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t event;
    mag.getEvent(&event);
    magX_sum += event.magnetic.x;
    magY_sum += event.magnetic.y;
    magZ_sum += event.magnetic.z;
    delay(50);
  }

  magX_offset = magX_sum / calibrationSamples;
  magY_offset = magY_sum / calibrationSamples;
  magZ_offset = magZ_sum / calibrationSamples;

  Serial.println("Magnetometer calibration complete.");
}


void calibrateMinMaxValues() {
  if (!isCalibrating) {
    min_x = min_y = min_z = FLT_MAX;
    max_x = max_y = max_z = FLT_MIN;
    Serial.println("Move the sensor in all directions to calibrate min and max values...");
  }

  sensors_event_t event;
  mag.getEvent(&event);
  mpu.getEvent(&a, &g, &temp);

  float magX = event.magnetic.x - magX_offset;
  float magY = event.magnetic.y - magY_offset;
  float magZ = event.magnetic.z - magZ_offset;

  if (magX < min_x) min_x = magX;
  if (magX > max_x) max_x = magX;
  if (magY < min_y) min_y = magY;
  if (magY > max_y) max_y = magY;
  if (magZ < min_z) min_z = magZ;
  if (magZ > max_z) max_z = magZ;

  Serial.print("Min X: "); Serial.println(min_x);
  Serial.print("Max X: "); Serial.println(max_x);
  Serial.print("Min Y: "); Serial.println(min_y);
  Serial.print("Max Y: "); Serial.println(max_y);
  Serial.print("Min Z: "); Serial.println(min_z);
  Serial.print("Max Z: "); Serial.println(max_z);
}

void saveMinMaxValuesToEEPROM() {
  EEPROM.put(0, min_x);
  EEPROM.put(sizeof(float), max_x);
  EEPROM.put(2 * sizeof(float), min_y);
  EEPROM.put(3 * sizeof(float), max_y);
  EEPROM.put(4 * sizeof(float), min_z);
  EEPROM.put(5 * sizeof(float), max_z);
  Serial.println("Min and max values saved to EEPROM.");
}

void readMinMaxValuesFromEEPROM() {
  EEPROM.get(0, min_x);
  EEPROM.get(sizeof(float), max_x);
  EEPROM.get(2 * sizeof(float), min_y);
  EEPROM.get(3 * sizeof(float), max_y);
  EEPROM.get(4 * sizeof(float), min_z);
  EEPROM.get(5 * sizeof(float), max_z);

  // Check if the values are valid, otherwise set default values
  if (isnan(min_x) || isnan(max_x) || isnan(min_y) || isnan(max_y) || isnan(min_z) || isnan(max_z)) {
    min_x = -100;
    min_y = -90; 
    min_z = -220; 
    max_x = 120;
    max_y = 130; 
    max_z = 115; 
    Serial.println("EEPROM values are invalid. Using default min and max values.");
  } else {
    Serial.println("Min and max values read from EEPROM.");
  }

  Serial.print("Min X: "); Serial.println(min_x);
  Serial.print("Max X: "); Serial.println(max_x);
  Serial.print("Min Y: "); Serial.println(min_y);
  Serial.print("Max Y: "); Serial.println(max_y);
  Serial.print("Min Z: "); Serial.println(min_z);
  Serial.print("Max Z: "); Serial.println(max_z);
}

void print_sign(float value) {
  if (value < 0) {
    Serial.print("-");
    Serial.print(-value);
  } else {
    Serial.print("+");
    Serial.print(value);
  }
}


void reset_value_yaw() {
  // Check if gyroZ is within the range of min to max
  if (gyroZ >= -.025 && gyroZ <= .025) {
    if (!isTiming) {
      // Start timing
      startTime = millis();
      lastGyroZValue = gyroZ;
      isTiming = true;
      //Serial.println("Started timing...");
    } else if (millis() - startTime > 150) {
      if (gyroZ == lastGyroZValue) {
        // If more than 2 seconds have passed and the value is the same, reset gyroZ
        gyroZ = 0;
        //Serial.println("gyroZ reset to 0 after 2 seconds.");
      }
      // Reset timing after 2 seconds regardless
      isTiming = false;
    } else {
      //Serial.print("Timing... "); Serial.print(millis() - startTime); Serial.println(" ms");
    }
  } else {
    // Reset timing if gyroZ is out of range
    isTiming = false;
    //Serial.println("gyroZ out of range, timing reset.");
  }
}


void resetGyroValues() {
  static int readoutCountX = 0;
  static int readoutCountY = 0;
  static int readoutCountZ = 0;

  // Check if no button is pressed
  if (keyState[0] == 0) {
    unsigned long currentTime = millis();

    if (abs(gyroX - lastGyroXValue) < 100) {
      readoutCountX++;
      if (readoutCountX >= 1) {
        gyroX = 0;
        readoutCountX = 0;
      }
    } else {
      lastGyroXValue = gyroX;
      lastGyroXTime = currentTime;
      readoutCountX = 0;
    }

    if (abs(gyroY - lastGyroYValue) < 100) {
      readoutCountY++;
      if (readoutCountY >= 1) {
        gyroY = 0;
        readoutCountY = 0;
      }
    } else {
      lastGyroYValue = gyroY;
      lastGyroYTime = currentTime;
      readoutCountY = 0;
    }

    if (abs(gyroZ - lastGyroZValue) < 100) {
      readoutCountZ++;
      if (readoutCountZ >= 1) {
        gyroZ = 0;
        readoutCountZ = 0;
      }
    } else {
      lastGyroZValue = gyroZ;
      lastGyroZTime = currentTime;
      readoutCountZ = 0;
    }
  }
}


void autoCalibrate(float magX, float magY, float magZ, float gyroX, float gyroY, float gyroZ) {
  if (magX > -60 && magX < 60 && magY > -60 && magY < 60 &&
      abs(magX - lastMagX) <= 1 && abs(magY - lastMagY) <= 1 &&
      abs(magZ - lastMagZ) <= 1 && abs(gyroX - lastGyroX) <= 0.1 &&
      abs(gyroY - lastGyroY) <= 0.1 && abs(gyroZ - lastGyroZ) <= 0.1) {
    
    if (!autoCalibTiming) {
      autoCalibStartTime = millis();
      autoCalibTiming = true;
    } else if (millis() - autoCalibStartTime > autoCalibDuration) {
      magX_offset += magX;
      magY_offset += magY;
      autoCalibTiming = false;
      Serial.println("Auto-calibration complete. Offsets updated.");
    }
  } else {
    autoCalibTiming = false;
  }

  lastMagX = magX;
  lastMagY = magY;
  lastMagZ = magZ;
  lastGyroX = gyroX;
  lastGyroY = gyroY;
  lastGyroZ = gyroZ;
}

void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}


uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void staticColor(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void fadeRedInOut() {
  static unsigned long lastFadeUpdate = 0;
  static int brightness = 0;
  static int fadeAmount = 5;

  if (millis() - lastFadeUpdate > 10) {
    strip.fill(strip.Color(brightness, 0, 0));
    strip.show();
    brightness += fadeAmount;
    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }
    lastFadeUpdate = millis();
  }
}
