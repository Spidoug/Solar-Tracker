#include <Wire.h>
#include "RTClib.h"
#include <Servo.h>
#include <Adafruit_HMC5883_U.h>
#include <EEPROM.h>
#include <math.h>

// Function declarations
void solarAutoCalibration();
void checkOrientation();
bool timeWithinInterval(DateTime now, int startHour, int startMin, int endHour, int endMin);
void adjustSystemTime();
void adjustSunrise();
void adjustSunset();
void adjustMagneticDeclination();
void displayMenu();
void processInput();
void displayCurrentStatus(DateTime now);
void startupSound();
void compassCalibrationSound();
void servoMovementSound();
void confirmationSound();
void orientationConfirmationSound();
void orientationErrorSound();
void rtcErrorSound();
void compassErrorSound();
void inputErrorSound();
void saveFilterStateEEPROM();
void loadFilterStateEEPROM();
void saveTimesEEPROM();
void loadTimesEEPROM();
void saveMagneticDeclinationEEPROM();
void loadMagneticDeclinationEEPROM();
void loadActiveModeEEPROM();
void saveActiveModeEEPROM();
void checkEEPROM();
float getCompassOrientation();
void resetErrorSum();
void calibrateCompass();
void debouncer(int pin, int& previousRead, int& currentRead);
void adjustAutoCalibrationTime();
void manualAdjust();

// Servo motor
Servo servoMotor;

// Set min and max values for compass axis calibration
float magX_min = -32768, magX_max = 32767;
float magY_min = -32768, magY_max = 32767;
float magZ_min = -32768, magZ_max = 32767;

// Servo, LDRs, and other components
const int ldrWest = A0;
const int ldrEast = A1;
const int servoPin = 3;
const int buzzerPin = 13;

// PID control variables and solar system settings
float Kp = 0.5, Ki = 0.1, Kd = 0.05;
float previousError = 0, errorSum = 0;
const float errorSumLimit = 1000;
float maxIrradiance = 0.0;
int maxIrradianceAngle = 90;
const int minLimit = 30, maxLimit = 150;
int servoAngle = 90;

// RTC and compass
RTC_PCF8563 rtc;
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

// Compass module position compensation value
float compVl = 33.01;

// Timing and periodic readings
unsigned long lastReadTime = 0;
const unsigned long readInterval = 1000;

// Default sunrise and sunset times
int sunriseHour = 6;
int sunriseMinute = 0;
int sunsetHour = 18;
int sunsetMinute = 0;

// Magnetic declination
float magneticDeclination = 0.0; // Default value

// Variables not previously declared
int previousLdrWestRead = 0;
int previousLdrEastRead = 0;
float previousIrradiance = 0.0;

// Variables for anti-disturbance filter
bool antiDisturbanceFilterEnabled = false;
bool activeMode = false;
bool manualMode = false;
unsigned long lastAlert = 0;
const unsigned long alertInterval = 1000;  // 1-second interval for misalignment beeps

// Serial input buffer
String inputBuffer = "";

// Alignment control variables
bool correctOrientation = false;
bool alertingMisalignment = false;

// Moving average variables
int movingAverageLdrWest[5] = {0};
int movingAverageLdrEast[5] = {0};
int averageIndex = 0;
int sumLdrWest = 0;
int sumLdrEast = 0;

void setup() {
  // Play startup sound
  startupSound();

  // Initialize servo motor on designated pin
  servoMotor.attach(servoPin);
  servoMotor.write(servoAngle);

  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Solar Tracker 1.00"));
  Serial.println(F(""));

  // Initialize the RTC
  if (!rtc.begin()) {
    rtcErrorSound();
    Serial.println(F("Unable to find RTC."));
    Serial.println(F(""));
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize the HMC5883L compass module
  if (!compass.begin()) {
    compassErrorSound();
    Serial.println(F("Failed to initialize compass."));
    Serial.println(F(""));
    while (1);
  }

  // Check if EEPROM data is correct
  checkEEPROM();

  // Load times and magnetic declination from EEPROM
  loadTimesEEPROM();
  loadMagneticDeclinationEEPROM();

  // Load anti-disturbance filter state from EEPROM
  loadFilterStateEEPROM();

  // Load the last system operating mode from EEPROM
  loadActiveModeEEPROM();

  // Start compass calibration
  calibrateCompass();

  // Play compass calibration completion sound
  compassCalibrationSound();

  // Solar irradiance auto-calibration
  solarAutoCalibration();

  // Display menu on startup
  displayMenu();

  if (antiDisturbanceFilterEnabled) {
    Serial.println(F(""));
    Serial.println(F("Anti-disturbance filter enabled."));
    Serial.println(F(""));
  } else {
    Serial.println(F(""));
    Serial.println(F("Anti-disturbance filter disabled."));
    Serial.println(F(""));
  }

  if (activeMode) {
    Serial.println(F(""));
    Serial.println(F("Operation mode: active."));
    Serial.println(F(""));
  } else {
    Serial.println(F(""));
    Serial.println(F("Operation mode: passive."));
    Serial.println(F(""));
  }
}

void loop() {
  DateTime now = rtc.now();
  checkOrientation();

  if (millis() - lastReadTime >= readInterval) {
    // LDR readings with moving average
    sumLdrWest -= movingAverageLdrWest[averageIndex];
    sumLdrEast -= movingAverageLdrEast[averageIndex];
    movingAverageLdrWest[averageIndex] = analogRead(ldrWest);
    movingAverageLdrEast[averageIndex] = analogRead(ldrEast);
    sumLdrWest += movingAverageLdrWest[averageIndex];
    sumLdrEast += movingAverageLdrEast[averageIndex];
    averageIndex = (averageIndex + 1) % 5;

    int ldrWestRead = sumLdrWest / 5;
    int ldrEastRead = sumLdrEast / 5;

    // Irradiance calculation
    int a2Value = analogRead(A2);
    float a2Voltage = (a2Value / 1023.0) * 5.0;
    float irradiance = (a2Voltage / 5.0) * 1000.0;

    if (manualMode == true) {
      displayCurrentStatus(now);
    } else {
      if (timeWithinInterval(now, sunriseHour, sunriseMinute, sunsetHour, sunsetMinute)) {
        if (activeMode == true) {
          // Active mode: control based on LDRs
          int error = ldrWestRead - ldrEastRead;
          int maxTolerance = 100;
          if (antiDisturbanceFilterEnabled) {
            maxTolerance = 80;
          } else {
            maxTolerance = 10;
          }
          if (abs(error) > maxTolerance) {  // Adjusted tolerance to 10
            errorSum += error;
            errorSum = constrain(errorSum, -errorSumLimit, errorSumLimit);  // Limit the error sum
            float derivative = error - previousError;
            float adjustment = Kp * error + Ki * errorSum + Kd * derivative;
            previousError = error;
            servoAngle = servoAngle + adjustment;
            servoAngle = constrain(servoAngle, minLimit, maxLimit);
            servoMotor.write(servoAngle);
            servoMovementSound();
          } else {
            resetErrorSum();  // Reset the error sum when within tolerance
          }
        } else {
          // Passive mode: move the servo proportionally to time within the solar interval

          // Calculate total operation time in minutes
          int startMinutes = sunriseHour * 60 + sunriseMinute;
          int endMinutes = sunsetHour * 60 + sunsetMinute;
          int currentMinutes = now.hour() * 60 + now.minute();
          int totalTime = endMinutes - startMinutes;  // Total operation time in minutes
          int elapsedTime = currentMinutes - startMinutes;  // Elapsed time since sunrise

          // Calculate the angle based on elapsed time
          float timeProportion = (float)elapsedTime / (float)totalTime;
          servoAngle = minLimit + timeProportion * (maxLimit - minLimit);

          // Constrain the angle within limits
          servoAngle = constrain(servoAngle, minLimit, maxLimit);

          // Move the servo to the calculated angle
          servoMotor.write(servoAngle);
        }
        displayCurrentStatus(now);

      } else {
        servoAngle = 90;  // Neutral position outside operation time
        servoMotor.write(servoAngle);
        Serial.println(F("Outside operating hours."));
        Serial.println(F(""));
      }
    }

    previousLdrWestRead = ldrWestRead;
    previousLdrEastRead = ldrEastRead;
    previousIrradiance = irradiance;

    lastReadTime = millis();
  }

  // Non-blocking serial input reading
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInput();
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// Function to check if the current time is within the operating interval (sunrise to sunset)
bool timeWithinInterval(DateTime now, int startHour, int startMin, int endHour, int endMin) {
  int currentMinutes = now.hour() * 60 + now.minute();
  int startMinutes = startHour * 60 + startMin;
  int endMinutes = endHour * 60 + endMin;

  return (currentMinutes >= startMinutes && currentMinutes <= endMinutes);
}

// Function to reset the error sum (used in PID control)
void resetErrorSum() {
  errorSum = 0;
}

// Function to calibrate the compass
void calibrateCompass() {
  Serial.println(F("Calibrating compass..."));
  Serial.println(F(""));
  Serial.println(F("Rotate the sensor in all directions to collect calibration data."));

  for (int i = 0; i < 1000; i++) {
    sensors_event_t event;
    compass.getEvent(&event);

    // Update min and max values for X, Y, Z axes
    magX_min = min(magX_min, event.magnetic.x);
    magX_max = max(magX_max, event.magnetic.x);
    magY_min = min(magY_min, event.magnetic.y);
    magY_max = max(magY_max, event.magnetic.y);
    magZ_min = min(magZ_min, event.magnetic.z);
    magZ_max = max(magZ_max, event.magnetic.z);

    delay(10);  // Wait to allow sensor movement
  }

  Serial.println(F(""));
  Serial.println(F("Calibration complete!"));
  Serial.println(F(""));
}

float getCompassOrientation() {
  sensors_event_t event;
  compass.getEvent(&event);

  // Correct readings of X, Y, Z axes based on calibrated values
  float magX = (event.magnetic.x - magX_min) / (magX_max - magX_min) * 2 - 1;
  float magY = (event.magnetic.y - magY_min) / (magY_max - magY_min) * 2 - 1;

  // Calculate the corrected heading (orientation)
  float heading = atan2(magY, magX) * 180 / PI;

  // Adjust for negative values and correct magnetic declination
  heading += magneticDeclination;
  if (heading >= 360) {
    heading -= 360;
  }
  
  if (heading < 0) {
    heading += 360;
  }

  return heading;
}

void displayMenu() {
  Serial.println(F("\nSelect an option:"));
  Serial.println(F(""));
  Serial.println(F("1. Adjust system time"));
  Serial.println(F("2. Adjust sunrise time"));
  Serial.println(F("3. Adjust sunset time"));
  Serial.println(F("4. Toggle anti-disturbance filter (Enable/Disable)"));
  Serial.println(F("5. Adjust magnetic declination"));
  Serial.println(F("6. Toggle between active or passive operation mode"));
  Serial.println(F("7. Enter manual positioning mode"));
  Serial.println(F(""));
  Serial.println(F("Enter the option number and press Enter."));
  Serial.println(F(""));
}

void processInput() {
  String input = inputBuffer;
  if (input == "1") {
    adjustSystemTime();
  } else if (input == "2") {
    adjustSunrise();
  } else if (input == "3") {
    adjustSunset();
  } else if (input == "4") {
    antiDisturbanceFilterEnabled = !antiDisturbanceFilterEnabled;
    saveFilterStateEEPROM();
    if (antiDisturbanceFilterEnabled) {
      Serial.println(F(""));
      Serial.println(F("Anti-disturbance filter enabled."));
      confirmationSound();
    } else {
      Serial.println(F(""));
      Serial.println(F("Anti-disturbance filter disabled."));
      confirmationSound();
    }
    Serial.println(F(""));
  } else if (input == "5") {
    adjustMagneticDeclination();
  } else if (input == "6") {
    activeMode = !activeMode;
    saveActiveModeEEPROM();
    if (activeMode) {
      Serial.println(F(""));
      Serial.println(F("Operation mode: active."));
      confirmationSound();
    } else {
      Serial.println(F(""));
      Serial.println(F("Operation mode: passive."));
      confirmationSound();
    }
    Serial.println(F(""));
  } else if (input == "7") {
    manualAdjust();
  } else {
    Serial.println(F(""));
    Serial.println(F("Invalid option. Please select a valid option."));
    Serial.println(F(""));
    inputErrorSound();
  }

  displayMenu();
}

void adjustMagneticDeclination() {
  Serial.println(F("Enter the magnetic declination value (in degrees):"));
  while (Serial.available() == 0);
  String declinationInput = Serial.readStringUntil('\n');
  declinationInput.trim();
  float declination = declinationInput.toFloat();
  if (declination != 0 || declinationInput == "0") {
    magneticDeclination = declination + compVl;
    saveMagneticDeclinationEEPROM();
    Serial.println(F("Magnetic declination set!"));
    Serial.println(F(""));
    orientationConfirmationSound();
  } else {
    Serial.println(F("Invalid input. Please make sure to enter a valid number."));
    Serial.println(F(""));
    inputErrorSound();
  }
}

void saveMagneticDeclinationEEPROM() {
  int declinationInt = magneticDeclination * 100;
  EEPROM.write(6, declinationInt >> 8);
  EEPROM.write(7, declinationInt & 0xFF);
}

void loadMagneticDeclinationEEPROM() {
  int declinationInt = (EEPROM.read(6) << 8) | EEPROM.read(7);
  magneticDeclination = declinationInt / 100.0;
}

void saveFilterStateEEPROM() {
  EEPROM.write(0, antiDisturbanceFilterEnabled);
}

void loadFilterStateEEPROM() {
  antiDisturbanceFilterEnabled = EEPROM.read(0);
}

void saveActiveModeEEPROM() {
  EEPROM.write(5, activeMode);
}

void loadActiveModeEEPROM() {
  activeMode = EEPROM.read(5);
}

void adjustSystemTime() {
  Serial.println(F(""));
  Serial.println(F("Enter the new date and time in the format: YYYY MM DD HH MM SS"));
  while (Serial.available() == 0);
  String dateTime = Serial.readStringUntil('\n');
  dateTime.trim();
  int year, month, day, hour, minute, second;
  if (sscanf(dateTime.c_str(), "%d %d %d %d %d %d", &year, &month, &day, &hour, &minute, &second) == 6) {
    rtc.adjust(DateTime(year, month, day, hour, minute, second));
    Serial.println(F("Date and time set!"));
    confirmationSound();
  } else {
    Serial.println(F("Invalid input. Make sure to follow the format: YYYY MM DD HH MM SS"));
    inputErrorSound();
  }
  Serial.println(F(""));
}

void adjustSunrise() {
  Serial.println(F(""));
  Serial.println(F("Enter the sunrise time in the format: HH MM"));
  while (Serial.available() == 0);
  String sunriseInput = Serial.readStringUntil('\n');
  sunriseInput.trim();
  int hSunrise, mSunrise;
  if (sscanf(sunriseInput.c_str(), "%d %d", &hSunrise, &mSunrise) == 2) {
    sunriseHour = hSunrise;
    sunriseMinute = mSunrise;
    saveTimesEEPROM();
    Serial.println(F("Sunrise time set!"));
    confirmationSound();
  } else {
    Serial.println(F("Invalid input. Make sure to follow the format: HH MM"));
    inputErrorSound();
  }
  Serial.println(F(""));
}

void adjustSunset() {
  Serial.println(F(""));
  Serial.println(F("Enter the sunset time in the format: HH MM"));
  while (Serial.available() == 0);
  String sunsetInput = Serial.readStringUntil('\n');
  sunsetInput.trim();
  int hSunset, mSunset;
  if (sscanf(sunsetInput.c_str(), "%d %d", &hSunset, &mSunset) == 2) {
    sunsetHour = hSunset;
    sunsetMinute = mSunset;
    saveTimesEEPROM();
    Serial.println(F("Sunset time set!"));
    confirmationSound();
  } else {
    Serial.println(F("Invalid input. Make sure to follow the format: HH MM"));
    inputErrorSound();
  }
  Serial.println(F(""));
}

void manualAdjust() {
  Serial.println(F(""));
  Serial.println(F("Enter the solar panel angle between 30° and 150°. If input is invalid, the system will operate in automatic mode."));
  while (Serial.available() == 0);
  String angleDef = Serial.readStringUntil('\n');
  angleDef.trim();
  int angle = angleDef.toInt();
  if (angle >= 30 && angle <= 150) {
    manualMode = true;
    servoAngle = angle;
    servoMotor.write(servoAngle);
    confirmationSound();
    Serial.println(F("Manual angle set. The system is now in manual mode!"));
    Serial.println(F(""));
    orientationConfirmationSound();
  } else {
    Serial.println(F("Invalid input. Make sure to enter a valid number."));
    Serial.println(F(""));
    Serial.println(F("The system is now in automatic mode!"));
    Serial.println(F(""));
    inputErrorSound();
    manualMode = false;
  }
}

void saveTimesEEPROM() {
  EEPROM.write(1, sunriseHour);
  EEPROM.write(2, sunriseMinute);
  EEPROM.write(3, sunsetHour);
  EEPROM.write(4, sunsetMinute);
}

void loadTimesEEPROM() {
  sunriseHour = EEPROM.read(1);
  sunriseMinute = EEPROM.read(2);
  sunsetHour = EEPROM.read(3);
  sunsetMinute = EEPROM.read(4);
}

void startupSound() {
  tone(buzzerPin, 1000, 200);
  delay(300);
  noTone(buzzerPin);
}

void compassCalibrationSound() {
  tone(buzzerPin, 8000, 500);
  delay(900);
  noTone(buzzerPin);
}

void servoMovementSound() {
  tone(buzzerPin, 1800, 200);
  delay(200);
  noTone(buzzerPin);
}

void confirmationSound() {
  tone(buzzerPin, 1000, 300);
  delay(400);
  tone(buzzerPin, 1200, 300);
  delay(400);
}

void orientationConfirmationSound() {
  tone(buzzerPin, 2500, 500);
  delay(600);
  noTone(buzzerPin);
}

void orientationErrorSound() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 400, 100);
    delay(150);
    noTone(buzzerPin);
  }
}

void rtcErrorSound() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 300, 500);
    delay(250);
    noTone(buzzerPin);
  }
}

void compassErrorSound() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 250, 240);
    delay(450);
    noTone(buzzerPin);
  }
}

void inputErrorSound() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 330, 140);
    delay(130);
    noTone(buzzerPin);
  }
}

void checkOrientation() {
  float heading = getCompassOrientation();

  if (heading >= 358 || heading <= 2) {
    if (!correctOrientation) {
      correctOrientation = true;
      alertingMisalignment = false;
      orientationConfirmationSound();
      Serial.println(F("Correct orientation relative to north."));
      Serial.println(F(""));
    }
  } else {
    if (!alertingMisalignment) {
      correctOrientation = false;
      alertingMisalignment = true;
      lastAlert = millis();
      Serial.print(F("Orientation outside expected: "));
      Serial.print(heading);
      Serial.println(F("° . Adjustment needed."));
      Serial.println(F(""));
      orientationErrorSound();
    }

    if (millis() - lastAlert >= alertInterval) {
      orientationErrorSound();
      lastAlert = millis();
    }
  }
}

void solarAutoCalibration() {
  Serial.println(F("Starting Auto-Calibration..."));
  Serial.println(F(""));

  // Dynamically determine delay time based on irradiance variation
  int delayTime = 500;

  for (int angle = minLimit; angle <= maxLimit; angle++) {
    servoMotor.write(angle);
    delay(delayTime);

    int a2Value = analogRead(A2);
    float a2Voltage = (a2Value / 1023.0) * 5.0;
    float irradiance = (a2Voltage / 5.0) * 1000.0;

    if (irradiance > maxIrradiance) {
      maxIrradiance = irradiance;
      maxIrradianceAngle = angle;
    }

    // Adjust delay time based on irradiance variation
    if (abs(irradiance - maxIrradiance) < 50) {
      delayTime = 300;  // Reduce delay time if variation is small
    } else {
      delayTime = 500;  // Increase delay time for large variations
    }

    Serial.print(F("Angle: "));
    Serial.print(angle);
    Serial.print(F("° - Irradiance: "));
    Serial.print(irradiance);
    Serial.println(F(" W/m²"));
  }

  servoMotor.write(maxIrradianceAngle);
  Serial.println(F(""));
  Serial.println(F("Auto-Calibration Complete!"));
  Serial.println(F(""));
  Serial.print(F("Best Angle: "));
  Serial.print(maxIrradianceAngle);
  Serial.println(F("°"));
  Serial.print(F("Max Irradiance: "));
  Serial.print(maxIrradiance);
  Serial.println(F(" W/m²"));
  Serial.println(F(""));
}

void displayCurrentStatus(DateTime now) {
  float heading = getCompassOrientation();

  Serial.print(F("Date: "));
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(F(" Time: "));
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);

  Serial.print(F(" - Angle: "));
  Serial.print(servoAngle);
  Serial.print(F("° - LDR West Reading: "));
  Serial.print(previousLdrWestRead);
  Serial.print(F(" - LDR East Reading: "));
  Serial.print(previousLdrEastRead);
  Serial.print(F(" - Irradiance: "));
  Serial.print(previousIrradiance);
  Serial.print(F(" W/m² - Compass Orientation: "));
  Serial.print(heading);
  Serial.println(F("°"));
}

void checkEEPROM() {
  // Function to check if EEPROM values are valid (simple validation implementation)
  int eepromData = EEPROM.read(0);
  if (eepromData < 0 || eepromData > 1) {
    EEPROM.write(0, 0);  // If invalid, reset to 0 (disabled)
  }
}
