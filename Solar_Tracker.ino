#include <Wire.h>
#include "RTClib.h"
#include <Servo.h>
#include <Adafruit_HMC5883_U.h>
#include <EEPROM.h>
#include <math.h>

// Function declarations
void solarAutoCalibration();
void checkOrientation();
bool isTimeWithinRange(DateTime now, int startHour, int startMin, int endHour, int endMin);
void adjustSystemTime();
void adjustSunriseTime();
void adjustSunsetTime();
void adjustMagneticDeclination();
void displayMenu();
void processInput();
void displayCurrentStatus(DateTime now);
void initialSoundAlert();
void compassCalibrationSoundAlert();
void servoMovementSoundAlert();
void confirmationSoundAlert();
void orientationConfirmationSound();
void orientationErrorSoundAlert();
void rtcErrorSoundAlert();
void compassErrorSoundAlert();
void inputErrorSoundAlert();
void saveFilterStateToEEPROM();
void loadFilterStateFromEEPROM();
void saveTimingsToEEPROM();
void loadTimingsFromEEPROM();
void saveMagneticDeclinationToEEPROM();
void loadMagneticDeclinationFromEEPROM();
void loadActiveOperationModeFromEEPROM();
void saveActiveOperationModeToEEPROM();
void checkEEPROM();
float getCompassOrientation();
void resetErrorSum();
void calibrateCompass();
void debounce(int pin, int& previousRead, int& currentRead);
void adjustAutoCalibrationTime();
void manualAdjustment();

// Servo motor
Servo servoMotor;

// Set the min and max axis values for compass calibration
float magX_min = -32768, magX_max = 32767;
float magY_min = -32768, magY_max = 32767;
float magZ_min = -32768, magZ_max = 32767;

// Servo, LDRs and other components
const int westLdr = A0;
const int eastLdr = A1;
const int servoPin = 3;
const int buzzerPin = 13;

// PID control variables and solar system
float Kp = 0.5, Ki = 0.1, Kd = 0.05;
float previousError = 0, errorSum = 0;
const float maxErrorSum = 1000;
float maxIrradiance = 0.0;
int maxIrradianceAngle = 90;
const int minLimit = 30, maxLimit = 150;
int servoAngle = 90;

// RTC and compass
RTC_PCF8563 rtc;
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

// Compass module position compensation
float compVl = 33.01;

// Timing and periodic readings
unsigned long lastReadingTime = 0;
const unsigned long readingInterval = 1000;

// Sunrise and sunset timings (default values)
int sunriseHour = 6;
int sunriseMinute = 0;
int sunsetHour = 18;
int sunsetMinute = 0;

// Magnetic declination
float magneticDeclination = 0.0; // Default value

// Previously undeclared variables
int previousWestLdrRead = 0;
int previousEastLdrRead = 0;
float previousIrradiance = 0.0;
int maxTolerance = 100;

// Sudden variation filter variables
bool suddenVariationFilterEnabled = false;
bool activeOperationMode = false;
bool manualMode = false;
unsigned long lastAlert = 0;
const unsigned long alertInterval = 1000;  // 1-second interval for misalignment beeps

// Serial reading buffer
String inputBuffer = "";

// Alignment control variables
bool correctOrientation = false;
bool misalignmentAlert = false;

// Moving average variables
int westLdrAverage[5] = {0};
int eastLdrAverage[5] = {0};
int averageIndex = 0;
int westLdrSum = 0;
int eastLdrSum = 0;

#define EEPROM_SIGNATURE 0xAB  // EEPROM signature identifier

// EEPROM addresses for each setting
const int EEPROM_ADDR_SIGNATURE = 0;
const int EEPROM_ADDR_FILTER_STATE = 1;
const int EEPROM_ADDR_SUNRISE_HOUR = 2;
const int EEPROM_ADDR_SUNRISE_MINUTE = 3;
const int EEPROM_ADDR_SUNSET_HOUR = 4;
const int EEPROM_ADDR_SUNSET_MINUTE = 5;
const int EEPROM_ADDR_OPERATION_MODE = 6;
const int EEPROM_ADDR_DECLINATION_HIGH = 7;
const int EEPROM_ADDR_DECLINATION_LOW = 8;

void setup() {

  // Play initial sound
  initialSoundAlert();

  // Check if EEPROM data is correct
  checkEEPROM();

  // Load the sudden variation filter state from EEPROM
  loadFilterStateFromEEPROM();

  // Load timings and magnetic declination from EEPROM
  loadTimingsFromEEPROM();

  // Load last system operation mode from EEPROM
  loadActiveOperationModeFromEEPROM();

  // Load Magnetic Declination value from EEPROM
  loadMagneticDeclinationFromEEPROM();

  // Initialize servo motor on designated pin
  servoMotor.attach(servoPin);
  servoMotor.write(servoAngle);

  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Solar Tracker"));
  Serial.println(F(""));

  // Start compass calibration
  calibrateCompass();

  // Display menu at startup
  displayMenu();

  if (suddenVariationFilterEnabled) {
    Serial.println(F(""));
    Serial.println(F("Sudden variation filter enabled."));
    Serial.println(F(""));
  } else {
    Serial.println(F(""));
    Serial.println(F("Sudden variation filter disabled."));
    Serial.println(F(""));
  }

  if (activeOperationMode) {
    Serial.println(F(""));
    Serial.println(F("Operation mode is set to active."));
    Serial.println(F(""));
  } else {
    Serial.println(F(""));
    Serial.println(F("Operation mode is set to passive."));
    Serial.println(F(""));
  }

  // Initialize RTC
  if (!rtc.begin()) {
    rtcErrorSoundAlert();
    Serial.println(F("RTC not found."));
    Serial.println(F(""));
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize HMC5883L compass module
  if (!compass.begin()) {
    compassErrorSoundAlert();
    Serial.println(F("Failed to initialize compass."));
    Serial.println(F(""));
    while (1);
  }
}

void loop() {
  DateTime now = rtc.now();
  checkOrientation();

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

  if (millis() - lastReadingTime >= readingInterval) {
    // Read LDRs with moving average
    westLdrSum -= westLdrAverage[averageIndex];
    eastLdrSum -= eastLdrAverage[averageIndex];
    westLdrAverage[averageIndex] = analogRead(westLdr);
    eastLdrAverage[averageIndex] = analogRead(eastLdr);
    westLdrSum += westLdrAverage[averageIndex];
    eastLdrSum += eastLdrAverage[averageIndex];
    averageIndex = (averageIndex + 1) % 5;

    int westLdrRead = westLdrSum / 5;
    int eastLdrRead = eastLdrSum / 5;

    // Irradiance calculation
    int a2Value = analogRead(A2);
    float a2Voltage = (a2Value / 1023.0) * 5.0;
    float irradiance = (a2Voltage / 5.0) * 1000.0;

    if (manualMode == true) {
      displayCurrentStatus(now);
    } else {
      if (isTimeWithinRange(now, sunriseHour, sunriseMinute, sunsetHour, sunsetMinute)) {
        if (activeOperationMode == true) {
          // Active mode: control based on LDRs
          int error = westLdrRead - eastLdrRead;
          
          if (suddenVariationFilterEnabled) {
            maxTolerance = 100;
          } else {
            maxTolerance = 10;
          }
          
          if (abs(error) > maxTolerance) {  // Adjusted tolerance to 10
            errorSum += error;
            errorSum = constrain(errorSum, -maxErrorSum, maxErrorSum);  // Limit error sum
            float derivative = error - previousError;
            float adjustment = Kp * error + Ki * errorSum + Kd * derivative;
            previousError = error;
            servoAngle = servoAngle + adjustment;
            servoAngle = constrain(servoAngle, minLimit, maxLimit);
            servoMotor.write(servoAngle);
            servoMovementSoundAlert();
          } else {
            resetErrorSum();  // Reset error sum when within tolerance
          }
        } else {
          // Passive mode: move servo proportionally to time within solar interval

          // Calculate the total operating period in minutes
          int startMinutes = sunriseHour * 60 + sunriseMinute;
          int endMinutes = sunsetHour * 60 + sunsetMinute;
          int currentMinutes = now.hour() * 60 + now.minute();
          int totalTime = endMinutes - startMinutes;  // Total operating time in minutes
          int elapsedTime = currentMinutes - startMinutes;  // Time since sunrise

          // Calculate angle based on elapsed time
          float timeRatio = (float)elapsedTime / (float)totalTime;
          servoAngle = minLimit + timeRatio * (maxLimit - minLimit);

          // Constrain angle within limits
          servoAngle = constrain(servoAngle, minLimit, maxLimit);

          // Move servo to calculated angle
          servoMotor.write(servoAngle);
        }
        displayCurrentStatus(now);

      } else {
        servoAngle = 90;  // Neutral position outside operating hours
        servoMotor.write(servoAngle);
        Serial.println(F("Outside operating hours."));
        Serial.println(F(""));
      }
    }

    previousWestLdrRead = westLdrRead;
    previousEastLdrRead = eastLdrRead;
    previousIrradiance = irradiance;

    lastReadingTime = millis();
  }
}

// Function to check if EEPROM values are valid and initialize with default values if necessary
void checkEEPROM() {
  // Check if EEPROM signature is present
  if (EEPROM.read(EEPROM_ADDR_SIGNATURE) != EEPROM_SIGNATURE) {
    // EEPROM not properly initialized, set default values
    EEPROM.write(EEPROM_ADDR_SIGNATURE, EEPROM_SIGNATURE);       // Write signature to indicate initialization
    EEPROM.write(EEPROM_ADDR_FILTER_STATE, 1);                   // Sudden variation filter disabled (0)
    EEPROM.write(EEPROM_ADDR_SUNRISE_HOUR, 06);                   // Sunrise time 6:00
    EEPROM.write(EEPROM_ADDR_SUNRISE_MINUTE, 00);
    EEPROM.write(EEPROM_ADDR_SUNSET_HOUR, 18);                   // Sunset time 18:00
    EEPROM.write(EEPROM_ADDR_SUNSET_MINUTE, 00);
    EEPROM.write(EEPROM_ADDR_OPERATION_MODE, 0);                 // Passive operation mode (0)

    // Default magnetic declination in degrees (0.0), scaled to an integer value
    int declinationDefault = 0 * 100;
    EEPROM.write(EEPROM_ADDR_DECLINATION_HIGH, declinationDefault >> 8);
    EEPROM.write(EEPROM_ADDR_DECLINATION_LOW, declinationDefault & 0xFF);
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

// Function to check if the current time is within the operating interval (sunrise to sunset)
bool isTimeWithinRange(DateTime now, int startHour, int startMin, int endHour, int endMin) {
  int currentMinutes = now.hour() * 60 + now.minute();
  int startMinutes = startHour * 60 + startMin;
  int endMinutes = endHour * 60 + endMin;

  return (currentMinutes >= startMinutes && currentMinutes <= endMinutes);
}

// Function to reset the error sum (used in PID control)
void resetErrorSum() {
  errorSum = 0;
}

// Compass calibration function
void calibrateCompass() {
  Serial.println(F("Calibrating the compass..."));
  Serial.println(F(""));
  Serial.println(F("Rotate the sensor in all directions to collect calibration data."));

  for (int i = 0; i < 1000; i++) {
    sensors_event_t event;
    compass.getEvent(&event);

    // Update the min and max values of the X, Y, Z axes
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

  // Play compass calibration completion sound
  compassCalibrationSoundAlert();
}

float getCompassOrientation() {
  sensors_event_t event;
  compass.getEvent(&event);

  // Correct X, Y, Z axis readings based on calibrated values
  float magX = (event.magnetic.x - magX_min) / (magX_max - magX_min) * 2 - 1;
  float magY = (event.magnetic.y - magY_min) / (magY_max - magY_min) * 2 - 1;

  // Calculate corrected heading
  float heading = atan2(magY, magX) * 180 / PI;

  // Adjust for negative values and correct for magnetic declination
  heading += magneticDeclination + compVl;

  if (heading < 0) {
    heading += 360;
  }

  if (heading >= 360) {
    heading -= 360;
  }

  return heading;
}

void checkOrientation() {
  float heading = getCompassOrientation();

  if (heading >= 358 || heading <= 2) {
    if (!correctOrientation) {
      correctOrientation = true;
      misalignmentAlert = false;
      orientationConfirmationSound();
      Serial.println(F("Correct orientation with respect to north."));
      Serial.println(F(""));
    }
  } else {
    if (!misalignmentAlert) {
      correctOrientation = false;
      misalignmentAlert = true;
      lastAlert = millis();
      Serial.print(F("Unexpected orientation: "));
      Serial.print(heading);
      Serial.println(F("° . Adjustment needed."));
      Serial.println(F(""));
      orientationErrorSoundAlert();
    }

    if (millis() - lastAlert >= alertInterval) {
      orientationErrorSoundAlert();
      lastAlert = millis();
    }
  }
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
  Serial.print(F("° - West LDR Reading: "));
  Serial.print(previousWestLdrRead);
  Serial.print(F(" - East LDR Reading: "));
  Serial.print(previousEastLdrRead);
  Serial.print(F(" - Irradiance: "));
  Serial.print(previousIrradiance);
  Serial.print(F(" W/m² - Compass Orientation: "));
  Serial.print(heading);
  Serial.println(F("°"));
}

void displayMenu() {
  Serial.println(F("\nSelect an option:"));
  Serial.println(F(""));
  Serial.println(F("1. Adjust system time"));
  Serial.println(F("2. Adjust sunrise time"));
  Serial.println(F("3. Adjust sunset time"));
  Serial.println(F("4. Toggle sudden variation filter (Enable/Disable)"));
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
    adjustSunriseTime();
  } else if (input == "3") {
    adjustSunsetTime();
  } else if (input == "4") {
    suddenVariationFilterEnabled = !suddenVariationFilterEnabled;
    saveFilterStateToEEPROM();
    if (suddenVariationFilterEnabled) {
      Serial.println(F(""));
      Serial.println(F("Sudden variation filter enabled."));
      confirmationSoundAlert();
    } else {
      Serial.println(F(""));
      Serial.println(F("Sudden variation filter disabled."));
      confirmationSoundAlert();
    }
    Serial.println(F(""));
  } else if (input == "5") {
    adjustMagneticDeclination();
  } else if (input == "6") {
    activeOperationMode = !activeOperationMode;
    saveActiveOperationModeToEEPROM();
    if (activeOperationMode) {
      Serial.println(F(""));
      Serial.println(F("Operation mode is set to active."));
      confirmationSoundAlert();
      solarAutoCalibration();
    } else {
      Serial.println(F(""));
      Serial.println(F("Operation mode is set to passive."));
      confirmationSoundAlert();
    }
    Serial.println(F(""));
  } else if (input == "7") {
    manualAdjustment();
  } else {
    Serial.println(F(""));
    Serial.println(F("Invalid option. Please select a valid option."));
    Serial.println(F(""));
    inputErrorSoundAlert();
  }

  displayMenu();
}

void adjustSystemTime() {
  Serial.println(F(""));
  Serial.println(F("Enter the new date and time in the format: YYYY MM DD HH MM SS"));
  while (Serial.available() == 0);
  String dateTimeInput = Serial.readStringUntil('\n');
  dateTimeInput.trim();
  int year, month, day, hour, minute, second;
  if (sscanf(dateTimeInput.c_str(), "%d %d %d %d %d %d", &year, &month, &day, &hour, &minute, &second) == 6) {
    rtc.adjust(DateTime(year, month, day, hour, minute, second));
    Serial.println(F("Date and time adjusted!"));
    confirmationSoundAlert();
  } else {
    Serial.println(F("Invalid input. Please follow the format: YYYY MM DD HH MM SS"));
    inputErrorSoundAlert();
  }
  Serial.println(F(""));
}

void adjustSunriseTime() {
  Serial.println(F(""));
  Serial.println(F("Enter the sunrise time in the format: HH MM"));
  while (Serial.available() == 0);
  String sunriseInput = Serial.readStringUntil('\n');
  sunriseInput.trim();
  int sunriseH, sunriseM;
  if (sscanf(sunriseInput.c_str(), "%d %d", &sunriseH, &sunriseM) == 2) {
    sunriseHour = sunriseH;
    sunriseMinute = sunriseM;
    saveTimingsToEEPROM();
    Serial.println(F("Sunrise time adjusted!"));
    confirmationSoundAlert();
  } else {
    Serial.println(F("Invalid input. Please follow the format: HH MM"));
    inputErrorSoundAlert();
  }
  Serial.println(F(""));
}

void adjustSunsetTime() {
  Serial.println(F(""));
  Serial.println(F("Enter the sunset time in the format: HH MM"));
  while (Serial.available() == 0);
  String sunsetInput = Serial.readStringUntil('\n');
  sunsetInput.trim();
  int sunsetH, sunsetM;
  if (sscanf(sunsetInput.c_str(), "%d %d", &sunsetH, &sunsetM) == 2) {
    sunsetHour = sunsetH;
    sunsetMinute = sunsetM;
    saveTimingsToEEPROM();
    Serial.println(F("Sunset time adjusted!"));
    confirmationSoundAlert();
  } else {
    Serial.println(F("Invalid input. Please follow the format: HH MM"));
    inputErrorSoundAlert();
  }
  Serial.println(F(""));
}

void manualAdjustment() {
  Serial.println(F(""));
  Serial.println(F("Enter the solar panel angle between 30° to 150°. If the input is invalid, the system will operate in automatic mode."));
  while (Serial.available() == 0);
  String angleInput = Serial.readStringUntil('\n');
  angleInput.trim();
  int angle = angleInput.toInt();
  if (angle >= 30 & angle <= 150) {
    manualMode = true;
    servoAngle = angle;
    servoMotor.write(servoAngle);
    confirmationSoundAlert();
    Serial.println(F("Angle manually set. The system is now in manual mode!"));
    Serial.println(F(""));
    orientationConfirmationSound();
  } else {
    Serial.println(F("Invalid input. Please enter a valid number."));
    Serial.println(F(""));
    Serial.println(F("The system is now in automatic mode!"));
    Serial.println(F(""));
    inputErrorSoundAlert();
    manualMode = false;
  }
}

void adjustMagneticDeclination() {
  Serial.println(F("Enter the magnetic declination value (in degrees):"));
  while (Serial.available() == 0);
  String declinationInput = Serial.readStringUntil('\n');
  declinationInput.trim();
  float declination = declinationInput.toFloat();
  if (declination != 0 || declinationInput == "0") {
    magneticDeclination = declination;
    saveMagneticDeclinationToEEPROM();
    Serial.println(F("Magnetic declination adjusted!"));
    Serial.println(F(""));
    orientationConfirmationSound();
  } else {
    Serial.println(F("Invalid input. Please enter a valid number."));
    Serial.println(F(""));
    inputErrorSoundAlert();
  }
}

void saveFilterStateToEEPROM() {
  EEPROM.write(1, suddenVariationFilterEnabled);
}

void loadFilterStateFromEEPROM() {
  suddenVariationFilterEnabled = EEPROM.read(1);
}

void saveTimingsToEEPROM() {
  EEPROM.write(2, sunriseHour);
  EEPROM.write(3, sunriseMinute);
  EEPROM.write(4, sunsetHour);
  EEPROM.write(5, sunsetMinute);
}

void loadTimingsFromEEPROM() {
  sunriseHour = EEPROM.read(2);
  sunriseMinute = EEPROM.read(3);
  sunsetHour = EEPROM.read(4);
  sunsetMinute = EEPROM.read(5);
}

void saveActiveOperationModeToEEPROM() {
  EEPROM.write(6, activeOperationMode);
}

void loadActiveOperationModeFromEEPROM() {
  activeOperationMode = EEPROM.read(6);
}

void saveMagneticDeclinationToEEPROM() {
  int declinationInt = magneticDeclination * 100;
  EEPROM.write(7, declinationInt >> 8);
  EEPROM.write(8, declinationInt & 0xFF);
}

void loadMagneticDeclinationFromEEPROM() {
  int declinationInt = (EEPROM.read(7) << 8) | EEPROM.read(8);
  magneticDeclination = declinationInt / 100.0;
}

void initialSoundAlert() {
  tone(buzzerPin, 1000, 200);
  delay(300);
  noTone(buzzerPin);
}

void compassCalibrationSoundAlert() {
  tone(buzzerPin, 8000, 500);
  delay(900);
  noTone(buzzerPin);
}

void servoMovementSoundAlert() {
  tone(buzzerPin, 1800, 200);
  delay(200);
  noTone(buzzerPin);
}

void confirmationSoundAlert() {
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

void orientationErrorSoundAlert() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 400, 100);
    delay(150);
    noTone(buzzerPin);
  }
}

void rtcErrorSoundAlert() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 300, 500);
    delay(250);
    noTone(buzzerPin);
  }
}

void compassErrorSoundAlert() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 250, 240);
    delay(450);
    noTone(buzzerPin);
  }
}

void inputErrorSoundAlert() {
  for (int i = 0; i < 2; i++) {
    tone(buzzerPin, 330, 140);
    delay(130);
    noTone(buzzerPin);
  }
}
