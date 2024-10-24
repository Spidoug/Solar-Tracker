# Solar-Tracker

![Uploading image.png…]()


This code implements a solar tracker system using an Arduino, which controls a servo motor to adjust the angle of a solar panel based on readings from two LDRs and an irradiance sensor. Additionally, there is a compass module (HMC5883) to correct the system's orientation and an RTC module (real-time clock) to manage the operating time (from sunrise to sunset).

The central logic includes active and passive operation modes, PID control to adjust the servo motor, and auto-calibration functionality to identify the best solar exposure angle. The EEPROM is used to store important settings such as schedules, magnetic declination, sudden variation filter state, and the operating mode.

Here are some highlights of the functionality:

Operation Modes:
Active Mode: Uses the LDRs to identify the best position based on the difference in luminosity between east and west. A PID controller adjusts the servo motor angle to align the solar panel.

Passive Mode: Gradually moves the panel throughout the day based on the time elapsed between sunrise and sunset.

Solar Auto-Calibration:
A servo angle sweep process to identify the point of maximum irradiance, dynamically adjusting the sweep time according to the irradiance variation.

Orientation Correction with Compass:
The compass module reading is corrected using the minimum and maximum values calibrated during initialization, along with the magnetic declination stored in the EEPROM.

Sound Notifications:
The system uses a buzzer to indicate events such as startup, errors, or successful calibrations and adjustments.

EEPROM:
Used to store and retrieve crucial information like schedules, filter state, and magnetic declination, ensuring the system retains its settings after restarts.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

The HTML code implements a control interface for the solar tracking system, allowing interaction with an Arduino through the Web Serial API (used to connect the browser to the device). Here are some important details:

Interface Components:
Serial Connection: Allows connecting and disconnecting the Arduino via USB, displaying the connection status.

Angle Adjustment: Enables manual input of an angle between 30° and 150° for the solar panel, with a button to activate automatic mode.

Time and Date Synchronization: Syncs the Arduino's date and time with the local computer.

Sunrise and Sunset Times: Allows adjusting these times that define the operation period of the tracker.

Magnetic Declination: Allows entering and sending the magnetic declination value, essential for correct compass orientation.

Tracking Mode and Sudden Variation Filter: Switches between active/passive modes of tracker operation and enables/disables the filter that smooths sudden irradiance variations.

Data Recording: Starts and stops data recording for auto-calibration (angle, irradiance, compass) and allows exporting the data as CSV.

Compass Display: Shows the current compass value in real-time.

Irradiance Chart: A bar chart correlating the solar panel angle with the measured irradiance.

Data Reading and Sending Functionality:
The interface reads data from the Arduino via serial communication and processes values such as angle, irradiance, and compass orientation.
The bar chart is updated in real-time, showing the relationship between the panel angle and irradiance.
Collected data can be exported in CSV format for further analysis.
API Used:
Web Serial API: Allows communication between the browser and serial devices (such as Arduino), essential for sending commands and receiving data without needing to reload the page.
