# Solar-Tracker

![Solar tracker](https://github.com/user-attachments/assets/14e7e882-e4fa-4f13-8470-cbe85fea057b)

This code implements a solar tracker system using an Arduino, which controls a servo motor to adjust the angle of a solar panel based on readings from two LDRs and an irradiance sensor. Additionally, there is a compass module (HMC5883) to correct the system's orientation and an RTC module (real-time clock) to manage the operating time (from sunrise to sunset).

The central logic includes active and passive operation modes, PID control to adjust the servo motor, and auto-calibration functionality to identify the best solar exposure angle. The EEPROM is used to store important settings such as schedules, magnetic declination, sudden variation filter state, and the operating mode.

![Circuit](https://github.com/user-attachments/assets/d62eb5dc-f63b-43ab-a22d-1e1633ed56ea)

1. Project Objective

The solar tracking system is designed to maximize solar energy capture by orienting a photovoltaic panel at the optimal angle throughout the day. Utilizing LDR sensors, a compass module, a servo motor, an RTC (Real-Time Clock), and EEPROM for parameter storage, the system ensures precise alignment to the sun and adapts to lighting conditions, providing precise and automated control with a calibration mechanism and manual adjustments.

2. General Structure and Operation

The system is structured with a set of functions and variables to manage and monitor the components of the solar tracker. Below are the main modules and operational flows of the code:

Initialization: Loads configuration data from the EEPROM, sets up the RTC and compass module, and starts the servo motor in a neutral position.
Servo Motor Control: Based on readings from LDR sensors (located to the east and west), the system calculates the error between readings to adjust the panel angle towards the most intense light source.
Calibration and Auto-Adjustment: An auto-calibration function seeks the angle of maximum irradiance, while the compass orientation ensures north-south alignment of the panel.
Operating Modes: Includes active and passive solar tracking modes, as well as a manual mode for independent angle setting.
Storage and Retrieval: The EEPROM saves magnetic declination settings, operation times, and filter status to ensure configuration persistence.

3. Key Components and Functions

RTC (Real-Time Clock): Used to determine sunrise and sunset times, activating the system only during solar incidence periods.
Compass (HMC5883L): Provides magnetic orientation, allowing the panel to precisely align on the north-south axis.
Servo Motor: Responsible for adjusting the solar panel to the optimal light angle, with limits set to avoid overload.
EEPROM: Enables the persistence of configurable parameters, such as magnetic declination, filter status, and operation times.
LDR Sensors: Aid in detecting the direction with the highest solar incidence, orienting the panel based on the light difference between sensors.

4. Positive Points and Relevant Aspects

Dynamic Calibration: The auto-calibration function adjusts the angle automatically based on irradiance, keeping the panel in the ideal position.
Time and EEPROM Configurations: By saving data in the EEPROM, the system preserves settings even after being powered off, eliminating the need for manual reconfiguration.
Error Detection and Sound Alerts: The system includes sound alerts for different situations, such as misalignment and orientation errors, improving interactivity and monitoring.
Flexible Operating Modes: The ability to operate in active, passive, and manual modes adds flexibility and allows adaptation to different usage scenarios.

5. Applications and Potential Expansions

This project can be used in various solar energy applications requiring cost-effective and high-efficiency solar tracking. Some of the main applications include:

Small photovoltaic installations for energy capture in homes or farms.
Self-sustaining energy systems, such as charging stations for electronic devices in remote areas.
Solar efficiency studies in universities and research centers to optimize solar energy use.
Potential Expansions:

Integration with meteorological sensors for adjustment in cloudy conditions.
Implementation of a stepper motor with lead screws, replacing the servo for more efficient movement in larger panels.

6. Conclusion

The code demonstrates a robust implementation of a solar tracking system, offering flexible configurations, persistent storage, and good precision control. With some improvements, particularly in movement logic and module communication, the system could achieve even more efficient and reliable performance, ensuring optimal panel alignment and maximizing solar energy capture.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

![App](https://github.com/user-attachments/assets/cdbe6c58-a20d-438f-949f-b8ca73dcf23d)

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
