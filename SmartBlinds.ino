////////////////////////////////////////////////////////////////////////
// Automated Blinds system.  Uses a Light Dependent Resistor (LDR) and a
// Temperature sensor (TBD) to open or close the blinds.
// Motor type, configuration TBD.
// 
// Requirements:
//   1. When dark out, the blinds are closed. (night time - rotated down)
//   2. When in light, the blinds are open. (rotated about 1/2 way open)
//   3. When in light, if the temperature exceeds 80 degrees, the blinds
//      rotate fully up to block light.
//      Use a rolling 7 temperature average (approx 30 seconds) to avoid having
//      spikes change the blinds.
//   4. Battery operated, but perhaps needs to detect power source to accommodate
//      desire #1 below.
//   5. Solar Power recharges
// Desires
//   1. Transmit telemetry wirelessly, so that any issues can be diagnosed
//      from the ground.
//   2. Somehow be configurable without re-programming.
//      Ideas: USB connection that connects to Wifi?  USB connection that sets
//        the values?  Wifi itself?  Probably not, since that would require
//        continuous operation, and I want to save batteries.  Bluetooth?
////////////////////////////////////////////////////////////////////////

#include <LowPower.h>
#include <Servo.h>
#include <EEPROMLog.h>

// Used for connecting a breadboard prototype, with default serial.
//#define PROTO 1

// Now using the LM35DZ temperature sensor.
#define LM35DZ 1

#ifdef PROTO
#define SERIAL_DEBUG 1
#else
#define SERIAL_INSTALL 1
#include <SoftwareSerial.h>
SoftwareSerial portOne (0, 1);
#endif

const float AREF_Voltage = 5.0;

const int RUN_INTERVAL = 50;  // Run at 20 Hz

// Number of degrees per interval to move the servo.
// this value equates to 100 degrees per second, well within the limits of the
// servo, according to the documentation. (0.19 sec/60 degrees)
// Going much faster makes the blinds audibly loud, which I don't want.
const int MAX_SERVO_INTERVAL = 5;

// Delay for 10 seconds after power up.  Allows me to center the servo
// and then disconnect in order to attach/debug.
const int CENTERING_DELAY = 10000;
const int MOVE_DELAY = 3000;

////////////////////////////////////////////////////////////////////////
// Pins
// All the pins chosen are on the right-hand side of the Pro Mini, which
// allows me to move it far to the left in the installation.
// The Analog pins are ordered in the manner easiest to wire up the project.
////////////////////////////////////////////////////////////////////////
const int servoPin = 10;  // Servo Library only supports pin 9 and 10

const int transistorPin = 11; // high-side switch transistor
const int activityPin = 13;   // on-board LED

const int ldrInputPin = A3;  // LDR Sensor Pin

const int tempInputPin = A2; // Temperature Sensor Pin

const int panelPin = A0;     // panel voltage pin
const int batteryPin = A1;   // battery voltage pin


////////////////////////////////////////////////////////////////////////
// Servo Info
Servo angleServo;
int motorAngle = 0;

// flags/vars to handle the movement of the blinds
boolean takeAction = false;
int actionInProgress = 0;


////////////////////////////////////////////////////////////////////////
// Battery info
const float MINIMUM_VOLTAGE = 3.0;

// battery must increase by 0.3 volts before we log a low-battery condition again
const float RELOG_MINIMUM = 0.3;   

float batteryVoltage = 0.0;    // Battery voltage as measured by the processor
boolean voltsLogged = false; // indicates we logged a low-battery condition.

float panelVoltage = 0.0;      // Solar Panel voltage

////////////////////////////////////////////////////////////////////////
int lightValue = 0;  // analog reading

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned int logCount = 0;

const int CHANGE_LEVEL = 400;
const int LIGHT_DEADBAND = 150; 

const int SAMPLE_COUNT = 7;              // samples to average the temperature (~60 sec)
const float MAXIMUM_TEMPERATURE = 30.0;  // about 86 degrees F
const float TEMP_DEADBAND = 3.0;         // about 5 degrees F

unsigned int numSamples = 0;  // stops incrementing at SAMPLE_COUNT

// Motor moves at SERVO_INTERVAL between these values.
const int NIGHT_CLOSED = 00;
const int DAY_OPEN = 80;      // rotated open, downs slightly, to let light in
const int SERVO_CENTER = 90;
const int HEAT_CLOSED = 165;  // rotated closed, pointing up, to block light

// constants used for state and action
const int MOVING = 1;
const int PAUSED = 4;

// current state of the blinds
int motorState = DAY_OPEN;

int desiredAngle = NIGHT_CLOSED;
int currentAngle = SERVO_CENTER;


////////////////////////////////////////////////////////////////////////
// Logging data
// I'm using analog signals for the "float" values because it's easier
// to read on the serial output.  At a later date maybe I'll put some
// smarts into the reading of the data.  For now it's mostly for debugging
// purposes.  Maybe COSMOS would be a good option.
////////////////////////////////////////////////////////////////////////
struct logData_t
{
   unsigned int logCount;
   unsigned long theTime;       // time of the log entry
   int theAngle;                // angle that we're moving to
   int theLight;                // analog light level detected
   float theTemp;               // temperature (analog signal) 
   float batteryVolts;          // battery voltage
   float panelVolts;            // panel voltage
};

// Declare the EEPROM Logging class instantiation
byte PATTERN [2] = {0xFA, 0x01};
EEPROMLog<logData_t> dataLog (PATTERN);


////////////////////////////////////////////////////////////////////////
// Voltage offset at 0 degrees C
////////////////////////////////////////////////////////////////////////
#ifdef LM35DZ
const float VOLTS_OFFSET = 0.0;  // (LM35DZ)
#else
const float VOLTS_OFFSET = 0.5;  // (TMP36)
#endif

// 10 mV / 1 deg C
const float SCALE_FACTOR = 100.0;

// scale factor for all analog readings
const float ANALOG_SCALE = 1.0 / 1024.0 * AREF_Voltage;

////////////////////////////////////////////////////////////////////////
// Returns the voltage voltage from the analog value read in
////////////////////////////////////////////////////////////////////////
float getVolts (int value)
{
   return ((float) value) * ANALOG_SCALE;
}


////////////////////////////////////////////////////////////////////////
// Returns the temperature from the sensor in degrees C.
////////////////////////////////////////////////////////////////////////
float getTemperature (int value)
{
   return (((float) value) * ANALOG_SCALE - VOLTS_OFFSET) * SCALE_FACTOR;
}


int tempValue = 0; // global for logging

// stored temperature, in C
float temperature = 0.0;

////////////////////////////////////////////////////////////////////////
// Average the temperature
// Used to smooth out any spikes in the temperature readings
////////////////////////////////////////////////////////////////////////
float averageTemperature (float average, float newTemp)
{
   if (numSamples < SAMPLE_COUNT)
      numSamples++;
   
   average -= (average / numSamples);
   average += (newTemp / numSamples);

   return average;
}


////////////////////////////////////////////////////////////////////////
// Simple degrees C to degrees F conversion for display purposes
// while debugging.
////////////////////////////////////////////////////////////////////////
float CtoF (float temperature)
{
   return ((temperature * 9.0 / 5.0) + 32.0);
}


////////////////////////////////////////////////////////////////////////
// This function sleeps the computer forever in the event the reported
// battery voltage is below the safe minimum.
// The idea is to protect a Li-Ion battery from over discharge.
////////////////////////////////////////////////////////////////////////
void BatteryManagement ()
{
   int batteryValue = 0;
   int panelValue = 0;
   
   logData_t event;

   // batteryVoltage and panelVoltage are stored globally so that they
   // can be logged whenever the blinds change position.
   batteryValue = analogRead (batteryPin);
   batteryValue = analogRead (batteryPin);
   batteryVoltage = getVolts (batteryValue);

   panelValue = analogRead (panelPin);
   panelValue = analogRead (panelPin);
   panelVoltage = getVolts (panelValue);
            
#ifndef SERIAL_DEBUG
   // If we detect that the battery voltage has dropped below the
   // minimum, then log the information.
   if (batteryVoltage > 0.0)
   {
      if ((batteryVoltage <= MINIMUM_VOLTAGE) && !voltsLogged)
      {
         // attempt to capture everything
         logCount++;
         event.logCount = logCount;
         event.theTime = millis ();
         event.theAngle = desiredAngle;
         event.theLight = lightValue;
         event.theTemp = temperature;
         event.batteryVolts = batteryVoltage;
         event.panelVolts = panelVoltage;
         voltsLogged = true;
         dataLog.Write (event);
      }
      else if (batteryVoltage > MINIMUM_VOLTAGE + RELOG_MINIMUM)
      {
         // reset the logging mechanism
         voltsLogged = false;
      }
      
   }
#endif
}


////////////////////////////////////////////////////////////////////////
// reportEntries
////////////////////////////////////////////////////////////////////////
void reportEntries ()
{
   logData_t entry;

   char tempC_str [7];
   char battV_str [6];
   char panelV_str [6];

   char buffer [80];

   int count = 0;
   
   // Print out on the serial port all the entries found in EEPROM
   while (dataLog.Read (entry))
   {
      dtostrf (entry.theTemp, 5, 2, tempC_str);
      dtostrf (entry.batteryVolts, 4, 2, battV_str);
      dtostrf (entry.panelVolts, 4, 2, panelV_str);
      count++;
      sprintf (buffer, "Entry # %d \tCount %d \tTime (ms) %lu \tAngle %d \tLight %d \tTemp %s C \tBattery %s V, Panel %s V",
               count, entry.logCount, entry.theTime, entry.theAngle, entry.theLight,
               tempC_str, battV_str, panelV_str);

#if SERIAL_DEBUG
      Serial.println (buffer);
#elif defined SERIAL_INSTALL
      portOne.println (buffer);
#endif

   }

}
   
////////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////////
void setup()
{
   // attach the servo and center it to start.
   angleServo.attach (servoPin);
   currentAngle = SERVO_CENTER;

   // The transister pin connected to the motor
   // must be set high, to allow the motor to run.
   pinMode (transistorPin, OUTPUT);
   pinMode (activityPin, OUTPUT);
            
   angleServo.write (currentAngle);

   // Printout the existing EEPROM entries (useful for debugging)
#if defined SERIAL_DEBUG 
   Serial.begin (9600);
   reportEntries ();
#elif defined SERIAL_INSTALL
   portOne.begin (9600);
   reportEntries ();
#endif
   
}

////////////////////////////////////////////////////////////////////////
// Compute detrmines whether action needs to be taken according
// to the light level, and sets the direction indication for the
// motor control.
////////////////////////////////////////////////////////////////////////
boolean Compute (int light, float theTemp)
{
   boolean actionNeeded = false;
   logData_t event;

   switch (motorState)
   {
      
      ////////////////////////////////////////////////////////////////////////
      case PAUSED:
         // If the light level is below the DARK level, then close the blinds
         // Or if it gets too hot, close the blinds.
         if (light < (CHANGE_LEVEL - LIGHT_DEADBAND))
         {
            desiredAngle = NIGHT_CLOSED;
         }
         else
         {
            if (theTemp > MAXIMUM_TEMPERATURE)
            {
               desiredAngle = HEAT_CLOSED;
            }
            else
            {
               desiredAngle = DAY_OPEN;
            }
         }
         actionNeeded = true;
         angleServo.attach (servoPin);
         digitalWrite (transistorPin, LOW);
         break;

         
      ////////////////////////////////////////////////////////////////////////
      case DAY_OPEN:
         
         // if light falls below the change level, then close the blinds.
         // if it gets hot, then send them to their heat closed position
         if (light < (CHANGE_LEVEL - LIGHT_DEADBAND))
         {
            desiredAngle = NIGHT_CLOSED;
         }
         else if (theTemp > MAXIMUM_TEMPERATURE)
         {
            desiredAngle = HEAT_CLOSED;
         }
         break;

         
      ////////////////////////////////////////////////////////////////////////
      case HEAT_CLOSED:

         // if the heat drops below the threshold, then open the blinds
         // it's possible this doesn't happen until after the sun goes down
         if (theTemp < (MAXIMUM_TEMPERATURE - TEMP_DEADBAND))
         {
            if (light > CHANGE_LEVEL + LIGHT_DEADBAND)
            {
               desiredAngle = DAY_OPEN;
            }
            else
            {
               desiredAngle = NIGHT_CLOSED;
            }
         }
         else if (light < (CHANGE_LEVEL - LIGHT_DEADBAND))
         {
            desiredAngle = NIGHT_CLOSED;
         }
         break;
                  
      
      ////////////////////////////////////////////////////////////////////////
      case MOVING:
         
         if (currentAngle == desiredAngle)
         {
            motorState = desiredAngle;
            actionNeeded = false;  // re-assert here to be clear...
         }
         else if (currentAngle > desiredAngle)
         {
            currentAngle = currentAngle - MAX_SERVO_INTERVAL;
            if (currentAngle <= desiredAngle)
            {
               // limit the angle commanded to the desired...
               currentAngle = desiredAngle;
            }
            actionNeeded = true;

         }
         else if (currentAngle < desiredAngle)
         {
            currentAngle = currentAngle + MAX_SERVO_INTERVAL;
            if (currentAngle >= desiredAngle)
            {
               // limit the angle commanded to the desired...
               currentAngle = desiredAngle;
            }
            actionNeeded = true;
         }

         // If we need to take action, write the servo value.
         // If no action is to be taken, then we're done, so detach the servo.
         if (actionNeeded)
         {
            angleServo.write (currentAngle);
         }
         else
         {
            angleServo.detach ();
         }
         break;

         
      ////////////////////////////////////////////////////////////////////////
      case NIGHT_CLOSED:
         // if it's daylight out, then we may want to open the blinds.  However,
         // if the temperature is above MAXIMUM, then it's too hot, we want to
         // close the blinds.
         if (light > (CHANGE_LEVEL + LIGHT_DEADBAND))
         {
            // it's possible that it's too hot by the time the blinds open.
            // If that's the case, move straight to the HEAT_CLOSED position
            if (theTemp > MAXIMUM_TEMPERATURE)
            {
               desiredAngle = HEAT_CLOSED;
            }
            else
            {
               desiredAngle = DAY_OPEN;
            }
         }
         break;
         
   }


   // If the desiredAngle and currentAngle are different, we need to move
   // or continue moving.
   if ((desiredAngle != currentAngle) && (motorState != MOVING))
   {
      angleServo.attach (servoPin);
      
      // If the angle is not the same, the high-side switch transister pin 
      // must be set low, to allow the motor to run.
      digitalWrite (transistorPin, LOW);
         
      // change the motor state to indicate we need to move.  The next pass
      // will change the commanded angle.
      motorState = MOVING;

      actionNeeded = true;

         
      logCount++;
         
      event.logCount = logCount;
      event.theTime = millis ();
      event.theAngle = desiredAngle;
      event.theLight = light;
      event.theTemp = theTemp;
      event.batteryVolts = batteryVoltage;
      event.panelVolts = panelVoltage;

      dataLog.Write (event);
   }

   return actionNeeded;
}

////////////////////////////////////////////////////////////////////////
// Main loop
////////////////////////////////////////////////////////////////////////

void loop()
{   
#if defined SERIAL_DEBUG || SERIAL_INSTALL
   char outputBuffer [80];
   char tempC_str [7];
   char tempF_str [7];
   char battV_str [6];
   char panelV_str [6];
#endif
   
   currentTime = millis ();

   if (currentTime < CENTERING_DELAY)
   {
      if (currentTime < MOVE_DELAY)
      {
         // enable motor movement
         digitalWrite (transistorPin, LOW);
         digitalWrite (activityPin, HIGH);
         angleServo.write (currentAngle);
         motorState = PAUSED;
      }
      else
      {
         // don't need to drive the motor the whole time
         digitalWrite (transistorPin, HIGH);
         digitalWrite (activityPin, LOW);
         angleServo.detach ();
      }
   }
   else
   {
      // put your main code here, to run repeatedly:
      if (currentTime > previousTime + RUN_INTERVAL)
      {
         if (motorState != MOVING)
         {
            // Read the LDR and temperature sensors
            // Don't read the sensors while moving the motor - it throws
            // the values off (at least for temperature)
            // analogRead sometimes needs a delay to get accurate values,
            lightValue = analogRead (ldrInputPin);
            lightValue = analogRead (ldrInputPin);

            tempValue = analogRead (tempInputPin);
            tempValue = analogRead (tempInputPin);
            
            temperature = averageTemperature (temperature, getTemperature (tempValue));

            BatteryManagement ();
         
#if defined SERIAL_DEBUG || SERIAL_INSTALL
            // create an output string for debugging purposes.
            dtostrf (temperature, 5, 2, tempC_str);
            dtostrf (CtoF (temperature), 5, 2, tempF_str);
            dtostrf (batteryVoltage, 4, 2, battV_str);
            dtostrf (panelVoltage, 4, 2, panelV_str);
            sprintf (outputBuffer, "time = %lu motorval = %d light = %d temp = %s C %s F BV = %s V PV = %s V",
                     currentTime, motorState, lightValue, tempC_str, tempF_str, battV_str, panelV_str);
#endif
#ifdef SERIAL_DEBUG
            Serial.println (outputBuffer);
#elif SERIAL_INSTALL
            portOne.println (outputBuffer);
#endif
         }

         // Compute whether action needs to be taken, and which direction.
         takeAction = Compute (lightValue, temperature);

         previousTime = currentTime;

         // Only go to sleep if the motor is done moving and there's no action
         // to take.  If there is action, or the motor is moving, we want to
         // complete the event before we go to sleep.
         if (!takeAction && (motorState != MOVING))
         {
            // Kill the high-side switch to eliminate current draw from the servo
            digitalWrite (transistorPin, HIGH);

            digitalWrite (activityPin, LOW);
#ifdef SERIAL_DEBUG
            delay (1000);
#else
            // delay (1000);
            // Due to the use of the sleep function, only about 194 "milliseconds" elapse
            // between cycles.  The sleep function disables Timer 1, which is the
            // millisecond timer.
            LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);
#endif
            digitalWrite (activityPin, HIGH);
         }
         
      } // currentTime > previousTime + RUN_INTERVAL
      
   } // currentTime > CENTERING_DELAY
}
