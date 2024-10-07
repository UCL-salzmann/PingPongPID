// Program for running the PingPongPID
// Serial commands:
// @BH XX sets the baseline distance in cm. Values between 0 and 100cm are meaningful for XX.
// @KP XX sets the Kp constant as XX.
// @KI XX sets the Ki constant as XX.
// @KD XX sets the Kd constant as XX.
// @TP XX sets the time period of one oscillation as XX
// @AMP XX sets the amplitude of oscillation in XX cm
// @PM returns the current values of Kp, Ki, Kd and baseline distance, amplitude and time period.
// @CS checks the distance sensor and returns its status.
// @DI returns the current distance.
// @GO starts a run based on the defined PID parameters. During a run, the runtime at about 200 millisecond
//     intervals, the current distance in cm, the setpoint distance in cm and the current output voltage will be
//     returned as semicolon-separated values which is convenient for saving as a CSV file.
// @STOP stops an active run.

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <math.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_seesaw.h"     // for rotary encoder

#define DEV_I2C Wire

VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);
uint8_t NewDataReady = 0;
VL53L4CD_Result_t results;
uint8_t status;
char report[64];

Adafruit_7segment display1 = Adafruit_7segment();
Adafruit_7segment display2 = Adafruit_7segment();

double Setpoint, Input, Output, Distance;
int displayheight, olddisplayheight;
double Kp, Ki, Kd;
float Vmin, Vmax, off;
boolean active, buttonmode;
unsigned long starttime, currtime;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

String serialstring;

double baselineHeight; //sets the baseline height for the pingpong ball
double amplitude; // sets the amplitude
double period; //sets the frequency of 1 oscillation

#define SS_SWITCH        24
#define SEESAW_ADDR          0x36
Adafruit_seesaw ss;
int32_t encoder_position;

void displaytest() {
  display1.writeDigitNum(0, 8, true);
  display1.writeDigitNum(1, 8, true);
  display1.writeDigitNum(3, 8, true);
  display1.writeDigitNum(4, 8, true);
  display1.writeDisplay();
  display2.writeDigitNum(0, 8, true);
  display2.writeDigitNum(1, 8, true);
  display2.writeDigitNum(3, 8, true);
  display2.writeDigitNum(4, 8, true);
  display2.writeDisplay();
  delay(2000);
}

void setup()
{
  // Initialize serial for output.
  Serial.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();
  // Configure VL53L4CD satellite component.
  sensor_vl53l4cd_sat.begin();
  // Switch off VL53L4CD satellite component.
  sensor_vl53l4cd_sat.VL53L4CD_Off();
  //Initialize VL53L4CD satellite component.
  sensor_vl53l4cd_sat.InitSensor();
  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);
  // Start Measurements
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
  olddisplayheight = -100;

  // Connect to powersupply, turn output on, deactivate front panel and set voltage to zero.
  Serial1.begin(9600);
  Serial1.println("OUT1");
  Serial1.println("LOCK1");
  Serial1.print("VSET1:");
  Serial1.println("0.00");

  // Default parameters for PID controller
  Vmax = 11.5;
  Vmin = 6.5;
  Kp = 0.05;
  Ki = 0.05;
  Kd = 0.002;
  off = 4.6;         // distance offset between sensor and resting position of ball. May need to change depending on the setup.
  baselineHeight = 80;
  amplitude = 0;
  period = 10;
  active = false;
  buttonmode = false;

  // Initialise PID controller.
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(Vmin, Vmax);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetSampleTime(100);

  // Activate displays.
  display1.begin(0x70);
  display2.begin(0x71);
  display1.setBrightness(12);
  display2.setBrightness(15);
  displaytest();

  display2.print(baselineHeight, 0);
  display2.writeDisplay();

  // Initialise rotary encoder.
  ss.begin(SEESAW_ADDR);
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);
  encoder_position = ss.getEncoderPosition();
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();
  encoder_position = 0;
}

void calcSetpoint() {
  currtime = millis() - starttime;
  float currsec = float(currtime) / 1000;
  Setpoint = baselineHeight + amplitude * sin(2 * PI * (1 / period) * currsec);
}

void loop() {

  if (active == true) {
    calcSetpoint();
    display2.print(Setpoint, 0);
    display2.writeDisplay();
  }

  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  int32_t new_position = ss.getEncoderPosition();
  if (encoder_position != new_position) {
    baselineHeight = baselineHeight + (encoder_position - new_position);
    if (baselineHeight > 100) baselineHeight = 100;
    if (baselineHeight < 0) baselineHeight = 0;
    calcSetpoint();
    encoder_position = new_position;
    display2.print(Setpoint, 0);
    display2.writeDisplay();
  }

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance.
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);

    Distance = (float(results.distance_mm) / 10) - off;
    if (Distance < 0) Distance = 0;
    Input = Distance;
    displayheight = round(Distance);

    // Write height to the display but only if it has changed.
    if (displayheight != olddisplayheight) {
      display1.print(displayheight);
      display1.writeDisplay();
      olddisplayheight = displayheight;
    }

    if (active == true) {
      if (results.range_status == 0) {
        myPID.Compute();
        Serial1.print("VSET1:");
        Serial1.println(Output, 2);

        currtime = millis() - starttime;
        float currsec = float(currtime) / 1000;
        if (buttonmode == false) {
          Serial.print(currsec, 3);
          Serial.print(";");
          Serial.print(Distance, 1);
          Serial.print(";");
          Serial.print(Setpoint, 1);
          Serial.print(";");
          Serial.println(Output, 2);
        }
      }
    }
  }

  // Check for button press
  uint32_t buttons = ss.digitalReadBulk((uint32_t)1 << SS_SWITCH);
  bool button_pressed = !(buttons & (1 << SS_SWITCH));
  static bool last_button_state = false;

  if (button_pressed && !last_button_state) { // Button pressed
    active = !active; // Toggle active state
    buttonmode = !buttonmode;
    if (active == true) {
      myPID.SetOutputLimits(0.0, 1.0);    // These lines are needed to delete the previous integral errors.
      myPID.SetOutputLimits(-1.0, 0.0);
      myPID.SetOutputLimits(Vmin, Vmax);
      display1.writeDigitRaw(0, 0b00111101); // Showing "GO" on the left blue display to show that the program has started.
      display1.writeDigitRaw(1, 0b00111111);
      display1.writeDigitRaw(3, 0b00000000);
      display1.writeDigitRaw(4, 0b00000000);
      display1.writeDisplay();
      delay(500); // Adding a 1/2 second delay (500 milliseconds) so that the people can see the message pop-up.
      starttime = millis(); // Reset start time

    } else {
      display1.writeDigitRaw(0, 0b1101101); // Showing "STOP" on the left blue display to show that the program has ended.
      display1.writeDigitRaw(1, 0b1111000);
      display1.writeDigitRaw(3, 0b0111111);
      display1.writeDigitRaw(4, 0b1110011);
      display1.writeDisplay();
      delay(500); // Adding a 1/2 second delay (500 milliseconds) so that people can see the message pop-up.
      Serial1.print("VSET1:");
      Serial1.println(0.00); // Stop the output
    }
    delay(200); // Debounce delay
  }
  last_button_state = button_pressed;

  // Serial communication with PC
  if (Serial.available() > 0) {
    serialstring = Serial.readStringUntil('\n\r');
    serialstring.trim();

    if (serialstring.startsWith("@")) {
      serialstring.remove(0, 1);
      if (serialstring.startsWith("GO")) {
        active = true;
        myPID.SetOutputLimits(0.0, 1.0);    // These lines are needed to delete the previous integral errors.
        myPID.SetOutputLimits(-1.0, 0.0);   //
        myPID.SetOutputLimits(Vmin, Vmax);  //
        starttime = millis();
        buttonmode = false;
      }
      if (serialstring.startsWith("STOP")) {
        active = false;
        buttonmode = false;
        Serial1.print("VSET1:");
        Serial1.println(0.00);
      }
      if (serialstring.startsWith("TP")) {
        serialstring.remove(0, 2);
        period = serialstring.toFloat();
        if (period == 0) {
          period = 1;
        }
      }
      if (serialstring.startsWith("BH")) {
        serialstring.remove(0, 2);
        baselineHeight = serialstring.toFloat();
        if (baselineHeight > 100) baselineHeight = 100;
        if (baselineHeight < 0) baselineHeight = 0;
        display2.print(baselineHeight, 0);
        display2.writeDisplay();
        if (baselineHeight + amplitude > 100) {
          amplitude = 100 - baselineHeight;// ensures the baseline height + amplitude does not exceed 100
        } else if (baselineHeight - amplitude < 0) {
          amplitude = baselineHeight; // ensures the baseline height - amplitude does not go below 0
        }
      }
      if (serialstring.startsWith("KP")) {
        serialstring.remove(0, 2);
        Kp = serialstring.toFloat();
        myPID.SetTunings(Kp, Ki, Kd);
      }
      if (serialstring.startsWith("KI")) {
        serialstring.remove(0, 2);
        Ki = serialstring.toFloat();
        myPID.SetTunings(Kp, Ki, Kd);
      }
      if (serialstring.startsWith("KD")) {
        serialstring.remove(0, 2);
        Kd = serialstring.toFloat();
        myPID.SetTunings(Kp, Ki, Kd);
      }
      if (serialstring.startsWith("AMP")) {
        serialstring.remove(0, 3);
        amplitude = serialstring.toFloat();
      }
      if (serialstring.startsWith("PM")) {
        Serial.print("Kp: ");
        Serial.print(Kp, 6);
        Serial.print(", Ki: ");
        Serial.print(Ki, 6);
        Serial.print(", Kd: ");
        Serial.print(Kd, 6);
        Serial.print(", Baseline: ");
        Serial.print(baselineHeight, 1);
        Serial.print(", Amplitude: ");
        Serial.print(amplitude, 1);
        Serial.print(", Time Period: ");
        Serial.println(period, 1);
      }
      if (serialstring.startsWith("CS")) {
        if (results.range_status == 0) {
          Serial.println("Distance sensor is okay.");
        } else {
          Serial.println("Problem with distance sensor.");
        }
      }
      if (serialstring.startsWith("DI")) {
        Serial.print("Distance: ");
        Serial.println(Distance, 1);
      }
    }
  }
}
