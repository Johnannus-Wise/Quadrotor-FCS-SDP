#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

// ===== Wi-Fi credentials =====
const char *ssid = "Hakeem_2.4GHz";
// const char *ssid = "Honor 8X Max";
const char *password = "19641964";

// Definitions and variables for pressure sensor
#define BMP388ADDRESS 0x76 // BMP388 i2c address
#define sdaPin 21          // data pin
#define sclPin 22          // clock pin
#define i2cSpeed 1000000   // i2c transmission speed
#define P0 101325          // AVG Sea Level Pressure
#define InitSampleSize 100 // Initial Sample Size

uint8_t altCounter = 0;

float PAR_T1, PAR_T2, PAR_T3;
float PAR_P1, PAR_P2, PAR_P3;
float PAR_P4, PAR_P5, PAR_P6;
float PAR_P7, PAR_P8, PAR_P9;
float PAR_P10, PAR_P11;

uint32_t rawPressure, rawTemp;

float pressure, temp, altitude, hoverAltitude, initialTemp, initialPressure, altitude1;
bool altitudeLock;
// Definisions and variables for the IMU

#define ICMAddress 0x68
#define accelSensitivity 8192
#define gyroSensitivity 65.536

int16_t rawGyroX, rawGyroY, rawGyroZ;
int16_t rawAccelX, rawAccelY, rawAccelZ;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float accelPitch = 0.0f, accelRoll = 0.0f;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

float alphaIMU = 0.99;

// Definisions and classes for the receiver
#define CRSF_Addr 0xC8            // crossfire protocol address
#define CRSF_Sensor 0xEA          // crossfire protocol sensor
#define CRSF_Type_Battery 0x08    // crossfire protocol battery packet
#define CRSF_Type_Altitude 0x09   // crossfire protocol altitude packet
#define CRSF_Type_Channels 0x16   // crossfire protocol channels packet
#define CRSF_Type_Attitude 0x1E   // crossfire protocol attitude packet
#define CRSF_Length_Channels 0x18 // crossfire protocol channels packet length (in bytes, excluding the address and type bytes)
#define CRSF_Length_Altitude 0x6  // crossfire protocol altitude packet length (in bytes, excluding the address and type bytes)
// for the UART communication with the receiver
#define Rx 16
#define Tx 17

// baud rate of the UART with the receiver
#define bRate 416666
// #define bRate 420000
// #define bRate 1872000

uint8_t channelsPacket[26], data0[22], crc8;

// General Definisions
#define frontLeftPin 25  // Yellow LED on the left (yellow wire)
#define frontRightPin 33 // Green LED on the front (green wire)
#define rearRightPin 32  // Yellow LED on the right (orange wire)
#define rearLeftPin 26   // Red LED on the rear (red wire)
#define frontLeft 1      // Yellow LED on the left (yellow wire)
#define frontRight 2     // Green LED on the front (green wire)
#define rearRight 3      // Yellow LED on the right (orange wire)
#define rearLeft 4       // Red LED on the rear (red wire)
#define PWMFrequency 500
#define PWMResolution 11
#define PWMRange pow(2.0, PWMResolution)
#define signalMax 1811
#define signalMin 174
#define tiltAngleMax 25
#define yawSpeedMax 90
#define minAltitude 0
#define maxAltitude 50
#define maxThrottleValue 2047
#define minThrottleValue 1023

#define batteryADCPin 36 // ADC1 pin
#define R1 10000
#define R2 1980
#define resistanceRatio (R1 + R2) / R2
#define Vref 2.8                 // voltage reference at max battery
#define maxBatteryVoltage 16.8   // Volts
#define maxBatteryCapacity 10400 // mAh

int16_t rawBatteryReadings = 0;
float measuredVoltage = 0.0, batteryVoltage = 0.0, batteryPercentUsed = 0.0;
uint32_t batteryCapacityRemaining = 0;
uint8_t batteryRemainingPercent = 0;
//--------------------------------------------------------------------------------------------------------------------
// Structure for the PID controller
//--------------------------------------------------------------------------------------------------------------------
struct PIDController
{
  // --- PID gains ---
  float Kp = 0, Ki = 0, Kd = 0;

  // --- Configuration ---
  float derivativeFilterAlpha = 0.95;
  float tolerance;
  float integralMax, integralMin;

  // --- Runtime Variables ---
  float error = 0;
  // reference or desired state, the feedback value, and previous feedback value
  float reference = 0, feedbackState = 0, previousFeedbackState = 0;
  // used to store the integral, raw derivative, and filtered derivative
  float integralTerm = 0, derivativeTermRaw = 0, derivativeTermFiltered = 0; // used to store integral and derivative
  // defines a starting point for when to begin computing the integral (very specific use case for this project, not a standard of PID controllers)
  float integralStartPoint = 0.55;
  // used to store the final PID sum
  float PID_output = 0;
  // used to store the max actuator signal
  float maxActuatorOutput = maxThrottleValue;

  PIDController(float kp, float ki, float kd, float iStPt, float tol)
  {
    Kp = kp;
    Ki = ki;
    Kd = kd;

    integralStartPoint = iStPt;

    tolerance = tol;
  }

  float compute(float reference, float feedback, float dt, float currentActuatorInput)
  {
    error = reference - feedback;
    // tolerance check
    if (abs(error) > tolerance)
    {
      // compute the raw derivative
      derivativeTermRaw = -(feedback - previousFeedbackState) / dt;
      // compute the filtered derivative
      derivativeTermFiltered = derivativeFilterAlpha * derivativeTermFiltered + (1.0 - derivativeFilterAlpha) * derivativeTermRaw;

      // condition to only compute integral when the main throttle is above the start point (default is 10%)
      if (currentActuatorInput > (maxActuatorOutput * integralStartPoint))
      {
        // compute the integral
        integralTerm = integralTerm + error * dt;
      }

      PID_output = (Kp * error) + (Ki * integralTerm) + (Kd * derivativeTermFiltered);
    }
    // storing the feedback for future derivative computations
    previousFeedbackState = feedback;
    return PID_output;
  }

  // --- Resets the controller
  void reset()
  {
    error = 0;
    integralTerm = 0;
    derivativeTermRaw = 0;
    derivativeTermFiltered = 0;
    PID_output = 0;
  }

  float unwind(float dt)
  {
    integralTerm = integralTerm - error * dt;
    return PID_output = (Kp * error) + (Ki * integralTerm) + (Kd * derivativeTermFiltered);
  }
};

// control loop refrences (main input)
int desiredPitch = 0, desiredRoll = 0, yawSpeed = 0, desiredAltitude = 0;
// used to compute the time
float previousTime = 0, deltaTime = 0, currentTime = 0;
// PID result for roll and pitch angles
float rollAnglePID = 0, pitchAnglePID = 0;
// PID result for roll, pitch, and yaw rates.
float rollRatePID = 0, pitchRatePID = 0, yawRatePID = 0;
// PID for altitude
float altitudePID = 0;

int mainThrottleInput = 0;
// Motor matrix, for F1 to F4, F1 is front left, F2 is front right, F3 is rear right, F4 is rear left.
int motorMatrix[4];

//--------------------------------------------------------------------------------------------------------------------
// Structures for the data channels
//--------------------------------------------------------------------------------------------------------------------
struct Bit11
{
  unsigned data : 11;
};

struct Channels
{
  Bit11 ch[16];
  // Variables needed to store the slope and intercept needed to calculate the rescaled functions
  float throttleSlope, throttleIntercept;
  float angleSlope, angleIntercept, yawSlope, yawIntercept, altitudeSlope, altitudeIntercept;
  // uint16_t ch[16];
  // Constructor
  Channels(int angleMin, int angleMax, int yawMin, int yawMax, int altitudeMin, int altitudeMax)
  {
    angleSlope = (float)(angleMax - angleMin) / (signalMax - signalMin);
    angleIntercept = (float)(angleMin - (angleSlope * signalMin));
    yawSlope = (float)(yawMax - yawMin) / (signalMax - signalMin);
    yawIntercept = (float)(yawMin - (yawSlope * signalMin));
    altitudeSlope = (float)(altitudeMax - altitudeMin) / (signalMax - signalMin);
    altitudeIntercept = (float)(altitudeMin - (altitudeSlope * signalMin));
  }

  // Setter for channel readings that takes the array of the receiver data as an argument
  void setChannelReadings(uint8_t refData[22])
  {
    ch[0].data = refData[0] | (refData[1] << 8);                                 // 5 bits remain
    ch[1].data = (refData[1] >> 3) | (refData[2] << 5);                          // 2 bits remain
    ch[2].data = ((refData[2] >> 6) | (refData[3] << 2) | (refData[4] << 10));   // 7 bits remain
    ch[3].data = (refData[4] >> 1) | (refData[5] << 7);                          // 4 bits remain
    ch[4].data = (refData[5] >> 4) | (refData[6] << 4);                          // 1 bit remains
    ch[5].data = (refData[6] >> 7) | (refData[7] << 1) | (refData[8] << 9);      // 6 bits remain
    ch[6].data = (refData[8] >> 2) | (refData[9] << 6);                          // 3 bits remain
    ch[7].data = (refData[9] >> 5) | (refData[10] << 3);                         // 0 remain
    ch[8].data = (refData[11]) | (refData[12] << 8);                             // 5 bits remain
    ch[9].data = (refData[12] >> 3) | (refData[13] << 5);                        // 2 bits remain
    ch[10].data = (refData[13] >> 6) | (refData[14] << 2) | (refData[15] << 10); // 7 bits remain
    ch[11].data = (refData[15] >> 1) | (refData[16] << 7);                       // 4 bits remain
    ch[12].data = (refData[16] >> 4) | (refData[17] << 4);                       // 1 bit remains
    ch[13].data = (refData[17] >> 7) | (refData[18] << 1) | (refData[19] << 9);  // 6 bits remain
    ch[14].data = (refData[19] >> 2) | (refData[20] << 6);                       // 3 bits remain
    ch[15].data = (refData[20] >> 5) | (refData[21] << 3);                       // 0 remain
    rescale();                                                                   // rescales the raw data into a simpler format and range
    // displayReadings();
  }

  void rescale()
  {
    // time1 = micros();
    mainThrottleInput = (int)(throttleSlope * ch[2].data + throttleIntercept); // computes the desired motor speed
    /* if the mainThrottle is greater than 60% of the max throttle (20% motor duty cycle)
    then it can start computing the desired angles, otherwise desired angles are 0. */
    if (mainThrottleInput > 0.6 * maxThrottleValue)
    {
      desiredRoll = angleSlope * ch[0].data + angleIntercept;  // computes the desired roll
      desiredPitch = angleSlope * ch[1].data + angleIntercept; // computes the desired pitch
    }
    else
    {
      desiredRoll = 0;
      desiredPitch = 0;
    }
    yawSpeed = yawSlope * ch[3].data + yawIntercept;
    desiredAltitude = altitudeSlope * ch[2].data + altitudeIntercept;
    // Serial.printf("Yaw Input: %i, desired pitch: %i, desired roll: %i\n", yawSpeed, desiredPitch, desiredRoll);
    // time2 = micros() - time1;
    // Serial.printf("Time taken: %li ms\n", time2);
  }

  // calculates the duty cycle for the PWM signal used to operate the ESCs
  void setDutyCycle(int desiredFrequency, int desiredResolution)
  {
    if (desiredFrequency > 500)
    {
      desiredFrequency = 500;
    }
    else if (desiredFrequency < 50)
    {
      desiredFrequency = 50;
    }
    float period, minThrottleSignal, maxThorttleSignal, minDuty, maxDuty;
    int range = pow(2.0, desiredResolution);
    period = 1.0 / desiredFrequency;
    minDuty = 0.001 / period;
    maxDuty = 0.002 / period;

    minThrottleSignal = range * minDuty;
    maxThorttleSignal = range * maxDuty;

    throttleSlope = (float)(maxThorttleSignal - minThrottleSignal) / (signalMax - signalMin);
    throttleIntercept = (float)(minThrottleSignal - (throttleSlope * signalMin));
  }

  // void setChannelReadings(uint8_t refData[22]) {
  //   ch[0] = (refData[0] | (refData[1] << 8)) & 0x07FF; //5 bits remain
  //   ch[1] = ((refData[1] >> 3) | (refData[2] << 5)) & 0x07FF; //2 bits remain
  //   ch[2] = ((refData[2] >> 6) | (refData[3] << 2) | (refData[4] << 10)) & 0x07FF;  //7 bits remain
  //   ch[3] = ((refData[4] >> 1) | (refData[5] << 7)) & 0x07FF; //4 bits remain
  //   ch[4] = ((refData[5] >> 4) | (refData[6] << 4)) & 0x07FF; //1 bit remains
  //   ch[5] = ((refData[6] >> 7) | (refData[7] << 1) | (refData[8] << 9)) & 0x07FF; //6 bits remain
  //   ch[6] = ((refData[8] >> 2) | (refData[9] << 6)) & 0x07FF; //3 bits remain
  //   ch[7] = ((refData[9] >> 5) | (refData[10] << 3)) & 0x07FF; //0 remain
  //   ch[8] = ((refData[11]) | (refData[12] << 8)) & 0x07FF;  //5 bits remain
  //   ch[9] = ((refData[12] >> 3) | (refData[13] << 5)) & 0x07FF; //2 bits remain
  //   ch[10] = ((refData[13] >> 6) | (refData[14] << 2) | (refData[15] << 10)) & 0x07FF;  //7 bits remain
  //   ch[11] = ((refData[15] >> 1) | (refData[16] << 7)) & 0x07FF; //4 bits remain
  //   ch[12] = ((refData[16] >> 4) | (refData[17] << 4)) & 0x07FF; //1 bit remains
  //   ch[13] = ((refData[17] >> 7) | (refData[18] << 1) | (refData[19] << 9)) & 0x07FF; //6 bits remain
  //   ch[14] = ((refData[19] >> 2) | (refData[20] << 6)) & 0x07FF; //3 bits remain
  //   ch[15] = ((refData[20] >> 5) | (refData[21] << 3)) & 0x07FF; //0 remain
  // }

  // void displayReadings() {
  //   Serial.printf("ch0: %i ch1: %i, ch2: %i, ch3: %i ", ch[0], ch[1], ch[2], ch[3]);
  //   Serial.printf("ch4: %i ch5: %i, ch6: %i, ch7: %i ", ch[4], ch[5], ch[6], ch[7]);
  //   Serial.printf("ch8: %i ch9: %i, ch10: %i, ch11: %i ", ch[8], ch[9], ch[10], ch[11]);
  //   Serial.printf("ch12: %i ch13: %i, ch14: %i, ch15: %i\n", ch[12], ch[13], ch[14], ch[15]);
  // }

  void displayReadings()
  {
    Serial.printf("ch0: %i ch1: %i, ch2: %i, ch3: %i ", desiredRoll, desiredPitch, ch[2].data, yawSpeed);
    Serial.printf("ch4: %i ch5: %i, ch6: %i, ch7: %i ", ch[4].data, ch[5].data, ch[6].data, ch[7].data);
    Serial.printf("ch8: %i ch9: %i, ch10: %i, ch11: %i ", ch[8].data, ch[9].data, ch[10].data, ch[11].data);
    Serial.printf("ch12: %i ch13: %i, ch14: %i, ch15: %i\n", ch[12].data, ch[13].data, ch[14].data, ch[15].data);
  }
};
// Initializing channels with max roll and pitch tilt angles
Channels channels(-tiltAngleMax - 1, tiltAngleMax, -yawSpeedMax, yawSpeedMax, minAltitude, maxAltitude);

// Angle PID controllers
PIDController rollAngleController(0.5, 0, 0, 0.55, 0);
PIDController pitchAngleController(0.5, 0, 0, 0.55, 0);
// Rate PID controllers
PIDController rollRateController(0.1, 0.1, 0.1, 0.55, 0);
PIDController pitchRateController(0.1, 0.1, 0.1, 0.55, 0);
PIDController yawRateController(0.1, 0.1, 0.1, 0.55, 0);
// Altitude PID controller
PIDController altitudeController(0.1, 0.1, 0.1, 0, 0);

// ===== Web server =====
WebServer server(80);

const char htmlPageTemplate[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Flight Controller Tuning</title>

<style>
body {
  font-family: Arial;
  max-width: 520px;
  margin: auto;
}
label {
  display: inline-block;
  width: 180px;
}
input {
  width: 90px;
}
</style>

<script>
function updateTelemetry() {
  fetch('/telemetry')
    .then(r => r.json())
    .then(d => {
      document.getElementById('roll').innerText = d.roll.toFixed(2);
      document.getElementById('pitch').innerText = d.pitch.toFixed(2);
      document.getElementById('altitude').innerText = d.altitude.toFixed(2);
    });
}

function updateParameters() {
  const p = new URLSearchParams();

  p.append('rollAngleKp', document.getElementById('rollAngleKp').value);
  p.append('rollAngleKi', document.getElementById('rollAngleKi').value);
  p.append('rollAngleKd', document.getElementById('rollAngleKd').value);
  p.append('rollAngleTol', document.getElementById('rollAngleTol').value);
  p.append('rollAngleDAlpha', document.getElementById('rollAngleDAlpha').value);

  p.append('pitchAngleKp', document.getElementById('pitchAngleKp').value);
  p.append('pitchAngleKi', document.getElementById('pitchAngleKi').value);
  p.append('pitchAngleKd', document.getElementById('pitchAngleKd').value);
  p.append('pitchAngleTol', document.getElementById('pitchAngleTol').value);
  p.append('pitchAngleDAlpha', document.getElementById('pitchAngleDAlpha').value);

  p.append('rollRateKp', document.getElementById('rollRateKp').value);
  p.append('rollRateKi', document.getElementById('rollRateKi').value);
  p.append('rollRateKd', document.getElementById('rollRateKd').value);
  p.append('rollRateTol', document.getElementById('rollRateTol').value);
  p.append('rollRateDAlpha', document.getElementById('rollRateDAlpha').value);

  p.append('pitchRateKp', document.getElementById('pitchRateKp').value);
  p.append('pitchRateKi', document.getElementById('pitchRateKi').value);
  p.append('pitchRateKd', document.getElementById('pitchRateKd').value);
  p.append('pitchRateTol', document.getElementById('pitchRateTol').value);
  p.append('pitchRateDAlpha', document.getElementById('pitchRateDAlpha').value);

  p.append('yawRateKp', document.getElementById('yawRateKp').value);
  p.append('yawRateKi', document.getElementById('yawRateKi').value);
  p.append('yawRateKd', document.getElementById('yawRateKd').value);
  p.append('yawRateTol', document.getElementById('yawRateTol').value);
  p.append('yawRateDAlpha', document.getElementById('yawRateDAlpha').value);

  p.append('altitudeKp', document.getElementById('altitudeKp').value);
  p.append('altitudeKi', document.getElementById('altitudeKi').value);
  p.append('altitudeKd', document.getElementById('altitudeKd').value);
  p.append('altitudeTol', document.getElementById('altitudeTol').value);
  p.append('altitudeDAlpha', document.getElementById('altitudeDAlpha').value);

  fetch('/update?' + p.toString());
}

setInterval(updateTelemetry, 300);
</script>
</head>

<body>
<h2>Flight Controller PID Tuning</h2>

<h3>Roll Angle</h3>
<input id="rollAngleKp" value="%.3f"> Kp<br>
<input id="rollAngleKi" value="%.3f"> Ki<br>
<input id="rollAngleKd" value="%.3f"> Kd<br>
<input id="rollAngleTol" value="%.3f"> Tol<br>
<input id="rollAngleDAlpha" value="%.3f"> Dα<br>

<h3>Pitch Angle</h3>
<input id="pitchAngleKp" value="%.3f">
<input id="pitchAngleKi" value="%.3f">
<input id="pitchAngleKd" value="%.3f">
<input id="pitchAngleTol" value="%.3f">
<input id="pitchAngleDAlpha" value="%.3f">

<h3>Roll Rate</h3>
<input id="rollRateKp" value="%.3f">
<input id="rollRateKi" value="%.3f">
<input id="rollRateKd" value="%.3f">
<input id="rollRateTol" value="%.3f">
<input id="rollRateDAlpha" value="%.3f">

<h3>Pitch Rate</h3>
<input id="pitchRateKp" value="%.3f">
<input id="pitchRateKi" value="%.3f">
<input id="pitchRateKd" value="%.3f">
<input id="pitchRateTol" value="%.3f">
<input id="pitchRateDAlpha" value="%.3f">

<h3>Yaw Rate</h3>
<input id="yawRateKp" value="%.3f">
<input id="yawRateKi" value="%.3f">
<input id="yawRateKd" value="%.3f">
<input id="yawRateTol" value="%.3f">
<input id="yawRateDAlpha" value="%.3f">

<h3>Altitude</h3>
<input id="altitudeKp" value="%.3f">
<input id="altitudeKi" value="%.3f">
<input id="altitudeKd" value="%.3f">
<input id="altitudeTol" value="%.3f">
<input id="altitudeDAlpha" value="%.3f">

<br><br>
<button onclick="updateParameters()">Update</button>

<hr>
<p>Roll: <span id="roll">0</span></p>
<p>Pitch: <span id="pitch">0</span></p>
<p>Altitude: <span id="altitude">0</span></p>

</body>
</html>
)rawliteral";
//--------------------------------------------------------------------------------------------------------------------
//          FUNCTION DECLARATIONS
//--------------------------------------------------------------------------------------------------------------------

void initBMP388();
void getInitialPressure(int sampleSize);
void ICM45686_Init();
// void setDutyCycle();
void PIDWebPage();
void update();
void movement();
void updateIMUReadings();
void updateAltitudeReadings();
void getChannelPacket();
void sendAltitudePacket();
void sendAttitudePacket();
void sendBatteryPacket();
float calSlope(float y2, float y1, float x2, float x1);
float calYIntercept(float slope, float x, float y);
bool updatePID();
void motorMixingAlgorithm(bool altitudeControlled);
uint8_t crc8_d5(const uint8_t *ptr, uint8_t len);
void getBatteryValues();
void getCompensationData();
void I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data);
uint8_t I2CReadIREG(uint16_t target_register, uint16_t base_increment);
void I2CReadBurst(uint8_t startReg, uint8_t deviceAddress, uint8_t burst);
void anti_Windup();
void clampMixedMotors();

//--------------------------------------------------------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200); // For USB debug
  // Use UART1 to receive CRSF at 420000 baud
  Serial1.begin(bRate, SERIAL_8N1, 16, 17); // RX=GPIO16, TX=GPIO17
  delay(2000);

  Serial.printf("Started Serial1 at %i baud\n", bRate);
  Wire.begin(sdaPin, sclPin, i2cSpeed); // initializing i2c communication

  delay(100);

  initBMP388(); // initializing the pressure sensor

  delay(100);

  getInitialPressure(InitSampleSize); // getting an initial sample of InitSampleSize for "initial position" to be used as reference

  delay(100);

  ICM45686_Init(); // initializing the IMU sensor

  channels.setDutyCycle(PWMFrequency, PWMResolution);

  // Attaching channels to the following GPIO pins with 500Hz frequency and 11 bit resolution. These will be used for sending PWM signals to the ESCs which control the brushless motors
  ledcSetup(frontLeft, PWMFrequency, PWMResolution);
  ledcSetup(frontRight, PWMFrequency, PWMResolution);
  ledcSetup(rearRight, PWMFrequency, PWMResolution);
  ledcSetup(rearLeft, PWMFrequency, PWMResolution);

  ledcAttachPin(frontLeftPin, frontLeft);
  ledcAttachPin(frontRightPin, frontRight);
  ledcAttachPin(rearRightPin, rearRight);
  ledcAttachPin(rearLeftPin, rearLeft);

  ledcWrite(frontLeft, 0);
  ledcWrite(frontRight, 0);
  ledcWrite(rearRight, 0);
  ledcWrite(rearLeft, 0);
  analogReadResolution(12);
  analogSetPinAttenuation(batteryADCPin, ADC_11db);
  PIDWebPage();
  Serial.println("EXITING SETUP");
  previousTime = micros();
}
//--------------------------------------------------------------------------------------------------------------------
// Main loop
//--------------------------------------------------------------------------------------------------------------------
void loop()
{

  // Updating all the sensor readings (Altitude, IMU, Receiver Data)
  update();
  // ################################################################################################################################
  //           ARMED STATE
  // ################################################################################################################################
  // Lower left switch position 1 (low) 
  if (channels.ch[4].data < 900)
  {
    Serial.println("armed");
    while (true)
    {
      update();
      // movement();
      if (channels.ch[4].data > 900)
      {
        // if disarmed
        break;
      }
      // Lower right switch position 1 (low) 
      if (channels.ch[6].data < 900)
      {
        ledcWrite(frontLeft, mainThrottleInput);
        ledcWrite(frontRight, mainThrottleInput);
        ledcWrite(rearRight, mainThrottleInput);
        ledcWrite(rearLeft, mainThrottleInput);
        // channels.displayReadings();
        // display
        //  Serial.printf("Desired Pitch: %i, Current Pitch: %.2f, Desired Roll: %i, Current Roll: %.2f, Front Left: %i, Front Right: %i, Rear Right: %i, Rear Left: %i\n", desiredPitch, pitch, desiredRoll, roll, frontLeftInput, frontRightInput, rearRightInput, rearLeftInput);
      }
    }
  }
  // ################################################################################################################################
  //           DISARMED STATE
  // ################################################################################################################################
  else if (channels.ch[4].data > 900)
  {
    Serial.println("disarmed");
    while (true)
    {
      update();
      // Lower left switch position 2 (high) 
      if (channels.ch[4].data < 900)
      {
        // if armed
        break;
      }
      // Lower right switch position 1 (low) 
      if (channels.ch[6].data < 900)
      {
        // ################################################################################################################################
        //           CALIBRATION STATE (WHEN DISARMED)
        // ################################################################################################################################
        // Serial.printf("In Calibration State\n");
        if (mainThrottleInput == 2047)
        {
          ledcWrite(frontLeft, mainThrottleInput);
          ledcWrite(frontRight, mainThrottleInput);
          ledcWrite(rearRight, mainThrottleInput);
          ledcWrite(rearLeft, mainThrottleInput);
          // Serial.println("High Set");
        }
        else if (mainThrottleInput == 1023)
        {
          ledcWrite(frontLeft, mainThrottleInput);
          ledcWrite(frontRight, mainThrottleInput);
          ledcWrite(rearRight, mainThrottleInput);
          ledcWrite(rearLeft, mainThrottleInput);
          // Serial.println("Low Set");
        }
        // display
        //  Serial.printf("Throttle: %i, Function: %i, Duty Cycle: %.2f\n", channels.ch[2].data, mainThrottleInput, mainThrottleInput/PWMRange);
        //  channels.displayReadings();
        //  Serial.printf("Main Throttle %i\n", mainThrottleInput);
      }
    }
  }

  // Serial.printf("Throttle: %i, Duty Cycle: %i\n", channels.ch[2].data, y);
  // channels.displayReadings();
}

//--------------------------------------------------------------------------------------------------------------------
// General Functions
//--------------------------------------------------------------------------------------------------------------------
void update()
{
  // Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0; // converted to seconds instead of micro seconds
  previousTime = currentTime;
  updateIMUReadings();
  updateAltitudeReadings();
  getChannelPacket();
  sendAltitudePacket();
  sendAttitudePacket();
  sendBatteryPacket();
  // if upper-right switch is in up position then update webpage
  if (channels.ch[7].data > 1200) {
    static uint32_t lastWebHandle = 0;
    uint32_t now = micros();
    if (now - lastWebHandle > 10000) {  // Only every 10ms
        server.handleClient();
        lastWebHandle = now;
    }
  }
  // if (altCounter > 1000) {
  // Serial.printf("Kp: %.2f, Ki: %.2f, Kd: %.2f\n", Kp, Ki, Kd);
  // altCounter = 0;
  // }
  // else {
  // altCounter++;
  // }
  // channels.displayReadings();
}

// calculates the slope using two points on a graph
float calSlope(float y2, float y1, float x2, float x1)
{
  return (float)(y2 - y1) / (x2 - x1);
}
// calculates the y intercept using the slope, x and y values
float calYIntercept(float slope, float x, float y)
{
  return y - (slope * x);
}

//--------------------------------------------------------------------------------------------------------------------
// Movement Functions
//--------------------------------------------------------------------------------------------------------------------
void movement()
{
  motorMixingAlgorithm(updatePID());
  // Serial.println("About to write Ledc");
  clampMixedMotors();
  ledcWrite(frontLeft, motorMatrix[0]);
  ledcWrite(frontRight, motorMatrix[1]);
  ledcWrite(rearRight, motorMatrix[2]);
  ledcWrite(rearLeft, motorMatrix[3]);
  // Serial.printf("finished writing ledC: 0 = %i, 1 = %i, 2 = %i, 3 = %i\n", motorMatrix[0], motorMatrix[1], motorMatrix[2], motorMatrix[3]);
}

bool updatePID()
{
  if (channels.ch[5].data < 500) // switch position 1 (low)
  {
    // Angle Only (Manual Throttle)
    Serial.println("Angle");
    if (altitudeLock)
    {
      altitudeLock = false;
      // altitudeController.reset();
    }
    rollAnglePID = rollAngleController.compute(desiredRoll, roll, deltaTime, mainThrottleInput);
    pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch, deltaTime, mainThrottleInput);

    rollRatePID = rollRateController.compute(rollAnglePID, gyroY, deltaTime, mainThrottleInput);
    pitchRatePID = pitchRateController.compute(pitchAnglePID, gyroX, deltaTime, mainThrottleInput);
    yawRatePID = yawRateController.compute(yawSpeed, gyroZ, deltaTime, mainThrottleInput);
    return false;
  }
  else if (channels.ch[5].data > 1400) // switch position 3 (high)
  {
    // Angle + Altitude (Autonomous Throttle)
    Serial.println("Angle + Altitude");
    if (altitudeLock)
    {
      altitudeLock = false;
      // altitudeController.reset();
    }
    // Altitude controller uses desired altitude instead of hover altitude
    rollAnglePID = rollAngleController.compute(desiredRoll, roll, deltaTime, mainThrottleInput);
    pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch, deltaTime, mainThrottleInput);

    rollRatePID = rollRateController.compute(rollAnglePID, gyroY, deltaTime, mainThrottleInput);
    pitchRatePID = pitchRateController.compute(pitchAnglePID, gyroX, deltaTime, mainThrottleInput);
    yawRatePID = yawRateController.compute(yawSpeed, gyroZ, deltaTime, mainThrottleInput);
    altitudePID = altitudeController.compute(desiredAltitude, altitude, deltaTime, mainThrottleInput);
    // motorMixingAlgorithm(true);
    return true;
  }
  else // switch position 2 (middle)
  {
    // Altitude Hold (Hover)
    Serial.println("Hover");
    // if altitude is not locked, setting hover to current altitude, resetting altitude controller, and setting condition to true
    if (!altitudeLock)
    {
      hoverAltitude = altitude;
      // altitudeController.reset();
      altitudeLock = true;
    }
    // altitude controller uses hover altitude instead of desired altitude
    rollAnglePID = rollAngleController.compute(desiredRoll, roll, deltaTime, mainThrottleInput);
    pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch, deltaTime, mainThrottleInput);

    rollRatePID = rollRateController.compute(rollAnglePID, gyroY, deltaTime, mainThrottleInput);
    pitchRatePID = pitchRateController.compute(pitchAnglePID, gyroX, deltaTime, mainThrottleInput);
    yawRatePID = yawRateController.compute(yawSpeed, gyroZ, deltaTime, mainThrottleInput);
    altitudePID = altitudeController.compute(hoverAltitude, altitude, deltaTime, mainThrottleInput);
    // motorMixingAlgorithm(true);
    return true;
  }
}

void motorMixingAlgorithm(bool altitudeControlled)
{
  if (!altitudeControlled)
  {
    // clamping the manual throttle value to 90% of the max throttle (80% duty cycle)
    if (mainThrottleInput > maxThrottleValue * 0.9)
    {
      mainThrottleInput = maxThrottleValue * 0.9;
    }
    // using the manuel throttle input to control the over all thrust of the qaudcopter
    motorMatrix[0] = mainThrottleInput - pitchRatePID + rollRatePID - yawRatePID;
    motorMatrix[1] = mainThrottleInput - pitchRatePID - rollRatePID + yawRatePID;
    motorMatrix[2] = mainThrottleInput + pitchRatePID - rollRatePID - yawRatePID;
    motorMatrix[3] = mainThrottleInput + pitchRatePID + rollRatePID + yawRatePID;
    // if the mixed control signal is below 60% of the PWM signal (20% motor duty cycle), then it is capped at 20% duty cycle
    if (motorMatrix[0] < maxThrottleValue * 0.6 && mainThrottleInput > maxThrottleValue * 0.6)
    {
      motorMatrix[0] = maxThrottleValue * 0.6;
    }
    if (motorMatrix[1] < maxThrottleValue * 0.6 && mainThrottleInput > maxThrottleValue * 0.6)
    {
      motorMatrix[1] = maxThrottleValue * 0.6;
    }
    if (motorMatrix[2] < maxThrottleValue * 0.6 && mainThrottleInput > maxThrottleValue * 0.6)
    {
      motorMatrix[2] = maxThrottleValue * 0.6;
    }
    if (motorMatrix[3] < maxThrottleValue * 0.6 && mainThrottleInput > maxThrottleValue * 0.6)
    {
      motorMatrix[3] = maxThrottleValue * 0.6;
    }
  }
  else if (altitudeControlled)
  {
    // checking for windup and unwiding if true
    anti_Windup();
    // clamping the altitude PID output to 90% of the max throttle (80% duty cycle), in case anti_windup did was not enough. (safety check)
    if (altitudePID > maxThrottleValue * 0.9)
    {
      altitudePID = maxThrottleValue * 0.9;
    }
    // using the altitude PID output to control thrust of the motors
    motorMatrix[0] = altitudePID - pitchRatePID + rollRatePID - yawRatePID;
    motorMatrix[1] = altitudePID - pitchRatePID - rollRatePID + yawRatePID;
    motorMatrix[2] = altitudePID + pitchRatePID - rollRatePID - yawRatePID;
    motorMatrix[3] = altitudePID + pitchRatePID + rollRatePID + yawRatePID;
  }
}

void anti_Windup()
{
  // check if altitude PID is greater than 90% of max throttle AND if it has the same sign as the error.
  if (altitudePID > 0.9 * maxThrottleValue && altitudePID * altitudeController.error > 0)
  {
    // unwind the intergral if both are true
    altitudePID = altitudeController.unwind(deltaTime);
  }
}

void clampMixedMotors()
{
  // Clamping the mixed motor matrix to max throttle (100% duty cycle)
  if (motorMatrix[0] > maxThrottleValue)
  {
    motorMatrix[0] = maxThrottleValue;
  }
  if (motorMatrix[1] > maxThrottleValue)
  {
    motorMatrix[1] = maxThrottleValue;
  }
  if (motorMatrix[2] > maxThrottleValue)
  {
    motorMatrix[2] = maxThrottleValue;
  }
  if (motorMatrix[3] > maxThrottleValue)
  {
    motorMatrix[3] = maxThrottleValue;
  }

  // Clamping the mixed motor matrix to min throttle (0% duty cycle)
  if (motorMatrix[0] < minThrottleValue)
  {
    motorMatrix[0] = minThrottleValue;
  }
  if (motorMatrix[1] < minThrottleValue)
  {
    motorMatrix[1] = minThrottleValue;
  }
  if (motorMatrix[2] < minThrottleValue)
  {
    motorMatrix[2] = minThrottleValue;
  }
  if (motorMatrix[3] < minThrottleValue)
  {
    motorMatrix[3] = minThrottleValue;
  }
}
//--------------------------------------------------------------------------------------------------------------------
// Receiver Related Functions
//--------------------------------------------------------------------------------------------------------------------

// reads the 16 channel packet from the receiver (does not implement crc8 checksum, some garbage packets can slip)
void getChannelPacket()
{
  uint8_t lastRead = Serial1.read();
  if (lastRead == CRSF_Addr)
  {
    channelsPacket[0] = lastRead; // storing the address (header) byte
    lastRead = Serial1.read();
    if (lastRead == CRSF_Length_Channels)
    {
      channelsPacket[1] = lastRead; // storing the length of the frame
      lastRead = Serial1.read();
      if (lastRead == CRSF_Type_Channels)
      {
        channelsPacket[2] = lastRead; // storing the type of frame
        // reading the remaining channels packet (16 channels + crc8, excluding type as it is read before)
        for (int i = 0; i < channelsPacket[1] - 1; i++)
        {
          channelsPacket[i + 3] = Serial1.read();
        }
        crc8 = crc8_d5(channelsPacket + 2, channelsPacket[1] - 1);
        // performing the crc8 checksum, if it is correct, then the data is assigned
        if (crc8 == channelsPacket[25])
        {
          // Serial.printf("Received crc8: %i, Computed crc8: %i\n", channelsPacket[25], crc8);
          channels.setChannelReadings(channelsPacket + 3);
        }
        // channels.displayReadings();
      }
    }
  }
}

uint8_t crc8_d5(const uint8_t *ptr, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    uint8_t inbyte = *ptr++;
    for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t mix = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (mix)
        crc ^= 0xD5;
      inbyte <<= 1;
    }
  }
  return crc;
}

void sendAltitudePacket()
{
  // if (altCounter == 100) {
  uint8_t altitudePacket[8];                      // packet frame in 8 bit array
  uint16_t dm_Altitude = (altitude * 10) + 10000; // altitude converted to decimeters and stored in 16bits
  // when the dm_Altitude is sent as decimeters, it will automatically be converted to meters in the transmitter

  altitudePacket[0] = CRSF_Sensor;        // packet address
  altitudePacket[1] = 0x06;               // 4 data + crc + type, 6 total (excluding address and length)
  altitudePacket[2] = CRSF_Type_Altitude; // packet type

  altitudePacket[3] = (dm_Altitude >> 8); // First byte big endian
  altitudePacket[4] = dm_Altitude;
  altitudePacket[5] = 0; // Vertical Speed
  altitudePacket[6] = 0;

  altitudePacket[7] = crc8_d5(altitudePacket + 2, 5); // packet + 2 to skip the first 2 bytes in the array (address and length), and 5 is to perform crc on 5 bytes
  // Serial.printf("Altitude in dm: %i, First Byte: 0x%X, Second Byte: 0x%X, int1: %i, int2: %i, Altitude: %.2f\n", dm_Altitude - 10000, altPacket[3], altPacket[4], altPacket[3], altPacket[4], altitude);
  Serial1.write(altitudePacket, 8); // sending the packet to receiver. 8 is for all 8 bytes in array
  altCounter = 0;
  // } else {
  //   altCounter++;
  // }
}

void sendAttitudePacket()
{
  uint8_t attitudePacket[10];
  uint16_t pitchPacket = ((pitch * PI) / 180) * 10000;
  uint16_t rollPacket = ((roll * PI) / 180) * 10000;
  uint16_t yawPacket = ((yaw * PI) / 180) * 10000;

  attitudePacket[0] = CRSF_Sensor;        // Packet destination is the RC transmitter
  attitudePacket[1] = 0x08;               // Packet Length (6 data + type + crc, 8 total)
  attitudePacket[2] = CRSF_Type_Attitude; // Packet type is attitude
  attitudePacket[3] = pitchPacket >> 8;   // Pitch, big endian
  attitudePacket[4] = pitchPacket;
  attitudePacket[5] = rollPacket >> 8; // Roll, big endian
  attitudePacket[6] = rollPacket;
  attitudePacket[7] = yawPacket >> 8; // Yaw, big endian
  attitudePacket[8] = yawPacket;
  attitudePacket[9] = crc8_d5(attitudePacket + 2, 7); // Calculating the crc8 checksum

  Serial1.write(attitudePacket, 10);
  // Serial.printf("pitch: %.2f, pitchPacket: %i, roll: %.2f, rollPacket: %i\n", pitch, pitchPacket, roll, rollPacket);
}

void sendBatteryPacket()
{
  getBatteryValues();
  uint8_t batteryPacket[12];
  uint16_t voltage = batteryVoltage * 10;                                                      // voltage LSB = 10µV
  uint16_t current = 10;                                                                       // current LSB = 10µA
  uint32_t usedCapacity_remaining = (batteryCapacityRemaining << 8) | batteryRemainingPercent; // upper 24bits are the used capacity in mAh, lower 8 bits are remaining percentage (%)

  batteryPacket[0] = CRSF_Sensor;
  batteryPacket[1] = 0x0A;
  batteryPacket[2] = CRSF_Type_Battery;
  batteryPacket[3] = voltage >> 8;
  batteryPacket[4] = voltage;
  batteryPacket[5] = current >> 8;
  batteryPacket[6] = current;
  batteryPacket[7] = usedCapacity_remaining >> 24;
  batteryPacket[8] = usedCapacity_remaining >> 16;
  batteryPacket[9] = usedCapacity_remaining >> 8;
  batteryPacket[10] = usedCapacity_remaining;
  batteryPacket[11] = crc8_d5(batteryPacket + 2, 9);

  Serial1.write(batteryPacket, 12);
}

void getBatteryValues()
{
  rawBatteryReadings = analogRead(batteryADCPin);
  measuredVoltage = (rawBatteryReadings / 4095.0) * Vref + 0.4;
  batteryVoltage = measuredVoltage * resistanceRatio;
  batteryPercentUsed = ((16.8 - batteryVoltage) / 4.8);                     // from 0-1
  batteryCapacityRemaining = (1 - batteryPercentUsed) * maxBatteryCapacity; // mAh
  batteryRemainingPercent = (1 - batteryPercentUsed) * 100;                 // percentage (%)
  // Serial.printf("Raw readings: %i, Measured voltage: %.2fV, Battery voltage: %.2fV, Capacity: %i mAh, Remaining: %i%%\n", rawBatteryReadings, measuredVoltage, batteryVoltage, batteryCapacityRemaining, batteryRemainingPercent);
}

//--------------------------------------------------------------------------------------------------------------------
//          BMP388 related functions
//--------------------------------------------------------------------------------------------------------------------

// initilizes the BMP388 sensor
void initBMP388()
{
  Serial.println("1");
  Wire.beginTransmission(BMP388ADDRESS); // begining communication with BMP388
  Wire.write(0x1B);                      // PWR CONTROL REGISTER
  Wire.write(0x00);                      // SET TO SLEEP MODE and ENABLE TEMP AND PRESSURE SENSORS
  Wire.endTransmission();
  delay(20);

  Serial.println("2");
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x1C); // OVER SAMPLING RESOLUTION REGISTER (osr_p and t)
  // Wire.write(0x0D); //SETTING THE TEMP RESOLUTION TO x2 AND PRESSURE TO x32
  Wire.write(0x03); // SETTING THE TEMP RESOLUTION TO x1 AND PRESSURE TO x8
  Wire.endTransmission();
  delay(20);

  Serial.println("3");
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x1D); // Oversampling reigster config
  // Wire.write(0x04); //setting the sampling period to 80ms
  Wire.write(0x02); // setting the sampling period to 20ms
  Wire.endTransmission();
  delay(20);

  Serial.println("4");
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x1F); // IIR FILTER REGISTER CONFIG
  Wire.write(0x04); // SET IIR COEFFICENT TO 3
  Wire.endTransmission();
  delay(20);

  Serial.println("5");
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x1B); // PWR CONTROL REGISTER
  Wire.write(0x33); // SET TO NORMAL MODE and ENABLE TEMP AND PRESSURE SENSORS
  Wire.endTransmission();
  delay(20);

  Serial.println("I WILL GET COMPENSATION DATA");
  getCompensationData();
  delay(20);
}

// prints the sensor status
void checkSensorStatus()
{
  // Read status register
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x03); // status
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388ADDRESS, 1);
  uint8_t status = Wire.read();

  // Read PWR_CTRL
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x1B); // PWR_CTRL
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388ADDRESS, 1);
  uint8_t pwr_ctrl = Wire.read();

  Serial.printf("Sensor status = 0x%02X | PWR_CTRL = 0x%02X\n", status, pwr_ctrl);
}

// reads compensation data used to compute temperature and pressure values from the sensor (these values are different from sensor to sensor)
void getCompensationData()
{

  int8_t NVM_PAR_T3, NVM_PAR_P3, NVM_PAR_P4, NVM_PAR_P7, NVM_PAR_P8, NVM_PAR_P10, NVM_PAR_P11;
  int16_t NVM_PAR_P1, NVM_PAR_P2, NVM_PAR_P9;
  uint16_t NVM_PAR_T1, NVM_PAR_T2, NVM_PAR_P5, NVM_PAR_P6;

  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x31);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388ADDRESS, 21);
  delay(500);
  Serial.println("I AM ABOUT TO READ");
  if (Wire.available() == 21)
  {
    Serial.println("I AM READING");
    NVM_PAR_T1 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_T2 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_T3 = (int8_t)Wire.read();
    NVM_PAR_P1 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_P2 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_P3 = (int8_t)Wire.read();
    NVM_PAR_P4 = (int8_t)Wire.read();
    NVM_PAR_P5 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_P6 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_P7 = (int8_t)Wire.read();
    NVM_PAR_P8 = (int8_t)Wire.read();
    NVM_PAR_P9 = Wire.read() | (Wire.read() << 8);
    NVM_PAR_P10 = (int8_t)Wire.read();
    NVM_PAR_P11 = (int8_t)Wire.read();
    delay(50);

    PAR_T1 = (float)NVM_PAR_T1 * 256.0f;
    PAR_T2 = (float)NVM_PAR_T2 / 1073741824.0f;
    PAR_T3 = (float)NVM_PAR_T3 / 281474976710656.0f;

    PAR_P1 = ((float)NVM_PAR_P1 - 16384.0f) / 1048576.0f;
    PAR_P2 = ((float)NVM_PAR_P2 - 16384.0f) / 536870912.0f;
    PAR_P3 = (float)NVM_PAR_P3 / 4294967296.0f;

    PAR_P4 = (float)NVM_PAR_P4 / 137438953472.0f;
    PAR_P5 = (float)NVM_PAR_P5 * 8.0f;
    PAR_P6 = (float)NVM_PAR_P6 / 64.0f;

    PAR_P7 = (float)NVM_PAR_P7 / 256.0f;
    PAR_P8 = (float)NVM_PAR_P8 / 32768.0f;
    PAR_P9 = (float)NVM_PAR_P9 / 281474976710656.0f;

    PAR_P10 = (float)NVM_PAR_P10 / 281474976710656.0f;
    PAR_P11 = (float)NVM_PAR_P11 / pow(2.0, 65); // Or hardcode: / 3.6893488e+19
    // Serial.println(NVM_PAR_T1);
    // Serial.println(NVM_PAR_T2);
    // Serial.println(NVM_PAR_T3);
    // Serial.println(NVM_PAR_P1);
    // Serial.println(NVM_PAR_P2);
    // Serial.println(NVM_PAR_P3);
    // Serial.println(NVM_PAR_P4);
    // Serial.println(NVM_PAR_P5);
    // Serial.println(NVM_PAR_P6);
    // Serial.println(NVM_PAR_P7);
    // Serial.println(NVM_PAR_P8);
    // Serial.println(NVM_PAR_P9);
    // Serial.println(NVM_PAR_P10);
    // Serial.println(NVM_PAR_P11);
  }
}

// reads the raw temp and pressure values
void readTempPres(uint32_t *p, uint32_t *t)
{
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x04);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388ADDRESS, 6);

  if (Wire.available() == 6)
  {
    *p = (Wire.read() | (Wire.read() << 8) | (Wire.read() << 16));
    *t = (Wire.read() | (Wire.read() << 8) | (Wire.read() << 16));
  }
}

// converts raw temperature values to proper values
float BMP388CompensatTemp(uint32_t uncompTemp)
{
  float partial_dataT1, partial_dataT2, t_lin;
  partial_dataT1 = (float)(uncompTemp - PAR_T1);
  partial_dataT2 = (float)(partial_dataT1 * PAR_T2);

  t_lin = partial_dataT2 + (partial_dataT1 * partial_dataT1) * PAR_T3;
  return t_lin;
}

// converts raw pressure values to proper values
float BMP388CompensatePressure(uint32_t uncompPressure, float calibratedTemp)
{
  float calibratedPressure;
  float partial_data1, partial_data2, partial_data3, partial_data4;
  float partial_out1, partial_out2;

  partial_data1 = PAR_P6 * calibratedTemp;
  partial_data2 = PAR_P7 * (calibratedTemp * calibratedTemp);
  partial_data3 = PAR_P8 * (calibratedTemp * calibratedTemp * calibratedTemp);
  partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = PAR_P2 * calibratedTemp;
  partial_data2 = PAR_P3 * (calibratedTemp * calibratedTemp);
  partial_data3 = PAR_P4 * (calibratedTemp * calibratedTemp * calibratedTemp);
  partial_out2 = (float)uncompPressure * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = (float)uncompPressure * (float)uncompPressure;
  partial_data2 = PAR_P9 + (PAR_P10 * calibratedTemp);
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((float)uncompPressure * (float)uncompPressure * (float)uncompPressure) * PAR_P11;
  calibratedPressure = partial_out1 + partial_out2 + partial_data4;

  return calibratedPressure;
}

// reads an initial pressure sample and averages them to use it as the reference pressure for the barometric forumla (to make the launch point of the drone the 0 altitude reference)
void getInitialPressure(int sampleSize)
{
  initialPressure = 0;
  int counter = 0;
  for (int i = 0; i < sampleSize; i++)
  {
    readTempPres(&rawPressure, &rawTemp);
    initialTemp = BMP388CompensatTemp(rawTemp);
    initialPressure = initialPressure + BMP388CompensatePressure(rawPressure, initialTemp);
    counter++;
  }
  initialPressure = initialPressure / counter;
}

// computes the altitude readings using the barometric formula and the reference sample pressure at initialization
void updateAltitudeReadings()
{
  // Check data ready bit in status register (0x03, bit 5 = drdy_press, bit 6 = drdy_temp)
  Wire.beginTransmission(BMP388ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388ADDRESS, 1);
  uint8_t status = Wire.read();
    
  if (!(status & 0x20)) return;  // Pressure not ready, skip this cycle

  readTempPres(&rawPressure, &rawTemp);
  temp = BMP388CompensatTemp(rawTemp);
  pressure = BMP388CompensatePressure(rawPressure, temp);
  altitude = 44330.0f * (1.0f - expf(0.1903f * logf(pressure / initialPressure)));
  // altitude = (44330 * (1 - pow(pressure / initialPressure, 0.1903))); // barometric formula without temperature
  // altitude1 = ((pow(initialPressure / pressure, 0.1903) - 1)*(temp + 273.15))/0.0065;
  // altitude = -(pressure - initialPressure) / (1.225 * 9.81); //hydro static approach
  // Serial.printf("Temp is: %.2f °C, and the Pressure is %f Pa \n", temp, pressure);
  // Serial.printf("Altitude is: %.2f m\n", altitude);
  // Serial.print(altitude);
  // Serial.print(", ");
  // Serial.println(altitude1);
}

//--------------------------------------------------------------------------------------------------------------------
//          ICM45686 related functions
//--------------------------------------------------------------------------------------------------------------------

// initilizes the ICM45686 sensor
void ICM45686_Init()
{
  Wire.beginTransmission(ICMAddress);
  Wire.write(0x72); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(ICMAddress, 1);

  if (Wire.available() == 1)
  {
    Serial.printf("The WHO AM I reads: %X\n", Wire.read());
  }
  delay(10);

  I2CWriteIREG(0xA6, 0xA400, 0x20); // Enabling gyro AAF, Disabling interpolator
  delay(10);
  Serial.printf("The data written at 0xA6 is: 0x%X \n", I2CReadIREG(0xA6, 0xA400));
  delay(10);

  I2CWriteIREG(0x7B, 0xA400, 0x01); // Enabling accel AAF, Disabling interpolator
  delay(10);
  Serial.printf("The data written at 0x7B is: 0x%X \n", I2CReadIREG(0x7B, 0xA400));
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x1B); // select register
  Wire.write(0X34); // setting accel to +-4g (gravity constants) and ODR to 3.2KHz
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x1C); // select register
  Wire.write(0X34); // setting gyro to +-500 degrees per second and ODR to 3.2KHz
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x10); // select register
  Wire.write(0X0F); // setting both gyro and accel to low noise mode
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x1D); // select register
  Wire.write(0X5F); // setting the fifo config0 register to stream mode, and depth of 8KB
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x22); // select register
  Wire.write(0X00); // setting fifo config 4 to disable compression packets, and disable time stamp into FIFO
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x21); // select register
  Wire.write(0X07); // setting fifo config 3 to disable high resolution. note: execute this after enabling the sensors
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x2F); // select register
  Wire.write(0X00); // setting IOC PAD SCENARIO to disable AUX1
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x3A); // select register
  Wire.write(0X00); // setting APEX CONFIG 1 to disable all interrupt generations
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ICMAddress);
  Wire.write(0x10); // select register
  Wire.write(0X0F); // setting INT2 CONFIG 0 to disable all interrupts
  Wire.endTransmission();
  delay(10);
}

// updates the IMU readings (to be called every iteration)
void updateIMUReadings()
{
  I2CReadBurst(0x00, ICMAddress, 12);

  // converting raw reading to proper values
  accelX = (rawAccelX * 9.8) / accelSensitivity;
  accelY = (rawAccelY * 9.8) / accelSensitivity;
  accelZ = (rawAccelZ * 9.8) / accelSensitivity;

  gyroX = (float)(rawGyroX / gyroSensitivity);
  gyroY = (float)(rawGyroY / gyroSensitivity);
  gyroZ = (float)(-rawGyroZ / gyroSensitivity);

  // computing the pitch and roll from accel
  accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  accelRoll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  // pitch and roll changes calculated using gyroscope
  pitch = pitch + (gyroX * deltaTime);
  roll = roll + (gyroY * deltaTime);

  // last calculation is done by using complementary filter with alpha
  pitch = alphaIMU * pitch + (1.0 - alphaIMU) * accelPitch;
  roll = alphaIMU * roll + (1.0 - alphaIMU) * accelRoll;

  // Serial.printf("Gyro X: %.2f°/s, Gyro Y: %.2f°/s , Gyro Z: %.2f°/s\n", gyroX, gyroY, gyroZ);

  // Serial.printf("Accel Pitch: %.2f°, Accel Roll: %.2f°", pitch, accelRoll);
  // Serial.printf(", Pitch: %.2f°, Roll: %.2f° \n", pitch, roll);
}

//--------------------------------------------------------------------------------------------------------------------
//  I2C Helper functions for IMU
//--------------------------------------------------------------------------------------------------------------------

void I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data)
{
  target_register = target_register + base_increment;
  uint8_t upperAddress = target_register >> 8;
  uint8_t lowerAddress = target_register;

  Wire.beginTransmission(ICMAddress);
  // Serial.println("Began transmission");
  Wire.write(0x7C); // IREG register high address
  // Serial.println("0x7C");
  Wire.write(upperAddress); // writing the upper address
  // Serial.println("upper");
  Wire.write(lowerAddress); // writing the lower address
  // Serial.println("lower");
  Wire.write(data); // writing the data into IREG DATA REGISTER
  // Serial.println("data");
  uint8_t err = Wire.endTransmission();
  // Serial.println("about to check");
  if (err != 0)
  {
    Serial.printf("I2C Write Error: %d\n", err);
  }
  else
  {
    // Serial.println("check passed");
  }
}

uint8_t I2CReadIREG(uint16_t target_register, uint16_t base_increment)
{
  target_register = target_register + base_increment;
  uint8_t upperAddress = target_register >> 8;
  uint8_t lowerAddress = target_register;
  uint8_t returnData;
  Wire.beginTransmission(ICMAddress);
  // Serial.println("Began transmission");
  Wire.write(0x7C); // IREG register high address
  // Serial.println("0xC7");
  Wire.write(upperAddress); // writing the upper address
  // Serial.println("Upper");
  Wire.write(lowerAddress); // writing the lower address
  // Serial.println("Lower");
  Wire.endTransmission(false);

  delayMicroseconds(5);
  Wire.requestFrom(ICMAddress, 1);
  // Serial.println("Read Request");

  returnData = Wire.read();
  // Serial.println("Reading");
  Wire.endTransmission();
  // Serial.println("Ended transmission");
  return returnData;
}

void I2CReadBurst(uint8_t startReg, uint8_t deviceAddress, uint8_t burst)
{

  Wire.beginTransmission(deviceAddress);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, burst);

  // Little Endian Format
  rawAccelX = Wire.read();
  rawAccelX = (Wire.read() << 8) | rawAccelX;
  rawAccelY = Wire.read();
  rawAccelY = (Wire.read() << 8) | rawAccelY;
  rawAccelZ = Wire.read();
  rawAccelZ = (Wire.read() << 8) | rawAccelZ;

  rawGyroX = Wire.read();
  rawGyroX = (Wire.read() << 8) | rawGyroX;
  rawGyroY = Wire.read();
  rawGyroY = (Wire.read() << 8) | rawGyroY;
  rawGyroZ = Wire.read();
  rawGyroZ = (Wire.read() << 8) | rawGyroZ;
}

//--------------------------------------------------------------------------------------------------------------------
// Wifi Functions
//--------------------------------------------------------------------------------------------------------------------

// ===== Root page handler =====
void handleRoot()
{
  static char page[8000]; // stack/static buffer

  snprintf(
      page, sizeof(page), htmlPageTemplate,

      // Roll Angle
      rollAngleController.Kp,
      rollAngleController.Ki,
      rollAngleController.Kd,
      rollAngleController.tolerance,
      rollAngleController.derivativeFilterAlpha,

      // Pitch Angle
      pitchAngleController.Kp,
      pitchAngleController.Ki,
      pitchAngleController.Kd,
      pitchAngleController.tolerance,
      pitchAngleController.derivativeFilterAlpha,

      // Roll Rate
      rollRateController.Kp,
      rollRateController.Ki,
      rollRateController.Kd,
      rollRateController.tolerance,
      rollRateController.derivativeFilterAlpha,

      // Pitch Rate
      pitchRateController.Kp,
      pitchRateController.Ki,
      pitchRateController.Kd,
      pitchRateController.tolerance,
      pitchRateController.derivativeFilterAlpha,

      // Yaw Rate
      yawRateController.Kp,
      yawRateController.Ki,
      yawRateController.Kd,
      yawRateController.tolerance,
      yawRateController.derivativeFilterAlpha,

      // Altitude
      altitudeController.Kp,
      altitudeController.Ki,
      altitudeController.Kd,
      altitudeController.tolerance,
      altitudeController.derivativeFilterAlpha);

  server.send(200, "text/html", page);
}

// ===== Update handler =====
void handleUpdate()
{

  // Roll Angle
  rollAngleController.Kp = server.arg("rollAngleKp").toFloat();
  rollAngleController.Ki = server.arg("rollAngleKi").toFloat();
  rollAngleController.Kd = server.arg("rollAngleKd").toFloat();
  rollAngleController.tolerance = server.arg("rollAngleTol").toFloat();
  rollAngleController.derivativeFilterAlpha = server.arg("rollAngleDAlpha").toFloat();

  // Pitch Angle
  pitchAngleController.Kp = server.arg("pitchAngleKp").toFloat();
  pitchAngleController.Ki = server.arg("pitchAngleKi").toFloat();
  pitchAngleController.Kd = server.arg("pitchAngleKd").toFloat();
  pitchAngleController.tolerance = server.arg("pitchAngleTol").toFloat();
  pitchAngleController.derivativeFilterAlpha = server.arg("pitchAngleDAlpha").toFloat();

  // Roll Rate
  rollRateController.Kp = server.arg("rollRateKp").toFloat();
  rollRateController.Ki = server.arg("rollRateKi").toFloat();
  rollRateController.Kd = server.arg("rollRateKd").toFloat();
  rollRateController.tolerance = server.arg("rollRateTol").toFloat();
  rollRateController.derivativeFilterAlpha = server.arg("rollRateDAlpha").toFloat();

  // Pitch Rate
  pitchRateController.Kp = server.arg("pitchRateKp").toFloat();
  pitchRateController.Ki = server.arg("pitchRateKi").toFloat();
  pitchRateController.Kd = server.arg("pitchRateKd").toFloat();
  pitchRateController.tolerance = server.arg("pitchRateTol").toFloat();
  pitchRateController.derivativeFilterAlpha = server.arg("pitchRateDAlpha").toFloat();

  // Yaw Rate
  yawRateController.Kp = server.arg("yawRateKp").toFloat();
  yawRateController.Ki = server.arg("yawRateKi").toFloat();
  yawRateController.Kd = server.arg("yawRateKd").toFloat();
  yawRateController.tolerance = server.arg("yawRateTol").toFloat();
  yawRateController.derivativeFilterAlpha = server.arg("yawRateDAlpha").toFloat();

  // Altitude
  altitudeController.Kp = server.arg("altitudeKp").toFloat();
  altitudeController.Ki = server.arg("altitudeKi").toFloat();
  altitudeController.Kd = server.arg("altitudeKd").toFloat();
  altitudeController.tolerance = server.arg("altitudeTol").toFloat();
  altitudeController.derivativeFilterAlpha = server.arg("altitudeDAlpha").toFloat();

  server.send(200, "text/plain", "Controller parameters updated");
}

void handleTelemetry()
{
  String json = "{";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"altitude\":" + String(altitude, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void PIDWebPage()
{
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);          // Disable modem sleep — reduces latency spikes
  esp_wifi_set_ps(WIFI_PS_NONE); // Ensure power save is fully off


  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // Web routes
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/telemetry", handleTelemetry);

  server.begin();
  Serial.println("Web server started");
}
