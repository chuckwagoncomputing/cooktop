#include <PID_v1.h>
#include <SimpleTimer.h>
#include <SPI.h>

#define DEBUG 0

// Default Settings
#define DEFAULTPWM 200
#define DEFAULTPOWERSETTING 2
#define DEFAULTTEMP 95
#define DEFAULTTIMER 30

// Pinout definitions
#define PINPWM 6
#define PINK 7
#define PINPAN 9
#define PINFAN 8
#define PINVAD 14
#define PINIAD 16
#define PINTMAIN 15
#define PINDAT 11
#define PINCLK 13
#define PINLEDENABLE 2
#define PINDIG1ENABLE 3
#define PINDIG2ENABLE 4
#define PINDIG3ENABLE 5
#define PINBTNS 19

// Protection
#define MINPWM 50
#define MAXPWM 250
#define MINCURRENT 400
#define MAXCURRENT 950

// Bytes to be sent to the shift register to display numbers 
#define DIGITOFF 255
const byte digits[] = {40, 235, 50, 162, 225, 164, 36, 234, 32, 224};

// LEDs and their position in the shift register byte
#define LEDPOWER 1
#define LEDTIMERMODE 2
#define LEDTEMPMODE 3
#define LEDPOWERMODE 4
#define LEDPOWERDISPLAY 5
#define LEDTEMPDISPLAY 6
#define LEDTIMERDISPLAY 7

// Steinhart-Hart values for thermistor
#define SHA 0.0006046968952860238
#define SHB 0.00022993592839436265
#define SHC 6.689628816274734e-8

SimpleTimer simpleTimer;

// Current feedback values for different power settings
const long powerSettings[] = {0, 460, 530, 600, 670, 740, 810, 880, 950} ;
int powerSetting = DEFAULTPOWERSETTING;
long timerSetting = DEFAULTTIMER;

// Power PID gains
double powerP = 0.1, powerI = 0.05, powerD = 0.01;
double powerPidIn, powerPidOut = DEFAULTPWM, powerPidSet = powerSettings[DEFAULTPOWERSETTING];
// Proportional on Measurement, and direct control, not inverse
PID powerPid(&powerPidIn, &powerPidOut, &powerPidSet, powerP, powerI, powerD, P_ON_M, DIRECT);

// Temp PID gains
double tempP = 10, tempI = 10, tempD = 2;
double tempPidIn, tempPidSet = DEFAULTTEMP;
// Proportional on Measurement, and direct control, not inverse
// Output is the setpoint for the power PID.
PID tempPid(&tempPidIn, &powerPidSet, &tempPidSet, tempP, tempI, tempD, P_ON_M, DIRECT);

// These are for tracking what's on.
// 0 is off, 1 is on but not displayed, and 2 is on and displayed.
int powerState = 0;
int tempState = 0;
int timerState = 0;
// onState doesn't have a mode 2
int onState = 0;

// Tracking instances of cook timer and fan shutoff timer so we can delete them
// If the timer is deleted, set this to -1 so we don't accidentally delete a different timer.
int timerInstance = -1;
int fanInstance = -1;

// 2 is power LED on
byte ledState = 2;
byte digitOneState = DIGITOFF;
byte digitTwoState = DIGITOFF;
byte digitThreeState = DIGITOFF;

// The LED currently being driven on
// 0 is LEDs, 1 is 1s place digit, 2 is 10s place digit, 3 is 100s place digit
int currentLedHolder = 0;

bool buttonDown = false;

void setup() {
  // Initialize serial
  Serial.begin(115200);
  // Initialize Shift Register SPI
  startSpi();
  // Initialize everything else
  initialize();
}
  
void loop() {
  simpleTimer.run();
  updateLeds();
  runPowerPid();
  if (tempState) {
    runTempPid();
  }
}

void initialize() {
  // Initialize LED pins
  pinMode(PINLEDENABLE, OUTPUT);
  digitalWrite(PINLEDENABLE, LOW);
  pinMode(PINDIG1ENABLE, OUTPUT);
  digitalWrite(PINDIG1ENABLE, HIGH);
  pinMode(PINDIG2ENABLE, OUTPUT);
  digitalWrite(PINDIG2ENABLE, HIGH);
  pinMode(PINDIG3ENABLE, OUTPUT);
  digitalWrite(PINDIG3ENABLE, HIGH);
  // Initialize other pins
  pinMode(PINPWM, OUTPUT);
  digitalWrite(PINPWM, LOW);
  pinMode(PINK, OUTPUT);
  digitalWrite(PINK, LOW);
  pinMode(PINFAN, OUTPUT);
  digitalWrite(PINFAN, LOW);
  pinMode(PINVAD, INPUT);
  pinMode(PINIAD, INPUT);
  pinMode(PINTMAIN, INPUT);
  pinMode(PINBTNS, INPUT);
  // Tell the power PID algorithm that the output will be capped.
  powerPid.SetOutputLimits(MINPWM, MAXPWM);
  // Sample as quickly as possible
  powerPid.SetSampleTime(0);
  // Tell the temp PID algorithm that the output will be capped.
  tempPid.SetOutputLimits(400, 950);
  // Sample every half second
  tempPid.SetSampleTime(500);
  // Run a monitor and reporter every second
  simpleTimer.setInterval(2000, monitor);
}

void startSpi() {
  pinMode(PINDAT, OUTPUT);
  pinMode(PINCLK, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(32000000, LSBFIRST, SPI_MODE0));
}

void enable() {
  // TODO check impedance to see if pan is in place
  // Start PWM on output control
  analogWrite(PINPWM, DEFAULTPWM);
  // Start the fan
  digitalWrite(PINFAN, HIGH);
  // Drive K High to enable cooktop
  digitalWrite(PINK, HIGH);
  onState = 1;
  // Delete the timer fan so it doesn't shut off on us.
  // This could happen if the machine was shut off, then back on within 30s.
  if (simpleTimer.isEnabled(fanInstance)) {
    simpleTimer.deleteTimer(fanInstance);
    fanInstance = -1;
  }
  simpleTimer.setTimeout(600, strikePan);
}

void disable() {
  // Drive K low to disable cooktop
  digitalWrite(PINK, LOW);
  // Stop PWM output control
  digitalWrite(PINPWM, LOW);
  fanInstance = simpleTimer.setTimeout(30000, turnOffFan);
  onState = 0;
  pausePowerPid();
}

void strikePan() {
  // Strike coil oscillation
  pinMode(PINPAN, OUTPUT);
  digitalWrite(PINPAN, LOW);
  delayMicroseconds(20);
  pinMode(PINPAN, INPUT);
  // Start power PID in 2 seconds.
  // If we start it too soon, we might lose oscillation.
  simpleTimer.setInterval(2000, initPowerPid);  
}

int getTemp(int v) {
  // Steinhart-Hart
  // https://rusefi.com/articles/measuring_temperature/Steinhart-Hart.shtml
  float r = 4900 * (1023.0 / float(v) - 1.0);
  return (1 / (SHA + SHB * log(r) + SHC * log(r) * log(r) * log(r))) - 273.15;
}

int analogDenoise(int t, int n, int p) {
  // Average three reads
  int a = analogRead(p);
  int b = analogRead(p);
  int c = analogRead(p);
  int d = (a + b + c) / 3;
  // Set deadband to ignore noise
  if (d >= t - n && d <= t + n) {
    return t;
  }
  else {
    return d;
  }
}

void monitor() {
  // TODO make sure voltage is within range (629 is normal)
  int vad = analogRead(PINVAD);
  int iad = analogRead(PINIAD);
  // If we've been running for more than 3 seconds and the output power is low,
  //  pause the power PID and strike again.
  if (millis() > 3000 && iad < 60) {
    powerPidOut = DEFAULTPWM;
    analogWrite(PINPWM, DEFAULTPWM);
    pausePowerPid();
    strikePan();
  }
  // TODO make sure temperature is within range
  int tmain = analogRead(PINTMAIN);
  if (DEBUG > 0) {
    Serial.print("Current feedback: ");
    Serial.println(iad);
    Serial.print("Temp feedback: ");
    Serial.print(tmain);
    Serial.print(", ");
    Serial.println(getTemp(tmain));
    Serial.print("Output control: ");
    Serial.println(powerPidOut);
    Serial.print("Power target: ");
    Serial.println(powerPidSet);
    Serial.print("Temp PID current parameters: ");
    Serial.print(tempPid.GetKp());
    Serial.print(", ");
    Serial.print(tempPid.GetKi());
    Serial.print(", ");
    Serial.println(tempPid.GetKd());
    Serial.print("Power PID current parameters: ");
    Serial.print(powerPid.GetKp());
    Serial.print(", ");
    Serial.print(powerPid.GetKi());
    Serial.print(", ");
    Serial.println(powerPid.GetKd());
  }
  // TODO also monitor IGBT temperature
  // Check if serial is available to adjust the PID tunings
  if (Serial.available() > 0) {
    char b = Serial.read();
    if (b == 'p') {
      String incoming = Serial.readString();
      tempP = incoming.toDouble();
      tempPid.SetTunings(tempP, tempI, tempD);
    }
    else if (b == 'i') {
      String incoming = Serial.readString();
      tempI = incoming.toDouble();
      tempPid.SetTunings(tempP, tempI, tempD);
    }
    else if (b == 'd') {
      String incoming = Serial.readString();
      tempD = incoming.toDouble();
      tempPid.SetTunings(tempP, tempI, tempD);
    }
    else if (b == 'a') {
      String incoming = Serial.readString();
      powerP = incoming.toDouble();
      powerPid.SetTunings(powerP, powerI, powerD);
    }
    else if (b == 'b') {
      String incoming = Serial.readString();
      powerI = incoming.toDouble();
      powerPid.SetTunings(powerP, powerI, powerD);
    }
    else if (b == 'c') {
      String incoming = Serial.readString();
      powerD = incoming.toDouble();
      powerPid.SetTunings(powerP, powerI, powerD);
    }
  }
}

void initPowerPid() {
  powerPid.SetMode(AUTOMATIC);
}

void runPowerPid() {
  powerPidIn = analogDenoise(powerPidSet, 20, PINIAD);
  powerPid.Compute();
  int o = powerPidOut;
  if (o < MINPWM) {
    o = MINPWM;
  }
  else if (o > MAXPWM) {
    o = MAXPWM;
  }
  analogWrite(PINPWM, o);
}

void pausePowerPid() {
  powerPid.SetMode(MANUAL);
}

void runTempPid() {
  tempPidIn = analogDenoise(tempPidSet, 2, PINTMAIN);
  tempPidIn = getTemp(tempPidIn);
  tempPid.Compute();
  if (powerPidSet < MINCURRENT) {
    powerPidSet = MINCURRENT;
  }
  else if (powerPidSet > MAXCURRENT) {
    powerPidSet = MAXCURRENT;
  }
}

void turnOffFan() {
  digitalWrite(PINFAN, LOW);
  fanInstance = -1;
}

void updateLeds() {
  // Turn off the last LED holder, poll buttons, write the state of the new
  //  holder, and turn it on.
  // Note that the LEDs have polarity opposite that of the digits.
  switch (currentLedHolder) {
    case 0:
      digitalWrite(PINDIG3ENABLE, HIGH);
      pollButtons();
      SPI.transfer(ledState);
      digitalWrite(PINLEDENABLE, HIGH);
      currentLedHolder = 1;
      break;
    case 1:
      digitalWrite(PINLEDENABLE, LOW);
      pollButtons();
      SPI.transfer(digitOneState);
      digitalWrite(PINDIG1ENABLE, LOW);
      currentLedHolder = 2;
      break;
    case 2:
      digitalWrite(PINDIG1ENABLE, HIGH);
      pollButtons();
      SPI.transfer(digitTwoState);
      digitalWrite(PINDIG2ENABLE, LOW);
      currentLedHolder = 3;
      break;
    case 3:
      digitalWrite(PINDIG2ENABLE, HIGH);
      pollButtons();
      SPI.transfer(digitThreeState);
      digitalWrite(PINDIG3ENABLE, LOW);
      currentLedHolder = 0;
      break;
  }
}

void pollButtons() {
  // There are 6 buttons.
  // 3 are tied between the data pin and one of the first three output pins
  //  on the shift register.
  // 3 are tied between a pullup and one of the first three output pins on
  //  the shift register.
  // We send a zero, then read the data and pullup pins, then shift 1 so the
  //  zero is on the second pin, read again, then shift and read again.
  // End SPI so we can control the pins manually.
  SPI.transfer(255);
  SPI.endTransaction();
  SPI.end();
  int aa, ab, ba, bb, ca, cb;  
  // Set clock to low before we begin
  digitalWrite(PINCLK, LOW);
  // Send zero
  digitalWrite(PINDAT, LOW);
  digitalWrite(PINCLK, HIGH);
  // Read data pin
  pinMode(PINDAT, INPUT);
  aa = digitalRead(PINDAT);
  // Read pullup pin;
  ab = analogRead(PINBTNS);
  digitalWrite(PINCLK, LOW);
  pinMode(PINDAT, OUTPUT);
  digitalWrite(PINDAT, HIGH);
  // Send one
  digitalWrite(PINCLK, HIGH);
  // Read data pin
  pinMode(PINDAT, INPUT);
  ba = digitalRead(PINDAT);
  // Read pullup pin;
  bb = analogRead(PINBTNS);
  digitalWrite(PINCLK, LOW);
  pinMode(PINDAT, OUTPUT);
  digitalWrite(PINDAT, HIGH);
  // Send one
  digitalWrite(PINCLK, HIGH);
  // Read data pin
  pinMode(PINDAT, INPUT);
  ca = digitalRead(PINDAT);
  // Read pullup pin;
  cb = analogRead(PINBTNS);
  digitalWrite(PINCLK, LOW);
  pinMode(PINDAT, OUTPUT);
  digitalWrite(PINDAT, HIGH);
  if (aa == 0) {
    if (!buttonDown) {
      if (DEBUG > 0) {
        Serial.println("Power Button");
      }
      buttonDown = true;
      // If the machine is on, turn it off
      if (onState) {
        setPowerState(0);
        setTempState(0);
        setTimerState(0);
        disable();
      }
      // If it's off, turn it on.
      else {
        setPowerState(2);
        enable();
      }
    }
  }
  else if (ba == 0) {
    if (!buttonDown) {
      if (DEBUG > 0) {
        Serial.println("Minus Button");
      }
      buttonDown = true;
      if (onState) {
        // Decrement the proper setting
        if (powerState == 2 && powerSetting > 1) {
          powerSetting--;
          powerPidSet = powerSettings[powerSetting];
          showNumber(powerSetting);
        }
        else if (tempState == 2 && tempPidSet > 0) {
          tempPidSet--;
          showNumber(tempPidSet);
        }
        else if (timerState == 2 && timerSetting > 1) {
          timerSetting--;
          showNumber(timerSetting);
        }
      }
    }
  }
  else if (ca == 0) {
    if (!buttonDown) {
      if (DEBUG > 0) {
        Serial.println("Plus Button");
      }
      buttonDown = true;
      if (onState) {
        // Increment the proper setting
        if (powerState == 2 && powerSetting < 8) {
          powerSetting++;
          powerPidSet = powerSettings[powerSetting];
          showNumber(powerSetting);
        }
        else if (tempState == 2 && tempPidSet < 480) {
          tempPidSet++;
          showNumber(tempPidSet);
        }
        else if (timerState == 2 && timerSetting < 999) {
          timerSetting++;
          showNumber(timerSetting);
        }
      }
    }
  }
  else if (ab < 500) {
    if (!buttonDown) {
      if (DEBUG > 0) {
        Serial.println("Timer Mode Button");
      }
      buttonDown = true;
      if (onState) {
        setTimerState(2);
      }
    }
  }
  else if (bb < 500) {
    if (!buttonDown) {
      if (DEBUG > 0) {
        Serial.println("Temp Mode Button");
      }
      buttonDown = true;
      if (onState) {
        setTempState(2);
      }
    }
  }
  else if (cb < 500) {
    if (!buttonDown) {
      if (DEBUG > 0) {
        Serial.println("Power Mode Button");
      }
      buttonDown = true;
      if (onState) {
        setPowerState(2);
      }
    }
  }
  else {
    buttonDown = false;
  }
  // Restart SPI after we stopped it.
  startSpi();
}

void ledOn(int l) {
  ledState |= 1 << l;
}

void ledOff(int l) {
  ledState &= ~(1 << l);
}

void showNumber(long n) {
  // 1s place
  int a = n % 10;
  // 10s place
  int b = (n / 10) % 10;
  // 100s place
  int c = (n / 100) % 10;
  // Only show a number if the places above it are non-zero,
  //  other wise we'd have 002 when we try to show 2.
  if (a + b + c > 0) {
    digitOneState = digits[a];
    if (b + c > 0) {
      digitTwoState = digits[b];
      if (c > 0) {
        digitThreeState = digits[c];
      }
      else {
        digitThreeState = DIGITOFF;
      }
    }
    else {
      digitTwoState = DIGITOFF;
      digitThreeState = DIGITOFF;
    }
  }
  else {
    digitOneState = DIGITOFF;
    digitTwoState = DIGITOFF;
    digitThreeState = DIGITOFF;
  }
}

void setPowerState(int l) {
  if (l == 0) {
    ledOff(LEDPOWERMODE);
    if (powerState == 2) {
      ledOff(LEDPOWERDISPLAY);
      showNumber(0);
    }
    powerState = 0;
  }
  else if (l > 0) {
    powerPidSet = powerSettings[powerSetting];
    setTempState(0);
    ledOn(LEDPOWERMODE);
    if (l == 1) {
      if (powerState == 2) {
        ledOff(LEDPOWERDISPLAY);
        showNumber(0);
      }
      powerState = 1;
    }
    else if (l == 2) {
      setTimerState(0);
      ledOn(LEDPOWERDISPLAY);
      showNumber(powerSetting);
      powerState = 2;
    }
  }
}

void setTempState(int l) {
  if (l == 0) {
    ledOff(LEDTEMPMODE);
    tempPid.SetMode(MANUAL);
    if (tempState == 2) {
      ledOff(LEDTEMPDISPLAY);
      showNumber(0);
    }
    tempState = 0;
  }
  if (l > 0) {
    setPowerState(0);
    ledOn(LEDTEMPMODE);
    if (l == 1) {
      tempPid.SetMode(AUTOMATIC);
      if (tempState == 2) {
        ledOff(LEDTEMPDISPLAY);
        showNumber(0);
      }
      tempState = 1;
    }
    else if (l == 2) {
      setTimerState(0);
      tempPid.SetMode(AUTOMATIC);
      ledOn(LEDTEMPDISPLAY);
      showNumber(tempPidSet);
      tempState = 2;
    }
  }
}

void setTimerState(int l) {
  if (l == 0) {
    if (simpleTimer.isEnabled(timerInstance)) {
      simpleTimer.deleteTimer(timerInstance);
      timerInstance = -1;
    }
    ledOff(LEDTIMERMODE);
    if (timerState == 2) {
      ledOff(LEDTIMERDISPLAY);
      showNumber(0);
    }
    timerState = 0;
  }
  else if (l == 1) {
    if (timerState == 0) {
      timerInstance = simpleTimer.setTimeout(60000, timerDown);
    }
    else if (timerState == 2) {
      ledOff(LEDTIMERDISPLAY);
      showNumber(0);
    }
    ledOn(LEDTIMERMODE);
    timerState = 1;
  }
  else if (l == 2) {
    if (timerState == 0) {
      timerInstance = simpleTimer.setTimeout(60000, timerDown);
    }
    if (powerState == 2) {
      setPowerState(1);
    }
    if (tempState == 2){
      setTempState(1);
    }
    ledOn(LEDTIMERMODE);
    ledOn(LEDTIMERDISPLAY);
    showNumber(timerSetting);
    timerState = 2;
  }
}

void timerDown() {
  timerInstance = -1;
  if (timerState == 2) {
    timerSetting--;
    showNumber(timerSetting);
    // If timer has run down, shut off the machine.
    if (timerSetting == 0) {
      setPowerState(0);
      setTempState(0);
      setTimerState(0);
      disable();
    }
    else {
      // Count out another minute.
      timerInstance = simpleTimer.setTimeout(60000, timerDown);
    }
  }
}
