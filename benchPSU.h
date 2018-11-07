#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Encoder.h>
#include <version.h>

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)  Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)   Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
#endif

/*
Pins:
    in: Rotary-Voltage
        Rotary-Current
        Voltage-Sens
        Current-Sens
    out: Voltage-Pot
         Current-Pot
         Voltage/Current limit LED
         LCD (5)
    TOTAL: 12
LCD:
    The circuit:
    * LCD RS pin to digital pin 4
    * LCD Enable pin to digital pin 9
    * LCD D4 pin to digital pin 8
    * LCD D5 pin to digital pin 7
    * LCD D6 pin to digital pin 6
    * LCD D7 pin to digital pin 5

    * LCD R/W pin to ground
    * LCD VSS pin to ground
    * LCD VCC pin to 5V
    * 10K resistor:
    * ends to +5V and ground
    * wiper to LCD VO pin (pin 3)

    initialize the library with the numbers of the interface pins
    LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
*/
//                        D  D  D  D
//                RS  EN  4  5  6  7

// CONSTANTS
#define loopdelay 100
#define uPRECISION 1023  // Number of divisions in Voltage range (30V)
#define iPRECISION 1023  // Number of divisions in Current range (3A)
#define uMAX 29.6
#define iMAX 2
//#define averageTicks 4  // number of "ticks" (main loop runs) over which the measured values will be averaged
#define oversamplingBits 1
#define modeChangeTicks 10  // number of ticks the calibration button needs to be pressed for mode change
#define uPwmPin 10      // output pin for voltage control signal
#define iPwmPin 9       // output pin for current control signal
#define enableOutSwitch 6   // physical toggle switch to enable / disable output (via uC)
#define modeBtn 12
#define enablePSUout 11     // output pin for gate-signal for relay that switches the output
#define fanPWM 8            // output pin for cooling fan control signal
#define uSensPin A6         // input pin for voltage sensing signal
#define iSensPin A7         // input pin for current sensing signal
#define tempSens A0
enum units {Uset, Iset, Usens, Isens};  // used for jumping to prominent positions on LCD without needing coordinates
enum calDat {cd_umax, cd_imax, cd_usmin, cd_ismin, cd_usmax, cd_ismax, cd_idispmin, cd_udispmin};  // used for calibrationData access

// Structs
struct numSpace{
public:
    float lower;
    float upper;
    numSpace(float _min, float _max) : lower(_min), upper(_max) {}
    numSpace(){}
    void setLimits(float _lower, float _upper){lower = _lower; upper = _upper;}
};

// PSU I/O Objects
LiquidCrystal lcd(7, A1, A2, A3, A4, A5);   // free A0 for thermometer
Encoder voltageAdjustKnob(3,5);
Encoder currentAdjustKnob(2,4);

// global Variables
float uSet = 0;
float iSet = 0;
String sw_version = "";
bool calibrationLoaded = false;
long uAdjustPosition = 0;   // internal value for voltage control signal (eventually mapped to 10 bit uint for timer1 duty cycle PWM output)
float dispSetVoltage = 0;   // voltage control signal scaled to 0 - 30
long iAdjustPosition = 0;   // internal value for current control signal (eventually mapped to 10 bit uint for timer1 duty cycle PWM output)
float dispSetCurrent = 0;   // current control signal scaled to 0 - 3
int intSensedVoltage = 0;   // raw analogRead value from voltage sense pin
int intSensedCurrent = 0;   // raw analogRead value from current sense pin
uint8_t uUpdateCount = 0;   // variable to keep track of averaging steps for voltage sensing
uint8_t iUpdateCount = 0;   // variable to keep track of averaging steps for current sensing
bool emergencyShutdownState = false;
long timekeeper = -1;
/*
[0] = RE(Umax); [1] = RE(laV); [2] = RE(Imax); [3] = RE(laC); [4] = VoltageSensMin;
[5] = CurrentSensMin; [6] = VoltageSensMax; [7] = CurrentSensMax;
*/
float calibrationData[8];
uint8_t mode = 0;   // 0 = Normal 1 = Calibration
uint16_t modeBtnCount = 0;
numSpace uKnobSpace;
numSpace uPwmSpace;
numSpace uDisplaySpace;
numSpace uSensSpace;
numSpace iKnobSpace;
numSpace iPwmSpace;
numSpace iDisplaySpace;
numSpace iSensSpace;


// functions
void initPSU();
void initLCD();
void modeButtonEval();
void getSwVersion();
bool getCalibrationData();
void storeCalibrationData();
void goLcd(uint8_t);
void updateSetVoltage();
void updateSetCurrent();
void updateSensVoltage();
void updateSensCurrent();
void enableOutput();
void disableOutput();
void emergencyShutdown(String);
void clearEmergency();
void resetOutputs();
void calibrate();
void initSpaces();
float floatMap(double, double, double, double, double);
float transformSpace(double, numSpace, numSpace);
