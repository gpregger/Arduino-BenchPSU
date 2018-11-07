#include <Arduino.h>
#include "benchPSU.h"
#include <TimerOne.h>
#include <LiquidCrystal.h>
#include <Encoder.h>
#include <EEPROM.h>

// set up various onboard configuration (pins and timer)
void setup()
{
    pinMode(enablePSUout, OUTPUT);
    digitalWrite(enablePSUout, LOW);
    pinMode(fanPWM, OUTPUT);
    pinMode(uPwmPin, OUTPUT);
    pinMode(iPwmPin, OUTPUT);
    pinMode(modeBtn, INPUT_PULLUP);
    Timer1.initialize(40);              // initialize timer1 used for output PWM waves at 25kHz
    Timer1.pwm(fanPWM, 512);            // set fan to 50%
    Timer1.pwm(uPwmPin, 0);             // initialize voltage and current limit to 0
    Timer1.pwm(iPwmPin, 0);
    pinMode(uSensPin, INPUT);
    pinMode(iSensPin, INPUT);
    initPSU();
    Serial.begin(9600);
}

// main loop
void loop()
{
    if (!(digitalRead(enableOutSwitch) == digitalRead(enablePSUout)))
        digitalRead(enableOutSwitch) ? enableOutput() : disableOutput();

    // Write Set Voltage to LCD
    updateSetVoltage();

    // Write Set Current to LCD
    updateSetCurrent();

    // Write Sensed Voltage to LCD
    updateSensVoltage();

    // Write Sensed Current to LCD
    updateSensCurrent();

    if (!digitalRead(modeBtn))
        modeButtonEval();

    delay(loopdelay);
}

// initialize unit (welcome screen, calibration data, lcd initialization)
void initPSU()
{
    DEBUG_PRINTLN("void initPSU()");
    lcd.begin(16, 2);
    lcd.noBlink();
    lcd.setCursor(0, 0);
    getSwVersion();
    if (getCalibrationData())
        lcd.print("  30V/3A  vPSU  ");
    else
        lcd.print(" 30V/3A  vPSU !C");
    lcd.setCursor(1, 1);
    lcd.print("  SW");
    lcd.print(sw_version);
    initSpaces();
    delay(1500);
    initLCD();
}

void getSwVersion()
{
    sw_version += __DATE__[9];
    sw_version += __DATE__[10];
    sw_version += '.';
    if (BUILD_MONTH < 10)
    {
        sw_version += '0';
        sw_version += BUILD_MONTH;
    }
    else
        sw_version += BUILD_MONTH;
    sw_version += '.';
    if (BUILD_DAY < 10)
    {
        sw_version += '0';
        sw_version += BUILD_DAY;
    }
    else
        sw_version += BUILD_DAY;
}

// init LCD and put current / voltage symbols on screen
void initLCD()
{
    DEBUG_PRINTLN("void initLCD()");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("U*:");
    dispSetVoltage < 10 ? lcd.print(dispSetVoltage, 3) : lcd.print(dispSetVoltage, 2);
    lcd.setCursor(9, 0);
    lcd.print("U:");
    lcd.setCursor(0, 1);
    lcd.print("I*:");
    lcd.print(dispSetCurrent, 3);
    lcd.setCursor(9, 1);
    lcd.print("I:");
    lcd.setCursor(0, 0);
}

// put cursor to appropriate section of the screen
void goLcd(uint8_t unit)
{
    switch (unit)
    {
        case Uset: lcd.setCursor(3, 0); break;
        case Iset: lcd.setCursor(3, 1); break;
        case Usens: lcd.setCursor(11, 0); break;
        case Isens: lcd.setCursor(11, 1); break;
        default: break;
    }
}

void modeButtonEval()
{
    while (!digitalRead(modeBtn))
    {
        modeBtnCount++;
        if (modeBtnCount > 50) break;
        delay(loopdelay);
    }
    if (modeBtnCount < 10)
    {
        resetOutputs();
        lcd.clear();
        initLCD();
    }
    else if (modeBtnCount > 50)
    {
        lcd.clear();
        lcd.begin(16, 2);
        initLCD();
    }
    else
    {
        mode = 1;
        calibrate();
    }
    modeBtnCount = 0;
    while (!digitalRead(modeBtn));
}

// check rotary encoders for change and update output parameters accordingly
void updateSetVoltage()
{
    // Determine Voltage to be set
    long newVoltagePos = voltageAdjustKnob.read();
    uSet += newVoltagePos * 0.1;

    if (uSet > uDisplaySpace.upper)
        uSet = uDisplaySpace.upper;
    if (uSet < uDisplaySpace.lower)
        uSet = uDisplaySpace.lower;
    voltageAdjustKnob.write(0);

    // Apply Voltage at output
    Timer1.setPwmDuty(uPwmPin, int(transformSpace(uSet, uDisplaySpace, uPwmSpace)));

    // Write Voltage to screen
    if (!mode)
    {
        goLcd(Uset);
        uSet < 10 ? lcd.print(uSet, 3) : lcd.print(uSet, 2);
    }
}

// check rotary encoders for change and update output parameters accordingly
void updateSetCurrent()
{
    // Determine Current to be set
    long newCurrentPos = currentAdjustKnob.read();
    iSet += newCurrentPos * 0.01;

    if (iSet > iDisplaySpace.upper)
        iSet = iDisplaySpace.upper;
    if (iSet < iDisplaySpace.lower)
        iSet = iDisplaySpace.lower;
    currentAdjustKnob.write(0);

    // Apply Current at output
    Timer1.setPwmDuty(iPwmPin, int(transformSpace(iSet, iDisplaySpace, iPwmSpace)));

    // Write Current to screen
    if (!mode)
    {
        goLcd(Iset);
        lcd.print(iSet, 3);
    }
}

// Sample and - if applicable - display voltage measurement
void updateSensVoltage()
{
    if (uUpdateCount == pow(4, oversamplingBits))
    {
        float sensedVoltage = transformSpace(intSensedVoltage / pow(4, oversamplingBits), uSensSpace, uDisplaySpace);
        if (sensedVoltage > (uMAX+2) && digitalRead(enableOutSwitch))
        {
            emergencyShutdown("Overvoltage");
            return;
        }
        goLcd(Usens);
        sensedVoltage < 10 ? lcd.print(sensedVoltage, 3) : lcd.print(sensedVoltage, 3);
        uUpdateCount = 0;
        intSensedVoltage = 0;
    }
    else
    {
        intSensedVoltage += analogRead(uSensPin);
        uUpdateCount++;
    }
}

// Sample and - if applicable - display current measurement
void updateSensCurrent()
{
    if (iUpdateCount == pow(4, oversamplingBits))
    {
        float sensedCurrent = transformSpace(intSensedCurrent / pow(4, oversamplingBits), iSensSpace, iDisplaySpace);
        if (sensedCurrent > (iMAX+0.5) && digitalRead(enableOutSwitch))
        {
            emergencyShutdown("Overcurrent");
            return;
        }
        goLcd(Isens);
        lcd.print(sensedCurrent, 3);
        iUpdateCount = 0;
        intSensedCurrent = 0;
    }
    else
    {
        intSensedCurrent += analogRead(iSensPin);
        iUpdateCount++;
    }
}

// turn on output relay
void enableOutput()
{
    DEBUG_PRINTLN("void enableOutput()");
    digitalWrite(enablePSUout, HIGH);
}

// turn off output relay
void disableOutput()
{
    DEBUG_PRINTLN("void disableOutput()");
    digitalWrite(enablePSUout, LOW);
}

// triggered on (measured) over-voltage or over-current
void emergencyShutdown(String reason)
{
    DEBUG_PRINTLN("void emergencyShutdown()");
    disableOutput();
    Timer1.setPwmDuty(fanPWM, 1023);
    emergencyShutdownState = true;
    resetOutputs();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("! EmSd cause:");
    lcd.setCursor(0, 1);
    lcd.print(reason);
    while(digitalRead(enableOutSwitch));
    initLCD();
    Timer1.setPwmDuty(fanPWM, 512);
    emergencyShutdownState = false;
}

// set all output parameters to 0
void resetOutputs()
{
    DEBUG_PRINTLN("void resetOutputs()");
    Timer1.setPwmDuty(uPwmPin, 0);
    Timer1.setPwmDuty(iPwmPin, 0);
    voltageAdjustKnob.write(0);
    currentAdjustKnob.write(0);
    uSet = 0;
    iSet = 0;
}

// runtime calibration routine
void calibrate()
{
    DEBUG_PRINTLN("void calibrate()");
    resetOutputs();
    disableOutput();
    calibrationLoaded = false;
    initSpaces();
    uint16_t uCalToStore = 0;
    uint16_t iCalToStore = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  Calibration  ");
    lcd.setCursor(0, 1);
    lcd.print("U/I:  0-30V 0-2A");
    delay(2000);

    calibrationData[cd_usmin] = analogRead(uSensPin);
    DEBUG_PRINTLN(calibrationData[cd_usmin]);
    calibrationData[cd_ismin] = analogRead(iSensPin);
    DEBUG_PRINTLN(calibrationData[cd_ismin]);

    lcd.clear();
    enableOutput();

    // Upper voltage limit
    while (!digitalRead(modeBtn));
    lcd.setCursor(0, 0);
    lcd.print("Upper U Limit:");
    lcd.setCursor(6, 1);
    lcd.print(uMAX);
    lcd.print("V");
    while (digitalRead(modeBtn))
    {
        updateSetVoltage();
        delay(loopdelay);
    }

    calibrationData[cd_usmax] = analogRead(uSensPin);
    DEBUG_PRINTLN(calibrationData[cd_usmax]);
    calibrationData[cd_umax] = transformSpace(uSet, uDisplaySpace, uPwmSpace);
    DEBUG_PRINTLN(calibrationData[cd_umax]);

    resetOutputs();
    lcd.clear();

    // Lower voltage limit
    while (!digitalRead(modeBtn));
    lcd.setCursor(0, 0);
    lcd.print("Set Umin:");
    lcd.setCursor(7, 1);
    lcd.print("0.000V");
    float dispSetCalVoltage;
    while (digitalRead(modeBtn))
    {
        long uCalPos = voltageAdjustKnob.read();
        dispSetCalVoltage += uCalPos * 0.01;
        voltageAdjustKnob.write(0);
        lcd.setCursor(7, 1);
        dispSetCalVoltage < 10 ? lcd.print(dispSetCalVoltage, 3) : lcd.print(dispSetCalVoltage, 2);
        delay(loopdelay);
    }
    calibrationData[cd_udispmin] = dispSetCalVoltage;
    DEBUG_PRINTLN(calibrationData[cd_udispmin]);
    resetOutputs();
    lcd.clear();

    // Upper current Limit
    // fix voltage at 1/3 for current limit
    Timer1.setPwmDuty(uPwmPin, calibrationData[cd_umax] / 3);
    while (!digitalRead(modeBtn));
    lcd.setCursor(0, 0);
    lcd.print("Upper I Limit:");
    lcd.setCursor(6, 1);
    lcd.print(iMAX);
    lcd.print("A");
    while (digitalRead(modeBtn))
    {
        updateSetCurrent();
        delay(loopdelay);
    }

    calibrationData[cd_ismax] = analogRead(iSensPin);
    DEBUG_PRINTLN(calibrationData[cd_ismax]);
    calibrationData[cd_imax] = transformSpace(iSet, iDisplaySpace, iPwmSpace);
    DEBUG_PRINTLN(calibrationData[cd_imax]);

    resetOutputs();
    lcd.clear();

    // lower current Limit
    Timer1.setPwmDuty(uPwmPin, calibrationData[cd_umax] / 3);
    while (!digitalRead(modeBtn));
    lcd.setCursor(0, 0);
    lcd.print("Set Imin:");
    lcd.setCursor(7, 1);
    lcd.print("0.000A");
    float dispSetCalCurrent;
    while (digitalRead(modeBtn))
    {
        long iCalPos = currentAdjustKnob.read();
        dispSetCalCurrent += iCalPos * 0.001;
        currentAdjustKnob.write(0);
        lcd.setCursor(7, 1);
        lcd.print(dispSetCalCurrent, 3);
        delay(loopdelay);
    }
    calibrationData[cd_idispmin] = dispSetCalCurrent;
    DEBUG_PRINTLN(calibrationData[cd_idispmin]);

    storeCalibrationData();
    getCalibrationData();
    initSpaces();
    resetOutputs();
    initLCD();
    digitalRead(enableOutSwitch) && !emergencyShutdownState ? enableOutput() : disableOutput();
    mode = 0;
}

// load calibration data from EEPROM
// return false if no calibration data is found (all 0)
bool getCalibrationData()
{
    int address = 0;

    for (uint8_t i = 0; i < sizeof(calibrationData) / sizeof(calibrationData[cd_umax]); i++)
    {
        EEPROM.get(address, calibrationData[i]);
        address += sizeof(float);
    }
    if (calibrationData[cd_umax] != 0)
    {
        calibrationLoaded = true;
        return true;
    }
    else
    {
        DEBUG_PRINTLN("calibration data NULL");
        calibrationLoaded = false;
        return false;
    }
}

// store calibration data in EEPROM
void storeCalibrationData()
{
    DEBUG_PRINTLN("bool storeCalibrationData()");
    int address = 0;

    for (uint8_t i = 0; i < sizeof(calibrationData) / sizeof(calibrationData[cd_umax]); i++)
    {
        EEPROM.put(address, calibrationData[i]);
        address += sizeof(float);
    }
}

void initSpaces()
{
    if (calibrationLoaded)
    {
        uPwmSpace.setLimits(0, calibrationData[cd_umax]);
        uSensSpace.setLimits(calibrationData[cd_usmin], calibrationData[cd_usmax]);
        iPwmSpace.setLimits(0, calibrationData[cd_imax]);
        iSensSpace.setLimits(calibrationData[cd_ismin], calibrationData[cd_ismax]);
        uDisplaySpace.setLimits(calibrationData[cd_udispmin], uMAX);
        iDisplaySpace.setLimits(calibrationData[cd_idispmin], iMAX);
    }
    else
    {
        uPwmSpace.setLimits(0, 1023);
        uSensSpace.setLimits(0, 1023);
        iPwmSpace.setLimits(0, 1023);
        iSensSpace.setLimits(300, 600);
        uDisplaySpace.setLimits(0, uMAX);
        iDisplaySpace.setLimits(0, iMAX);
    }
}

float transformSpace(double value, numSpace source, numSpace target)
{
    return floatMap(value, source.lower, source.upper, target.lower, target.upper);
}

// reimplementation of the "map"-function but for floating point values
float floatMap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
