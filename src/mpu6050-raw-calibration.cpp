/*
 * Copyright 2019 Melika Barzegaran <melika.barzegaran.hosseini@gmail.com>
 * Copyright 2014 Luis Ródenas <luisrodenaslorda@gmail.com>
 * Copyright 2011 Jeff Rowberg <jeff@rowberg.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/*
 * title:
 *      MPU-6050 calibration
 *
 * description:
 *      Returns calibration offsets for x, y, and z axes of MPU-6050 built-in accelerometer and gyroscope.
 *
 *      These offsets were meant to calibrate MPU-6050's internal Digital Motion Processor (DMP), but they can be
 *      also useful for reading data from sensors.
 *
 *      The effect of temperature has not been taken into account. It is not guaranteed that it'll work if is
 *      calibrated indoors and used outdoors. Best is to calibrate and use at the same temperature.
 *
 *      This is a highly-edited version of MPU-6050 calibration sketch written by Luis Ródenas. It uses MPU-6050
 *      library, written by Jeff Rowberg. That MPU-6050 library, uses the I2C serial communication protocol library,
 *      which is also written by Jeff Rowberg. Finally, the I2C serial communication protocol library uses the official
 *      Arduino I2C library, called Wire.
 *
 *      The MPU-6050 calibration sketch, written by Luis Ródenas, was originally posted here:
 *          https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
 *
 *      The link to the source code of MPU-6050 library, written by Jeff Rowberg, is here:
 *          https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *
 *      The link to the source code of I2C serial protocol library, also written by Jeff Rowberg, is here:
 *          https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
 *
 *      The reference to the official Arduino I2C library, called Wire, can be found here:
 *          https://www.arduino.cc/en/reference/wire
 *
 * configuration:
 *      +-----------------------+----------------------------+
 *      | Arduino Uno board pin | GY-521 break-out board pin |
 *      +-----------------------+----------------------------+
 *      |          VCC          |             VCC            |
 *      +-----------------------+----------------------------+
 *      |          GND          |             GND            |
 *      +-----------------------+----------------------------+
 *      |           A5          |             SCL            |
 *      +-----------------------+----------------------------+
 *      |           A4          |             SDA            |
 *      +-----------------------+----------------------------+
 *
 * author:
 *      Melika Barzegaran <melika.barzegaran.hosseini@gmail.com>
 *
 * version:
 *      1.0.0
 */

/*
 * Add MPU-6050 library, written by Jeff Rowberg <jeff@rowberg.net>.
 * Link to the library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */
#include "MPU6050.h"

/*
 * Define MPU-6050 object under calibration.
 * Input parameter is the I2C serial communication protocol address for the device.
 *
 * +-------------------+---------------------------------+
 * | address pin (AD0) |        I2C device address       |
 * +-------------------+---------------------------------+
 * |  connected to GND |  0x68 (MPU6050_ADDRESS_AD0_LOW) |
 * +-------------------+---------------------------------+
 * |  connected to VCC | 0x69 (MPU6050_ADDRESS_ADO_HIGH) |
 * +-------------------+---------------------------------+
 */
MPU6050 motionTrackingDevice(MPU6050_ADDRESS_AD0_LOW); // NOLINT(cert-err58-cpp)

/*
 * Number of reading sensor values to discard.
 * Default value is set to 100.
 */
const int DISCARD_SIZE = 100;

/*
 * Number of reading sensor values used to compute average.
 * Make it higher to get more precision but the sketch will be lower.
 * Default value is set to 1000.
 */
const int BUFFER_SIZE = 1000;

/*
 * Error range allowed for accelerometer calibration offsets.
 * Make it lower to get more precision, but sketch may not converge.
 * Default value is set to 8.
 */
const int ACCELEROMETER_TOLERANCE = 8;

/*
 * Error range allowed for gyroscope calibration offsets.
 * Make it lower to get more precision, but sketch may not converge.
 * Default value is set to 1.
 */
const int GYROSCOPE_TOLERANCE = 1;

/*
 * Step value, tuned for accelerometer, used in each step of calibration.
 * Default value is set to 8.
 */
const int ACCELEROMETER_STEP = 8;

/*
 * Step value, tuned for gyroscope, used in each step of calibration.
 * Default value is set to 3.
 */
const int GYROSCOPE_STEP = 3;

/*
 * Values we prefer to read from sensors in each axes when placing the device horizontally, package letters facing
 * up, without any movement.
 *
 * The goal values for all sensors in each axes are 0, except for z axis of accelerometer. The goal value for z
 * axis of accelerometer is 16384, meaning +1 g, indicating the object is under gravity.
 */
const int16_t ACCELEROMETER_X_GOAL = 0;
const int16_t ACCELEROMETER_Y_GOAL = 0;
const int16_t ACCELEROMETER_Z_GOAL = 16384;
const int16_t GYROSCOPE_X_GOAL = 0;
const int16_t GYROSCOPE_Y_GOAL = 0;
const int16_t GYROSCOPE_Z_GOAL = 0;

/*
 * The average value of each sensor in each axis.
 * Default values are set to max int.
 */
int16_t accelerometerXAverage = INT16_MAX;
int16_t accelerometerYAverage = INT16_MAX;
int16_t accelerometerZAverage = INT16_MAX;
int16_t gyroscopeXAverage = INT16_MAX;
int16_t gyroscopeYAverage = INT16_MAX;
int16_t gyroscopeZAverage = INT16_MAX;

/*
 * The calibration offsets of each sensor in each axis.
 * Default values are set to 0.
 */
int16_t accelerometerXOffset = 0;
int16_t accelerometerYOffset = 0;
int16_t accelerometerZOffset = 0;
int16_t gyroscopeXOffset = 0;
int16_t gyroscopeYOffset = 0;
int16_t gyroscopeZOffset = 0;

void setupUartSerialCommunicationProtocol();

void setupI2cSerialCommunicationProtocol();

void setClockSource();

void setRangeAndSensitivity();

void wakeUp();

void waitForCommandToStartCalibrating();

void calibrate();

void printResult();

bool isCalibrated();

void setCalibrationOffsets();

void discardUnreliableValues();

void computeAverageValues();

void printHeader();

void printAverageValues();

void printCalibrationOffsets();

void updateCalibrationOffsetsIfNotCalibrated();

bool isAccelerometerXCalibrated();

bool isAccelerometerYCalibrated();

bool isAccelerometerZCalibrated();

bool isGyroscopeXCalibrated();

bool isGyroscopeYCalibrated();

bool isGyroscopeZCalibrated();

void updateAccelerometerXOffset();

void updateAccelerometerYOffset();

void updateAccelerometerZOffset();

void updateGyroscopeXOffset();

void updateGyroscopeYOffset();

void updateGyroscopeZOffset();

int getAccelerometerXDifference();

int getAccelerometerYDifference();

int getAccelerometerZDifference();

int getGyroscopeXDifference();

int getGyroscopeYDifference();

int getGyroscopeZDifference();

void setup() {
    setupUartSerialCommunicationProtocol();
    setupI2cSerialCommunicationProtocol();
    setClockSource();
    setRangeAndSensitivity();
    wakeUp();
    waitForCommandToStartCalibrating();
    calibrate();
    printResult();
}

void loop() {}

void setupUartSerialCommunicationProtocol() {
    /*
     * Setup UART serial communication protocol.
     *
     * General UART serial communication protocol frame format is as below:
     * +-------+------+--------+------+
     * | start | data | parity | stop |
     * +-------+------+--------+------+
     * |   1   |  5-8 |   0-1  |  1-2 |
     * +-------+------+--------+------+
     *
     * 8N1 frame format, which is the default frame format, is used. It has:
     *
     *      - 1 bit for start,
     *      - 8 bits for data,
     *      - no parity,
     *      - and 1 bit for stop.
     *
     * As a result, frames used for UART serial communication protocol are overall 10 bits and as below:
     * +-------+------+------+
     * | start | data | stop |
     * +-------+------+------+
     * |   1   |   8  |   1  |
     * +-------+------+------+
     *
     * Set baud rate to 38400 bps. 38400 bps is chosen because the baud rate for UART serial communication protocol
     * in calibration sketch be the same as other sketches. This way, there will be no need to change the baud rate
     * in serial monitor when navigating between sketches.
     */
    Serial.begin(38400);
}

void setupI2cSerialCommunicationProtocol() {
    /*
     * Setup I2C serial communication protocol.
     *
     * Set clock Frequency of I2C serial communication protocol to the default value, which is the standard mode
     * (100 KHz). To set the clock frequency to the fast mode (400 KHz), use the code below:
     *
     * ```
     * Wire.setClock(400000);
     * ```
     */
    Wire.begin();
}

void setClockSource() {
    /*
     * register name = PWR_MGMT_1 (power management 1)
     *
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * |          0x6B         |          107          |   -  |   -  |   -  |   -  |   -  |     CLKSEL[2:0]    |
     * +-----------------------+-----------------------+------+------+------+------+------+--------------------+
     *
     * +--------+---------------------------------------------------------+
     * | CLKSEL |                       clock source                      |
     * +--------+---------------------------------------------------------+
     * |    0   |                internal 8 MHz oscillator                |
     * +--------+---------------------------------------------------------+
     * |    1   |           PLL with x axis gyroscope reference           | <- selected
     * +--------+---------------------------------------------------------+
     * |    2   |           PLL with y axis gyroscope reference           |
     * +--------+---------------------------------------------------------+
     * |    3   |           PLL with z axis gyroscope reference           |
     * +--------+---------------------------------------------------------+
     * |    4   |          PLL with external 32.768 KHz reference         |
     * +--------+---------------------------------------------------------+
     * |    5   |           PLL with external 19.2 MHz reference          |
     * +--------+---------------------------------------------------------+
     * |    6   |                         reserved                        |
     * +--------+---------------------------------------------------------+
     * |    7   | stops the clock and keeps the timing generator in reset |
     * +--------+---------------------------------------------------------+
     *
     * Set clock source to use x axis of gyroscope as clock reference.
     * Selecting one of the axes of gyroscope as clock reference provides us with a more accurate clock source.
     */
    motionTrackingDevice.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
}

void setRangeAndSensitivity() {
    /*
     * register name = ACCEL_CONFIG (accelerometer configuration)
     *
     * +-----------------------+-----------------------+------+------+------+-------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 |  bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+-------+------+------+------+------+
     * |          0x1C         |           28          |   -  |   -  |   -  | AFS_SEL[1:0] |   -  |   -  |   -  |
     * +-----------------------+-----------------------+------+------+------+--------------+------+------+------+
     *
     * +---------+------------------+-------------+
     * | AFS_SEL | full-scale range | sensitivity |
     * +---------+------------------+-------------+
     * |    0    |      +/-2 g      | 16384 LSB/g | <-- selected
     * +---------+------------------+-------------+
     * |    1    |      +/-4 g      |  8192 LSB/g |
     * +---------+------------------+-------------+
     * |    2    |      +/-8 g      |  4096 LSB/g |
     * +---------+------------------+-------------+
     * |    3    |      +/-16 g     |  2048 LSB/g |
     * +---------+------------------+-------------+
     *
     * Set full-scale range for accelerometer to +/-2 g.
     * Set sensitivity for accelerometer to 16384 LSB/g.
     * Reading 16384 from accelerometer over one of its axes means the acceleration of +1 g towards that axis.
     */
    motionTrackingDevice.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    /*
     * register name = GYRO_CONFIG (gyroscope configuration)
     *
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * |          0x1B         |           27          |   -  |   -  |   -  | FS_SEL[1:0] |   -  |   -  |   -  |
     * +-----------------------+-----------------------+------+------+------+-------------+------+------+------+
     *
     * +--------+------------------+--------------+
     * | FS_SEL | full-scale range |  sensitivity |
     * +--------+------------------+--------------+
     * |    0   |    +/-250 dps    |  131 LSB/dps | <- selected
     * +--------+------------------+--------------+
     * |    1   |    +/-500 dps    | 65.5 LSB/dps |
     * +--------+------------------+--------------+
     * |    2   |    +/-1000 dps   | 32.8 LSB/dps |
     * +--------+------------------+--------------+
     * |    3   |    +/-2000 dps   | 16.4 LSB/dps |
     * +--------+------------------+--------------+
     *
     * Set full-scale range for gyroscope to +/-250 dps.
     * Set sensitivity for gyroscope to 131 LSB/dps.
     * Reading 131 from gyroscope over one of its axes means the angular speed of +1 dps around that axis.
     */
    motionTrackingDevice.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void wakeUp() {
    /*
     * register name = PWR_MGMT_1 (power management 1)
     *
     * +-----------------------+-----------------------+------+-------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 |  bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+-------+------+------+------+------+------+------+
     * |          0x6B         |          107          |   -  | SLEEP |   -  |   -  |   -  |   -  |   -  |   -  |
     * +-----------------------+-----------------------+------+-------+------+------+------+------+------+------+
     *
     * Wake up the device from sleep mode.
     * Also consider that all the sensors (accelerometer, gyroscope, and temperature) are enabled and consuming around
     * 3.8 (mA) current.
     */
    motionTrackingDevice.setSleepEnabled(false);
}

void waitForCommandToStartCalibrating() {
    /*
     * Empty the buffer.
     */
    while (Serial.available() && Serial.read());

    /*
     * Wait for command to start calibrating.
     */
    while (!Serial.available()) {
        Serial.println(F("Send any character to start calibrating..."));
        delay(1500);
    }

    /*
     * Empty the buffer again.
     */
    while (Serial.available() && Serial.read());
}

void calibrate() {
    while (!isCalibrated()) {
        Serial.println(F("\ncalibrating..."));
        setCalibrationOffsets();
        discardUnreliableValues();
        computeAverageValues();
        printHeader();
        printAverageValues();
        printCalibrationOffsets();
        updateCalibrationOffsetsIfNotCalibrated();
    }
}

void printResult() {
    Serial.println(F("\nMPU-6050 is calibrated."));
    Serial.println(F("\nUse these calibration offsets in your code:"));

    Serial.print(F("motionTrackingDevice.setXAccelOffset("));
    Serial.print(accelerometerXOffset);
    Serial.println(F(");"));

    Serial.print(F("motionTrackingDevice.setYAccelOffset("));
    Serial.print(accelerometerYOffset);
    Serial.println(F(");"));

    Serial.print(F("motionTrackingDevice.setZAccelOffset("));
    Serial.print(accelerometerZOffset);
    Serial.println(F(");"));

    Serial.print(F("motionTrackingDevice.setXGyroOffset("));
    Serial.print(gyroscopeXOffset);
    Serial.println(F(");"));

    Serial.print(F("motionTrackingDevice.setYGyroOffset("));
    Serial.print(gyroscopeYOffset);
    Serial.println(F(");"));

    Serial.print(F("motionTrackingDevice.setZGyroOffset("));
    Serial.print(gyroscopeZOffset);
    Serial.println(F(");"));
}

bool isCalibrated() {
    return isAccelerometerXCalibrated()
           && isAccelerometerYCalibrated()
           && isAccelerometerZCalibrated()
           && isGyroscopeXCalibrated()
           && isGyroscopeYCalibrated()
           && isGyroscopeZCalibrated();
}

void setCalibrationOffsets() {
    motionTrackingDevice.setXAccelOffset(accelerometerXOffset);
    motionTrackingDevice.setYAccelOffset(accelerometerYOffset);
    motionTrackingDevice.setZAccelOffset(accelerometerZOffset);
    motionTrackingDevice.setXGyroOffset(gyroscopeXOffset);
    motionTrackingDevice.setYGyroOffset(gyroscopeYOffset);
    motionTrackingDevice.setZGyroOffset(gyroscopeZOffset);
}

void discardUnreliableValues() {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    int16_t gyroscopeX;
    int16_t gyroscopeY;
    int16_t gyroscopeZ;

    for (int counter = 0; counter < DISCARD_SIZE; counter++) {
        motionTrackingDevice.getMotion6(&accelerometerX, &accelerometerY, &accelerometerZ, &gyroscopeX, &gyroscopeY,
                                        &gyroscopeZ);

        /*
         * So that we don't get the same values over, and over again.
         */
        delay(2);
    }
}

void computeAverageValues() {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    int16_t gyroscopeX;
    int16_t gyroscopeY;
    int16_t gyroscopeZ;

    long accelerometerXBuffer = 0;
    long accelerometerYBuffer = 0;
    long accelerometerZBuffer = 0;
    long gyroscopeXBuffer = 0;
    long gyroscopeYBuffer = 0;
    long gyroscopeZBuffer = 0;

    for (int counter = 0; counter < BUFFER_SIZE; counter++) {
        motionTrackingDevice.getMotion6(&accelerometerX, &accelerometerY, &accelerometerZ, &gyroscopeX, &gyroscopeY,
                                        &gyroscopeZ);

        accelerometerXBuffer += accelerometerX;
        accelerometerYBuffer += accelerometerY;
        accelerometerZBuffer += accelerometerZ;
        gyroscopeXBuffer += gyroscopeX;
        gyroscopeYBuffer += gyroscopeY;
        gyroscopeZBuffer += gyroscopeZ;

        /*
         * So that we don't get the same values over, and over again.
         */
        delay(2);
    }

    accelerometerXAverage = static_cast<int>(accelerometerXBuffer / BUFFER_SIZE);
    accelerometerYAverage = static_cast<int>(accelerometerYBuffer / BUFFER_SIZE);
    accelerometerZAverage = static_cast<int>(accelerometerZBuffer / BUFFER_SIZE);
    gyroscopeXAverage = static_cast<int>(gyroscopeXBuffer / BUFFER_SIZE);
    gyroscopeYAverage = static_cast<int>(gyroscopeYBuffer / BUFFER_SIZE);
    gyroscopeZAverage = static_cast<int>(gyroscopeZBuffer / BUFFER_SIZE);
}

void printHeader() {
    Serial.println(F("\t\t\tax\tay\taz\tgx\tgy\tgz"));
    Serial.println(F("\t\t\t------------------------------------------------"));
}

void printAverageValues() {
    Serial.print(F("average values:\t\t"));
    Serial.print(accelerometerXAverage);
    Serial.print(F("\t"));
    Serial.print(accelerometerYAverage);
    Serial.print(F("\t"));
    Serial.print(accelerometerZAverage);
    Serial.print(F("\t"));
    Serial.print(gyroscopeXAverage);
    Serial.print(F("\t"));
    Serial.print(gyroscopeYAverage);
    Serial.print(F("\t"));
    Serial.println(gyroscopeZAverage);
}

void printCalibrationOffsets() {
    Serial.print(F("calibration offsets:\t"));
    Serial.print(accelerometerXOffset);
    Serial.print(F("\t"));
    Serial.print(accelerometerYOffset);
    Serial.print(F("\t"));
    Serial.print(accelerometerZOffset);
    Serial.print(F("\t"));
    Serial.print(gyroscopeXOffset);
    Serial.print(F("\t"));
    Serial.print(gyroscopeYOffset);
    Serial.print(F("\t"));
    Serial.println(gyroscopeZOffset);
}

void updateCalibrationOffsetsIfNotCalibrated() {
    if (!isAccelerometerXCalibrated()) updateAccelerometerXOffset();
    if (!isAccelerometerYCalibrated()) updateAccelerometerYOffset();
    if (!isAccelerometerZCalibrated()) updateAccelerometerZOffset();
    if (!isGyroscopeXCalibrated()) updateGyroscopeXOffset();
    if (!isGyroscopeYCalibrated()) updateGyroscopeYOffset();
    if (!isGyroscopeZCalibrated()) updateGyroscopeZOffset();
}

bool isAccelerometerXCalibrated() { return abs(getAccelerometerXDifference()) <= ACCELEROMETER_TOLERANCE; }

bool isAccelerometerYCalibrated() { return abs(getAccelerometerYDifference()) <= ACCELEROMETER_TOLERANCE; }

bool isAccelerometerZCalibrated() { return abs(getAccelerometerZDifference()) <= ACCELEROMETER_TOLERANCE; }

bool isGyroscopeXCalibrated() { return abs(getGyroscopeXDifference()) <= GYROSCOPE_TOLERANCE; }

bool isGyroscopeYCalibrated() { return abs(getGyroscopeYDifference()) <= GYROSCOPE_TOLERANCE; }

bool isGyroscopeZCalibrated() { return abs(getGyroscopeZDifference()) <= GYROSCOPE_TOLERANCE; }

void updateAccelerometerXOffset() { accelerometerXOffset += getAccelerometerXDifference() / ACCELEROMETER_STEP; }

void updateAccelerometerYOffset() { accelerometerYOffset += getAccelerometerYDifference() / ACCELEROMETER_STEP; }

void updateAccelerometerZOffset() { accelerometerZOffset += getAccelerometerZDifference() / ACCELEROMETER_STEP; }

void updateGyroscopeXOffset() { gyroscopeXOffset += getGyroscopeXDifference() / GYROSCOPE_STEP; }

void updateGyroscopeYOffset() { gyroscopeYOffset += getGyroscopeYDifference() / GYROSCOPE_STEP; }

void updateGyroscopeZOffset() { gyroscopeZOffset += getGyroscopeZDifference() / GYROSCOPE_STEP; }

int getAccelerometerXDifference() { return ACCELEROMETER_X_GOAL - accelerometerXAverage; }

int getAccelerometerYDifference() { return ACCELEROMETER_Y_GOAL - accelerometerYAverage; }

int getAccelerometerZDifference() { return ACCELEROMETER_Z_GOAL - accelerometerZAverage; }

int getGyroscopeXDifference() { return GYROSCOPE_X_GOAL - gyroscopeXAverage; }

int getGyroscopeYDifference() { return GYROSCOPE_Y_GOAL - gyroscopeYAverage; }

int getGyroscopeZDifference() { return GYROSCOPE_Z_GOAL - gyroscopeZAverage; }