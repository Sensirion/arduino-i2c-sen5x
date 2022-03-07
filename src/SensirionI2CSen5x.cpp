/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * I2C-Generator: 0.3.0
 * Yaml Version: 2.1.3
 * Template Version: 0.7.0-112-g190ecaa
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SensirionI2CSen5x.h"
#include "Arduino.h"
#include "SensirionCore.h"
#include <Wire.h>
#include <math.h>

#define SEN5X_I2C_ADDRESS 0x69
#define UINT_INVALID 0xFFFF
#define INT_INVALID 0x7FFF

SensirionI2CSen5x::SensirionI2CSen5x() {
}

void SensirionI2CSen5x::begin(TwoWire& i2cBus) {
    _i2cBus = &i2cBus;
}

uint16_t SensirionI2CSen5x::startMeasurement() {
    uint16_t error = 0;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x21, buffer, 2);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(50);
    return error;
}

uint16_t SensirionI2CSen5x::startMeasurementWithoutPm() {
    uint16_t error = 0;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x37, buffer, 2);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(50);
    return error;
}

uint16_t SensirionI2CSen5x::stopMeasurement() {
    uint16_t error = 0;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x104, buffer, 2);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(200);
    return error;
}

uint16_t SensirionI2CSen5x::readDataReady(bool& dataReady) {
    uint16_t error = 0;
    uint8_t buffer[3];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x202, buffer, 3);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 3);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 3,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    uint8_t padding;
    error |= rxFrame.getUInt8(padding);  // remove padding
    error |= rxFrame.getBool(dataReady);
    return error;
}

uint16_t SensirionI2CSen5x::readMeasuredValues(
    float& massConcentrationPm1p0, float& massConcentrationPm2p5,
    float& massConcentrationPm4p0, float& massConcentrationPm10p0,
    float& ambientHumidity, float& ambientTemperature, float& vocIndex,
    float& noxIndex) {

    uint16_t error = 0;
    uint16_t massConcentrationPm1p0Int;
    uint16_t massConcentrationPm2p5Int;
    uint16_t massConcentrationPm4p0Int;
    uint16_t massConcentrationPm10p0Int;
    int16_t ambientHumidityInt;
    int16_t ambientTemperatureInt;
    int16_t vocIndexInt;
    int16_t noxIndexInt;

    error = readMeasuredValuesAsIntegers(
        massConcentrationPm1p0Int, massConcentrationPm2p5Int,
        massConcentrationPm4p0Int, massConcentrationPm10p0Int,
        ambientHumidityInt, ambientTemperatureInt, vocIndexInt, noxIndexInt);

    if (error) {
        return error;
    }

    massConcentrationPm1p0 = massConcentrationPm1p0Int == UINT_INVALID
                                 ? NAN
                                 : massConcentrationPm1p0Int / 10.0f;
    massConcentrationPm2p5 = massConcentrationPm2p5Int == UINT_INVALID
                                 ? NAN
                                 : massConcentrationPm2p5Int / 10.0f;
    massConcentrationPm4p0 = massConcentrationPm4p0Int == UINT_INVALID
                                 ? NAN
                                 : massConcentrationPm4p0Int / 10.0f;
    massConcentrationPm10p0 = massConcentrationPm10p0Int == UINT_INVALID
                                  ? NAN
                                  : massConcentrationPm10p0Int / 10.0f;
    ambientHumidity =
        ambientHumidityInt == INT_INVALID ? NAN : ambientHumidityInt / 100.0f;
    ambientTemperature = ambientTemperatureInt == INT_INVALID
                             ? NAN
                             : ambientTemperatureInt / 200.0f;
    vocIndex = vocIndexInt == INT_INVALID ? NAN : vocIndexInt / 10.0f;
    noxIndex = noxIndexInt == INT_INVALID ? NAN : noxIndexInt / 10.0f;

    return NoError;
}

uint16_t SensirionI2CSen5x::readMeasuredValuesAsIntegers(
    uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
    uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
    int16_t& ambientHumidity, int16_t& ambientTemperature, int16_t& vocIndex,
    int16_t& noxIndex) {
    uint16_t error = 0;
    uint8_t buffer[24];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x3C4, buffer, 24);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 24);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 24,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(massConcentrationPm1p0);
    error |= rxFrame.getUInt16(massConcentrationPm2p5);
    error |= rxFrame.getUInt16(massConcentrationPm4p0);
    error |= rxFrame.getUInt16(massConcentrationPm10p0);
    error |= rxFrame.getInt16(ambientHumidity);
    error |= rxFrame.getInt16(ambientTemperature);
    error |= rxFrame.getInt16(vocIndex);
    error |= rxFrame.getInt16(noxIndex);
    return error;
}

uint16_t SensirionI2CSen5x::readMeasuredRawValues(int16_t& rawHumidity,
                                                  int16_t& rawTemperature,
                                                  uint16_t& rawVoc,
                                                  uint16_t& rawNox) {
    uint16_t error = 0;
    uint8_t buffer[12];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x3D2, buffer, 12);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 12);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 12,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(rawHumidity);
    error |= rxFrame.getInt16(rawTemperature);
    error |= rxFrame.getUInt16(rawVoc);
    error |= rxFrame.getUInt16(rawNox);
    return error;
}

uint16_t SensirionI2CSen5x::readMeasuredValuesSen50(
    float& massConcentrationPm1p0, float& massConcentrationPm2p5,
    float& massConcentrationPm4p0, float& massConcentrationPm10p0) {

    uint16_t error = 0;
    float ambientHumidityDummy;
    float ambientTemperatureDummy;
    float vocIndexDummy;
    float noxIndexDummy;
    error = readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                               massConcentrationPm4p0, massConcentrationPm10p0,
                               ambientHumidityDummy, ambientTemperatureDummy,
                               vocIndexDummy, noxIndexDummy);
    return error;
}

uint16_t SensirionI2CSen5x::readMeasuredPmValues(
    float& massConcentrationPm1p0, float& massConcentrationPm2p5,
    float& massConcentrationPm4p0, float& massConcentrationPm10p0,
    float& numberConcentrationPm0p5, float& numberConcentrationPm1p0,
    float& numberConcentrationPm2p5, float& numberConcentrationPm4p0,
    float& numberConcentrationPm10p0, float& typicalParticleSize) {

    uint16_t error = 0;
    uint16_t massConcentrationPm1p0Int;
    uint16_t massConcentrationPm2p5Int;
    uint16_t massConcentrationPm4p0Int;
    uint16_t massConcentrationPm10p0Int;
    uint16_t numberConcentrationPm0p5Int;
    uint16_t numberConcentrationPm1p0Int;
    uint16_t numberConcentrationPm2p5Int;
    uint16_t numberConcentrationPm4p0Int;
    uint16_t numberConcentrationPm10p0Int;
    uint16_t typicalParticleSizeInt;

    error = readMeasuredPmValuesAsIntegers(
        massConcentrationPm1p0Int, massConcentrationPm2p5Int,
        massConcentrationPm4p0Int, massConcentrationPm10p0Int,
        numberConcentrationPm0p5Int, numberConcentrationPm1p0Int,
        numberConcentrationPm2p5Int, numberConcentrationPm4p0Int,
        numberConcentrationPm10p0Int, typicalParticleSizeInt);

    if (error) {
        return error;
    }

    massConcentrationPm1p0 = massConcentrationPm1p0Int == UINT_INVALID
                                 ? NAN
                                 : massConcentrationPm1p0Int / 10.0f;
    massConcentrationPm2p5 = massConcentrationPm2p5Int == UINT_INVALID
                                 ? NAN
                                 : massConcentrationPm2p5Int / 10.0f;
    massConcentrationPm4p0 = massConcentrationPm4p0Int == UINT_INVALID
                                 ? NAN
                                 : massConcentrationPm4p0Int / 10.0f;
    massConcentrationPm10p0 = massConcentrationPm10p0Int == UINT_INVALID
                                  ? NAN
                                  : massConcentrationPm10p0Int / 10.0f;
    numberConcentrationPm0p5 = numberConcentrationPm0p5Int == UINT_INVALID
                                   ? NAN
                                   : numberConcentrationPm0p5Int / 10.0f;
    numberConcentrationPm1p0 = numberConcentrationPm1p0Int == UINT_INVALID
                                   ? NAN
                                   : numberConcentrationPm1p0Int / 10.0f;
    numberConcentrationPm2p5 = numberConcentrationPm2p5Int == UINT_INVALID
                                   ? NAN
                                   : numberConcentrationPm2p5Int / 10.0f;
    numberConcentrationPm4p0 = numberConcentrationPm4p0Int == UINT_INVALID
                                   ? NAN
                                   : numberConcentrationPm4p0Int / 10.0f;
    numberConcentrationPm10p0 = numberConcentrationPm10p0Int == UINT_INVALID
                                    ? NAN
                                    : numberConcentrationPm10p0Int / 10.0f;
    typicalParticleSize = typicalParticleSizeInt == UINT_INVALID
                              ? NAN
                              : typicalParticleSizeInt / 10.0f;

    return NoError;
}

uint16_t SensirionI2CSen5x::readMeasuredPmValuesAsIntegers(
    uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
    uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
    uint16_t& numberConcentrationPm0p5, uint16_t& numberConcentrationPm1p0,
    uint16_t& numberConcentrationPm2p5, uint16_t& numberConcentrationPm4p0,
    uint16_t& numberConcentrationPm10p0, uint16_t& typicalParticleSize) {
    uint16_t error = 0;
    uint8_t buffer[30];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x413, buffer, 30);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 30);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 30,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(massConcentrationPm1p0);
    error |= rxFrame.getUInt16(massConcentrationPm2p5);
    error |= rxFrame.getUInt16(massConcentrationPm4p0);
    error |= rxFrame.getUInt16(massConcentrationPm10p0);
    error |= rxFrame.getUInt16(numberConcentrationPm0p5);
    error |= rxFrame.getUInt16(numberConcentrationPm1p0);
    error |= rxFrame.getUInt16(numberConcentrationPm2p5);
    error |= rxFrame.getUInt16(numberConcentrationPm4p0);
    error |= rxFrame.getUInt16(numberConcentrationPm10p0);
    error |= rxFrame.getUInt16(typicalParticleSize);
    return error;
}

uint16_t SensirionI2CSen5x::startFanCleaning() {
    uint16_t error = 0;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x5607, buffer, 2);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::setTemperatureOffsetSimple(float tempOffset) {
    int16_t defaultSlope = 0;
    uint16_t defaultTimeConstant = 0;
    int16_t tempOffsetTicks = static_cast<int16_t>(tempOffset * 200);
    return setTemperatureOffsetParameters(tempOffsetTicks, defaultSlope,
                                          defaultTimeConstant);
}

uint16_t SensirionI2CSen5x::getTemperatureOffsetSimple(float& tempOffset) {
    int16_t tempOffsetTicks;
    int16_t slope;
    uint16_t timeConstant;
    uint16_t error = 0;

    error =
        getTemperatureOffsetParameters(tempOffsetTicks, slope, timeConstant);
    if (error) {
        return error;
    }

    tempOffset = static_cast<float>(tempOffsetTicks) / 200.0f;

    return NoError;
}

uint16_t SensirionI2CSen5x::setTemperatureOffsetParameters(
    int16_t tempOffset, int16_t slope, uint16_t timeConstant) {
    uint16_t error = 0;
    uint8_t buffer[11];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60B2, buffer, 11);

    error |= txFrame.addInt16(tempOffset);
    error |= txFrame.addInt16(slope);
    error |= txFrame.addUInt16(timeConstant);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getTemperatureOffsetParameters(
    int16_t& tempOffset, int16_t& slope, uint16_t& timeConstant) {
    uint16_t error = 0;
    uint8_t buffer[9];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60B2, buffer, 9);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 9);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 9,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(tempOffset);
    error |= rxFrame.getInt16(slope);
    error |= rxFrame.getUInt16(timeConstant);
    return error;
}

uint16_t SensirionI2CSen5x::setWarmStartParameter(uint16_t warmStart) {
    uint16_t error = 0;
    uint8_t buffer[5];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60C6, buffer, 5);

    error |= txFrame.addUInt16(warmStart);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getWarmStartParameter(uint16_t& warmStart) {
    uint16_t error = 0;
    uint8_t buffer[3];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60C6, buffer, 3);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 3);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 3,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(warmStart);
    return error;
}

uint16_t SensirionI2CSen5x::setVocAlgorithmTuningParameters(
    int16_t indexOffset, int16_t learningTimeOffsetHours,
    int16_t learningTimeGainHours, int16_t gatingMaxDurationMinutes,
    int16_t stdInitial, int16_t gainFactor) {
    uint16_t error = 0;
    uint8_t buffer[20];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60D0, buffer, 20);

    error |= txFrame.addInt16(indexOffset);
    error |= txFrame.addInt16(learningTimeOffsetHours);
    error |= txFrame.addInt16(learningTimeGainHours);
    error |= txFrame.addInt16(gatingMaxDurationMinutes);
    error |= txFrame.addInt16(stdInitial);
    error |= txFrame.addInt16(gainFactor);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getVocAlgorithmTuningParameters(
    int16_t& indexOffset, int16_t& learningTimeOffsetHours,
    int16_t& learningTimeGainHours, int16_t& gatingMaxDurationMinutes,
    int16_t& stdInitial, int16_t& gainFactor) {
    uint16_t error = 0;
    uint8_t buffer[18];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60D0, buffer, 18);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 18);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 18,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(indexOffset);
    error |= rxFrame.getInt16(learningTimeOffsetHours);
    error |= rxFrame.getInt16(learningTimeGainHours);
    error |= rxFrame.getInt16(gatingMaxDurationMinutes);
    error |= rxFrame.getInt16(stdInitial);
    error |= rxFrame.getInt16(gainFactor);
    return error;
}

uint16_t SensirionI2CSen5x::setNoxAlgorithmTuningParameters(
    int16_t indexOffset, int16_t learningTimeOffsetHours,
    int16_t learningTimeGainHours, int16_t gatingMaxDurationMinutes,
    int16_t stdInitial, int16_t gainFactor) {
    uint16_t error = 0;
    uint8_t buffer[20];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60E1, buffer, 20);

    error |= txFrame.addInt16(indexOffset);
    error |= txFrame.addInt16(learningTimeOffsetHours);
    error |= txFrame.addInt16(learningTimeGainHours);
    error |= txFrame.addInt16(gatingMaxDurationMinutes);
    error |= txFrame.addInt16(stdInitial);
    error |= txFrame.addInt16(gainFactor);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getNoxAlgorithmTuningParameters(
    int16_t& indexOffset, int16_t& learningTimeOffsetHours,
    int16_t& learningTimeGainHours, int16_t& gatingMaxDurationMinutes,
    int16_t& stdInitial, int16_t& gainFactor) {
    uint16_t error = 0;
    uint8_t buffer[18];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60E1, buffer, 18);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 18);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 18,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(indexOffset);
    error |= rxFrame.getInt16(learningTimeOffsetHours);
    error |= rxFrame.getInt16(learningTimeGainHours);
    error |= rxFrame.getInt16(gatingMaxDurationMinutes);
    error |= rxFrame.getInt16(stdInitial);
    error |= rxFrame.getInt16(gainFactor);
    return error;
}

uint16_t SensirionI2CSen5x::setRhtAccelerationMode(uint16_t mode) {
    uint16_t error = 0;
    uint8_t buffer[5];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60F7, buffer, 5);

    error |= txFrame.addUInt16(mode);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getRhtAccelerationMode(uint16_t& mode) {
    uint16_t error = 0;
    uint8_t buffer[3];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x60F7, buffer, 3);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 3);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 3,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(mode);
    return error;
}

uint16_t SensirionI2CSen5x::setVocAlgorithmState(const uint8_t state[],
                                                 uint8_t stateSize) {
    uint16_t error = 0;
    uint8_t buffer[14];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x6181, buffer, 14);

    error |= txFrame.addBytes(state, stateSize);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getVocAlgorithmState(uint8_t state[],
                                                 uint8_t stateSize) {
    uint16_t error = 0;
    uint8_t buffer[12];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x6181, buffer, 12);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 12);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 12,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(state, stateSize);
    return error;
}

uint16_t SensirionI2CSen5x::setFanAutoCleaningInterval(uint32_t interval) {
    uint16_t error = 0;
    uint8_t buffer[8];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x8004, buffer, 8);

    error |= txFrame.addUInt32(interval);

    if (error) {
        return error;
    }

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(20);
    return error;
}

uint16_t SensirionI2CSen5x::getFanAutoCleaningInterval(uint32_t& interval) {
    uint16_t error = 0;
    uint8_t buffer[6];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x8004, buffer, 6);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 6);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 6,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt32(interval);
    return error;
}

uint16_t SensirionI2CSen5x::getProductName(unsigned char productName[],
                                           uint8_t productNameSize) {
    uint16_t error = 0;
    uint8_t buffer[48];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0xD014, buffer, 48);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(50);

    SensirionI2CRxFrame rxFrame(buffer, 48);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 48,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(productName, productNameSize);
    return error;
}

uint16_t SensirionI2CSen5x::getSerialNumber(unsigned char serialNumber[],
                                            uint8_t serialNumberSize) {
    uint16_t error = 0;
    uint8_t buffer[48];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0xD033, buffer, 48);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(50);

    SensirionI2CRxFrame rxFrame(buffer, 48);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 48,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(serialNumber, serialNumberSize);
    return error;
}

uint16_t
SensirionI2CSen5x::getVersion(uint8_t& firmwareMajor, uint8_t& firmwareMinor,
                              bool& firmwareDebug, uint8_t& hardwareMajor,
                              uint8_t& hardwareMinor, uint8_t& protocolMajor,
                              uint8_t& protocolMinor) {
    uint16_t error = 0;
    uint8_t buffer[12];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0xD100, buffer, 12);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 12);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 12,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt8(firmwareMajor);
    error |= rxFrame.getUInt8(firmwareMinor);
    error |= rxFrame.getBool(firmwareDebug);
    error |= rxFrame.getUInt8(hardwareMajor);
    error |= rxFrame.getUInt8(hardwareMinor);
    error |= rxFrame.getUInt8(protocolMajor);
    error |= rxFrame.getUInt8(protocolMinor);
    uint8_t padding;
    error |= rxFrame.getUInt8(padding);  // remove padding
    return error;
}

uint16_t SensirionI2CSen5x::readDeviceStatus(uint32_t& deviceStatus) {
    uint16_t error = 0;
    uint8_t buffer[6];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0xD206, buffer, 6);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 6);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 6,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt32(deviceStatus);
    return error;
}

uint16_t SensirionI2CSen5x::readAndClearDeviceStatus(uint32_t& deviceStatus) {
    uint16_t error = 0;
    uint8_t buffer[6];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0xD210, buffer, 6);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    if (error) {
        return error;
    }

    delay(20);

    SensirionI2CRxFrame rxFrame(buffer, 6);
    error = SensirionI2CCommunication::receiveFrame(SEN5X_I2C_ADDRESS, 6,
                                                    rxFrame, *_i2cBus);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt32(deviceStatus);
    return error;
}

uint16_t SensirionI2CSen5x::deviceReset() {
    uint16_t error = 0;
    uint8_t buffer[2];
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0xD304, buffer, 2);

    error = SensirionI2CCommunication::sendFrame(SEN5X_I2C_ADDRESS, txFrame,
                                                 *_i2cBus);
    delay(200);
    return error;
}
