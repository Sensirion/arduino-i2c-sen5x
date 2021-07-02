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

#include <Arduino.h>
#include <SensirionI2CSen55.h>
#include <Wire.h>

SensirionI2CSen55 sen55;

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen55.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen55.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen55.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}

void setup() {

    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();

    sen55.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = sen55.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    // Print SEN55 module information
    printSerialNumber();
    printModuleVersions();

    // Start Measurement
    error = sen55.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
}

void loop() {
    uint16_t error;
    char errorMessage[256];

    delay(1000);

    // Read Measurement
    uint16_t massConcentrationPm1p0;
    uint16_t massConcentrationPm2p5;
    uint16_t massConcentrationPm4p0;
    uint16_t massConcentrationPm10p0;
    int16_t ambientHumidity;
    int16_t ambientTemperature;
    int16_t vocIndex;
    int16_t noxIndex;

    error = sen55.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("MassConcentrationPm1p0:");
        Serial.print(massConcentrationPm1p0 / 10.0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm2p5:");
        Serial.print(massConcentrationPm2p5 / 10.0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm4p0:");
        Serial.print(massConcentrationPm4p0 / 10.0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm10p0:");
        Serial.print(massConcentrationPm10p0 / 10.0);
        Serial.print("\t");
        Serial.print("AmbientHumidity:");
        Serial.print(ambientHumidity / 100.0);
        Serial.print("\t");
        Serial.print("AmbientTemperature:");
        Serial.print(ambientTemperature / 200.0);
        Serial.print("\t");
        Serial.print("VocIndex:");
        Serial.print(vocIndex / 10.0);
        Serial.print("\t");
        Serial.print("NoxIndex:");
        Serial.println(noxIndex / 10.0);
    }
}
