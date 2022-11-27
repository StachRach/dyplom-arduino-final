// WYKONAWCA:  Stanisław Rachwał
// KIERUNEK:   Inżynieria Biomedyczna, spec. Fizyka Medyczna
// STOPIEŃ:    1
// SEMESTR:    7

// ---------------------------------------------------------------------------------------------------------------------
//                                             IMPLEMENTACJE BIBLIOTEK
// ---------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ---------------------------------------------------------------------------------------------------------------------
//                                           DEFINICJE ZMIENNYCH GLOBALNYCH
// ---------------------------------------------------------------------------------------------------------------------

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

bool blink = false;

// ---------------------------------------------------------------------------------------------------------------------
//                                                 DEFINICJE FUNKCJI
// ---------------------------------------------------------------------------------------------------------------------

void init_mpu();
void get_gyr_pry(int time, int t_s);
void filtered_angles();
void show(int t, double a_x, double a_y, double a_z);

// ---------------------------------------------------------------------------------------------------------------------
//                                             FUNKCJA STARTOWA - SETUP
// ---------------------------------------------------------------------------------------------------------------------

__attribute__((unused)) void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(38400);
}

// ---------------------------------------------------------------------------------------------------------------------
//                                        GŁÓWNE CIAŁO PROGRAMU - FUNKCJA LOOP
// ---------------------------------------------------------------------------------------------------------------------

__attribute__((unused)) void loop() {
    String value = Serial.readString();

    if (value == "0") {
        filtered_angles();
    }
    else if (value == "1") {
        get_gyr_pry(1000, 10);
    }
    else if (value == "11") {
        get_gyr_pry(1000, 100);
    }
    else if (value == "2") {
        get_gyr_pry(10000, 10);
    }
    else if (value == "22") {
        get_gyr_pry(10000, 100);
    }
    else if (value == "9") {
        init_mpu();
    }
}

// ---------------------------------------------------------------------------------------------------------------------
//                                             CIAŁA ZDEFINIOWANYCH FUNKCJI
// ---------------------------------------------------------------------------------------------------------------------

void init_mpu() {
    Serial.println("Checking connection with a shield...");
    Serial.println(mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)
        ? "Connection successful." : "Connection failed.");

    Serial.println("Offset configuration...");
    mpu.calibrateGyro();
    Serial.println("Configuration completed.");

    mpu.setThreshold(3);
}

void filtered_angles() {
    Serial.println("Time \t Pitch \t Roll \t (K)Pitch \t (K)Roll");

    for (int i = 0; i <= 1000; i += 10) {
        Vector acc = mpu.readNormalizeAccel();
        Vector gyr = mpu.readNormalizeGyro();

        double acc_pitch = -(atan2(acc.XAxis, sqrt(square(acc.YAxis) + square(acc.ZAxis))) * 180.0) / M_PI;
        double acc_roll = (atan2(acc.YAxis, sqrt(square(acc.XAxis) + square(acc.ZAxis))) * 180.0) / M_PI;

        double kal_pitch = kalmanY.update(acc_pitch, gyr.YAxis);
        double kal_roll = kalmanX.update(acc_roll, gyr.XAxis);

        Serial.print(i); Serial.print("\t");
        Serial.print(acc_pitch); Serial.print("\t"); Serial.print(acc_roll); Serial.print("\t");
        Serial.print(kal_pitch); Serial.print("\t"); Serial.println(kal_roll);
    }
}

void get_gyr_pry(int time, int t_s) {
    Serial.println("t [ms] \t pitch \t roll \t yaw");

    double pitch = 0;
    double roll = 0;
    double yaw = 0;

    for (int i = 0; i <= time; i+= t_s) {
        Vector gyr = mpu.readNormalizeGyro();

        pitch += gyr.YAxis * t_s / 1000;
        roll += gyr.XAxis * t_s / 1000;
        yaw += gyr.ZAxis * t_s / 1000;

        show(i, pitch, roll, yaw);

        blink = !blink;
        digitalWrite(LED_BUILTIN, blink);

        delay(t_s);
    }
}

void show(int t, double a_x, double a_y, double a_z) {
    Serial.print(t); Serial.print("\t");
    Serial.print(a_x); Serial.print("\t");
    Serial.print(a_y); Serial.print("\t");
    Serial.println(a_z);
}