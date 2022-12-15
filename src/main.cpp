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

bool blink = false;
unsigned long timer;

// ---------------------------------------------------------------------------------------------------------------------
//                                                 DEFINICJE FUNKCJI
// ---------------------------------------------------------------------------------------------------------------------

void init_mpu();
void get_gyr_pry(int time, int t_s);
void filtered_angles(int t);
void show(int t, double a_x, double a_y, double a_z);

// ---------------------------------------------------------------------------------------------------------------------
//                                             FUNKCJA STARTOWA - SETUP
// ---------------------------------------------------------------------------------------------------------------------

__attribute__((unused)) void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000L);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
}

// ---------------------------------------------------------------------------------------------------------------------
//                                        GŁÓWNE CIAŁO PROGRAMU - FUNKCJA LOOP
// ---------------------------------------------------------------------------------------------------------------------

__attribute__((unused)) void loop() {
    String value = Serial.readString();

    if (value == "0") {
        filtered_angles(10);
    }
    else if (value == "1") {
        get_gyr_pry(10000, 10);
    }
    else if (value == "9") {
        init_mpu();
    }
}

// ---------------------------------------------------------------------------------------------------------------------
//                                             CIAŁA ZDEFINIOWANYCH FUNKCJI
// ---------------------------------------------------------------------------------------------------------------------

void init_mpu() {
    Serial.println("Sprawdzanie polaczenia z shieldem...");
    Serial.println(mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)
        ? "Polaczenie udane." : "Polaczenie nieudane.");

    Serial.println("Kalibracja zyroskopu...");
    mpu.calibrateGyro();
    Serial.println("Kalibracja ukonczona.");

    mpu.setThreshold(0);
}

void filtered_angles(int t) {
    KalmanFilter kalmanX(0.001, 0.003, 0.03);
    KalmanFilter kalmanY(0.001, 0.003, 0.03);

    Serial.println("Czas \t Pitch \t Roll \t (K)Pitch \t (K)Roll");

    for (long i = 0; i <= 10000; i += t) {
        Vector acc = mpu.readNormalizeAccel();
        Vector gyr = mpu.readNormalizeGyro();

        timer = millis();

        double acc_pitch = -(atan2(acc.XAxis, sqrt(square(acc.YAxis) + square(acc.ZAxis))) * 180.0) / M_PI;
        double acc_roll = (atan2(acc.YAxis, sqrt(square(acc.XAxis) + square(acc.ZAxis))) * 180.0) / M_PI;

        double kal_pitch = kalmanY.update(acc_pitch, gyr.YAxis);
        double kal_roll = kalmanX.update(acc_roll, gyr.XAxis);

        Serial.print(i); Serial.print("\t");
        Serial.print(acc_pitch); Serial.print("\t"); Serial.print(acc_roll); Serial.print("\t");
        Serial.print(kal_pitch); Serial.print("\t"); Serial.println(kal_roll);

        delay(millis() - timer);
    }
}

void get_gyr_pry(int time, int t_s) {
    Serial.println("t [ms] \t pitch \t roll \t yaw");

    double pitch = 0;
    double roll = 0;
    double yaw = 0;

    for (int i = 0; i <= time; i+= t_s) {
        Vector gyr = mpu.readNormalizeGyro();

        timer = millis();

        pitch += gyr.YAxis * t_s / 1000;
        roll += gyr.XAxis * t_s / 1000;
        yaw += gyr.ZAxis * t_s / 1000;

        show(i, pitch, roll, yaw);

        blink = !blink;
        digitalWrite(LED_BUILTIN, blink);

        delay(millis() - timer);
    }
}

void show(int t, double a_x, double a_y, double a_z) {
    Serial.print(t); Serial.print("\t");
    Serial.print(a_x); Serial.print("\t");
    Serial.print(a_y); Serial.print("\t");
    Serial.println(a_z);
}