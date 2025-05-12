/* 
    Project Name: Face Recognizing and Following Robot
    Engineer: Avinash Damse
    Institution: California State Univercity, Northridge.
    Date: 04/09/2025
*/

#include <Arduino.h>
#include "HUSKYLENS.h"
#include "HardwareSerial.h"

HUSKYLENS huskylens;
HardwareSerial mySerial(2); // UART2 on ESP32 (TX:17, RX:16)

// Motor control pins (L298N Motor Driver)
const int MOTOR1_ENA = 15;   // PWM for Motor 1- Right
const int MOTOR1_IN1 = 2;   // Direction 1
const int MOTOR1_IN2 = 4;  // Direction 2
const int MOTOR2_ENB = 27;  // PWM for Motor 2 - Left
const int MOTOR2_IN3 = 14;  // Direction 3
const int MOTOR2_IN4 = 12;  // Direction 4

// Ultrasonic sensor Pins
const int TRIG_PIN = 18;
const int ECHO_PIN = 19;

// PID constants 
float Kp_fb = 0.9, Ki_fb = 0.1, Kd_fb = 0.05; // Forward/backward
float Kp_lr = 0.25, Ki_lr = 0.02, Kd_lr = 0.05; // Left/right

// PID variables
float error_fb = 0, prevError_fb = 0, integral_fb = 0, derivative_fb = 0;
float error_lr = 0, prevError_lr = 0, integral_lr = 0, derivative_lr = 0;

float setpoint_fb = 65, setpoint_lr = 160;

// Thresholds 
const int minPWM = 55, maxPWM = 255;
const int faceHeightThreshold = 22;
const int faceXThreshold = 20;
const int obstacleThreshold = 30; // 30cm

// Timing
unsigned long lastFaceTime = 0;
const unsigned long noFaceTimeout = 1000;
unsigned long lastObstacleCheck = 0;
const unsigned long obstacleInterval = 200;

int targetFaceID = 1;

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, 16, 17);

    // Motor pins
    pinMode(MOTOR1_ENA, OUTPUT);
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_ENB, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    stopMotors();
    
    // Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // HuskyLens
    while(!huskylens.begin(mySerial)) {
        Serial.println(F("HuskyLens error!"));
        delay(1000);
    }
    huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION);
}



void loop() {
    // Obstacle checking
    if(millis() - lastObstacleCheck > obstacleInterval) {
        lastObstacleCheck = millis();
        float dist = getDistance();
        if(dist > 0 && dist < obstacleThreshold) {
            Serial.print("Obstacle: "); Serial.print(dist); Serial.println("cm");
            stopMotors();
            while(getDistance() < obstacleThreshold) {
                delay(100); // Keep checking obstacle
            }
            return;
        }
    }

    // Face following
    if(!huskylens.request()) {
        Serial.println(F("HUSKYLENS error"));
        return;
    } 
    else if(!huskylens.isLearned()) {
        Serial.println(F("No faces learned"));
    }
    else {
        bool faceFound = false;
        while(huskylens.available()) {
            HUSKYLENSResult result = huskylens.read();
            if(result.ID == targetFaceID) {
                faceFound = true;
                lastFaceTime = millis();
                
                // PID Calculations
                int faceX = result.xCenter;
                int faceH = result.height;
                
                error_fb = setpoint_fb - faceH;
                integral_fb += error_fb;
                derivative_fb = error_fb - prevError_fb;
                prevError_fb = error_fb;

                float output_fb = Kp_fb*error_fb + Ki_fb*integral_fb + Kd_fb*derivative_fb;
                output_fb = constrain(output_fb, -maxPWM, maxPWM);
                
                
                error_lr = setpoint_lr - faceX;
                integral_lr += error_lr;
                derivative_lr = error_lr - prevError_lr;
                prevError_lr = error_lr;

                float output_lr = Kp_lr*error_lr + Ki_lr*integral_lr + Kd_lr*derivative_lr;
                output_lr = constrain(output_lr, -maxPWM, maxPWM);
                

                // Movement logic
                if(abs(error_fb) > faceHeightThreshold) {
                    int speed = map(abs(output_fb), 0, maxPWM, minPWM, maxPWM);
                    if(output_fb > 0) {
                        moveForward(speed);
                    } else {
                        moveBackward(speed);
                    }
                    delay(100);
                } 
                else if(abs(error_lr) > faceXThreshold) {
                    int speed = map(abs(output_lr), 0, maxPWM, minPWM, maxPWM);
                    if(output_lr > 0) {
                        turnLeft(speed);
                    } else {
                        turnRight(speed);
                    }
                    delay(100);
                }
                else {
                    stopMotors();
                    // Reset PID variables when stopped
                    error_fb = prevError_fb = integral_fb = derivative_fb = 0;
                    error_lr = prevError_lr = integral_lr = derivative_lr = 0;
                }
                
                break;
            }
        }
        
        if(!faceFound && millis()-lastFaceTime > noFaceTimeout) {
            Serial.println(F("No face - stopping"));
            stopMotors();
            // Reset PID variables when stopped
            error_fb = prevError_fb = integral_fb = derivative_fb = 0;
            error_lr = prevError_lr = integral_lr = derivative_lr = 0;
        }
    }
}

float getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return pulseIn(ECHO_PIN, HIGH) * 0.034/2;
}

void moveForward(int speed) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    analogWrite(MOTOR1_ENA, speed);
    digitalWrite(MOTOR2_IN3, LOW);
    digitalWrite(MOTOR2_IN4, HIGH);
    analogWrite(MOTOR2_ENB, speed);
    Serial.print("Moving Forward: "); Serial.println(speed);
}

void moveBackward(int speed) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_ENA, speed);
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);
    analogWrite(MOTOR2_ENB, speed);
    Serial.print("Moving Backward: "); Serial.println(speed);
}

void turnLeft(int speed) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    analogWrite(MOTOR1_ENA, speed);
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);
    analogWrite(MOTOR2_ENB, speed);
    Serial.print("Turning Left: "); Serial.println(speed);
}

void turnRight(int speed) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_ENA, speed);
    digitalWrite(MOTOR2_IN3, LOW);
    digitalWrite(MOTOR2_IN4, HIGH);
    analogWrite(MOTOR2_ENB, speed);
    Serial.print("Turning Right: "); Serial.println(speed);
}

void stopMotors() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_ENA, 0);
    digitalWrite(MOTOR2_IN3, LOW);
    digitalWrite(MOTOR2_IN4, LOW);
    analogWrite(MOTOR2_ENB, 0);
    Serial.println("Robot Stopped!");
}