////////////////////////////////////////////////////////////////////////////////////////////////
//                            PID Control System Balance Beam (PID BOT)
//                            By Beatrix Tierney
//                            GitHub Repository Link: https://github.com/btier002/PID-Control-System-Balance-Beam-BT/upload/main
////////////////////////////////////////////////////////////////////////////////////////////////
#include <Servo.h>

Servo servo;

// Servo limitations for the system
const int MIN_ANGLE = 40;
const int MAX_ANGLE = 100;
const int SERVO_CENTER = 59; // The "Level" point for your specific setup

// Rangefinder Pins
const int trigPin = 10;  
const int echoPin = 11; 

// PID Math Variables
const double CM_TO_INCH = 1.0 / 2.54;
float set_position = 6.0; // Target distance in inches
float actual_position = 0;
float error = 0, last_error = 0, integral = 0, derivative = 0;
float last_angle = 0;

// PID Tuning Constants
// Kp must be quite high to get a great enough angle to correct the uphill error
// Kd must be high to keep up with the speed of the system
// A small Ki is necessary to adjust where the ball will end up being leveled
const double kp = 8.4;
const double kd = 4.3;
const double ki = 0.1;

unsigned long lastTime = 0;

// Ultrasonic Range Finder distance measurement
// Rejects values that are impossible on the beam system
// Added a Low Pass filter to smooth reading values and prevent impossible level shifts from invalid sensor data
float getFilteredDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return actual_position; // Use last good reading if sensor misses
  
  // Converting to inches 
  float distance = (duration / 2.0) * 0.0344 * CM_TO_INCH;

  // Reject impossible values (Adjust 20 to your track length)
  if (distance <= 0 || distance > 12) return actual_position;

  // Simple Low-Pass Filter: 70% old value, 30% new value
  return (actual_position * 0.7) + (distance * 0.3);
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);  
  
  // Attach servo pin and adjust to center (level)
  servo.attach(9);
  servo.write(SERVO_CENTER); 
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  double dt = (double)(now - lastTime) / 1000.0;

  if (dt >= 0.03) { // Went from 0.02 to 0.03 and seemed to be more stable in its data, the Arduino Mega(the tank) was also able to keep up better
    actual_position = getFilteredDistance();

    // PID controller logic
    // Calculate error 
    error = actual_position - set_position;

    // During each time interval, calculate integral and constrain it to avoid spikes
    if (dt > 0) {
      integral += error * dt;
      integral = constrain(integral, -5, 5);
      
      // Derivative with a bit of smoothing to prevent spikes
      float current_derivative = (error - last_error) / dt;
      derivative = (derivative * 0.5) + (current_derivative * 0.5);
    }

    // Calculate a preprocessed final output(angle for servo)
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    
    // Finally, using reversed logic: Center - output, constrain the final values to be within the servo and beam parameters for the final angle
    int finalAngle = constrain(SERVO_CENTER - (int)output, MIN_ANGLE, MAX_ANGLE);
 
    servo.write(finalAngle);

    // Last values for preprocessing
    last_error = error;
    lastTime = now;

    // Serial Monitor Debugging
    Serial.print("Dist:"); Serial.print(actual_position);
    Serial.print(" | Err:"); Serial.print(error);
    Serial.print(" | Angle:"); Serial.println(finalAngle);
  }
}