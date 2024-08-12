const int analogPinInput = A0; // Pin connected to the input voltage divider
const int analogPinOutput = A1; // Pin connected to the output voltage divider
const int pwmPin = 11;          // PWM output pin (OC2A, controlled by Timer2)

const float R1 = 10000.0;       // 10kΩ for input voltage divider
const float R2 = 2000.0;        // 2kΩ for input voltage divider
const float R3 = 1000.0;       // 1kΩ for output voltage divider
const float R4 = 10000.0;        // 10kΩ for output voltage divider

// Lookup table for car battery voltage and corresponding Li-Ion voltage
//const float carBatteryVoltages[] = {11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.4};
  const float carBatteryVoltages[] = {12.0, 12.1, 12.4, 12.5, 12.8, 12.9, 14.0, 14.4};
       const float liIonVoltages[] = {3.0,  3.05, 3.2,  3.8,  4.0,  4.1,  4.2,  4.2};
const int numPoints = sizeof(carBatteryVoltages) / sizeof(carBatteryVoltages[0]);

// PID control variables
float kp = 1.0;  // Proportional gain
float ki = 0.1;  // Integral gain
float kd = 0.05; // Derivative gain

float previousError = 0.0;
float integral = 0.0;

void setup() {
  Serial.begin(9600); // Start serial communication
  
  // Set up Timer2 for fast PWM mode with a prescaler of 1
  pinMode(pwmPin, OUTPUT);
  
  TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM mode, non-inverting
  TCCR2B = (1 << WGM22) | (1 << CS20);  // No prescaler
  
  OCR2A = 128; // Start with 50% duty cycle
}

void loop() {
  // Measure input voltage (car battery voltage)
  float carBatteryVoltage = readVoltage(analogPinInput, R1, R2);
  
  // Interpolate to find the corresponding target Li-Ion voltage
  float targetLiIonVoltage = interpolateLiIonVoltage(carBatteryVoltage);
  
  // Measure output voltage (Li-Ion battery voltage)
  float actualLiIonVoltage = readVoltage(analogPinOutput, R3, R4);

  // Calculate error
  float error = targetLiIonVoltage - actualLiIonVoltage;

  // Proportional term
  float proportional = kp * error;

  // Integral term
  integral += error;
  float integralTerm = ki * integral;

  // Derivative term
  float derivative = kd * (error - previousError);
  previousError = error;

  // Calculate PID output
  float pidOutput = proportional + integralTerm + derivative;

  // Convert PID output to PWM signal (0-255)
  int pwmValue = constrain(map(pidOutput, 0, 5, 0, 255), 0, 255);

  // Output PWM signal
  OCR2A = pwmValue; // Set PWM duty cycle

  // Debug output
  Serial.print("Car Battery Voltage: ");
  Serial.print(carBatteryVoltage);
  Serial.print(" V, Target Li-Ion Voltage: ");
  Serial.print(targetLiIonVoltage);
  Serial.print(" V, Actual Li-Ion Voltage: ");
  Serial.print(actualLiIonVoltage);
  Serial.print(" V, PWM: ");
  Serial.print(pwmValue);
  Serial.println();

 // delay(100); // Wait for 100 ms before the next cycle
}

// Function to read voltage from an analog pin with a voltage divider
float readVoltage(int pin, float r1, float r2) {
  int rawValue = analogRead(pin);
  float voltage = (rawValue / 1023.0) * 5.0; // Convert to voltage (0-5V)
  return voltage * (r1 + r2) / r2; // Convert to actual voltage
}

// Function to interpolate Li-Ion voltage based on car battery voltage
float interpolateLiIonVoltage(float carVoltage) {
  if (carVoltage <= carBatteryVoltages[0]) {
    return liIonVoltages[0];
  } else if (carVoltage >= carBatteryVoltages[numPoints - 1]) {
    return liIonVoltages[numPoints - 1];
  }

  for (int i = 0; i < numPoints - 1; i++) {
    if (carVoltage >= carBatteryVoltages[i] && carVoltage <= carBatteryVoltages[i + 1]) {
      float slope = (liIonVoltages[i + 1] - liIonVoltages[i]) / (carBatteryVoltages[i + 1] - carBatteryVoltages[i]);
      return liIonVoltages[i] + slope * (carVoltage - carBatteryVoltages[i]);
    }
  }
  return 0.0;
}
