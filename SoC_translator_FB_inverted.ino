const int analogPinInput = A0; // Pin connected to the input voltage divider
const int analogPinOutput = A1; // Pin connected to the output voltage divider
const int pwmPin = 11;          // PWM output pin

const float R1 = 10000.0;       // 10kΩ for input voltage divider
const float R2 = 2000.0;        // 2kΩ for input voltage divider
const float R3 = 10000.0;       // 10kΩ for output voltage divider
const float R4 = 2000.0;        // 2kΩ for output voltage divider

const float carBatteryVoltages[] = {11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.4};
const float liIonVoltages[] = {3.0, 3.3, 3.6, 3.8, 4.0, 4.1, 4.2, 4.2};
const int numPoints = sizeof(carBatteryVoltages) / sizeof(carBatteryVoltages[0]);

// PID control variables
float kp = 1.0;  // Proportional gain
float ki = 0.1;  // Integral gain
float kd = 0.05; // Derivative gain

float previousError = 0.0;
float integral = 0.0;

void setup() {
  Serial.begin(9600);
  
  pinMode(pwmPin, OUTPUT);

  // Configure Timer2 for a lower PWM frequency (e.g., 1 kHz)
  TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM mode, non-inverting
  TCCR2B = (1 << WGM22) | (1 << CS21);  // Prescaler 8 for lower frequency
  
  OCR2A = 128; // Start with 50% duty cycle
}

void loop() {
  float carBatteryVoltage = readVoltage(analogPinInput, R1, R2);
  float targetLiIonVoltage = interpolateLiIonVoltage(carBatteryVoltage);
  float actualLiIonVoltage = readVoltage(analogPinOutput, R3, R4);

  float error = targetLiIonVoltage - actualLiIonVoltage;
  float proportional = kp * error;

  integral += error;
  float integralTerm = ki * integral;

  float derivative = kd * (error - previousError);
  previousError = error;

  float pidOutput = proportional + integralTerm + derivative;

  // Inverted PWM logic: Map PID output to PWM signal (0-255)
  int pwmValue = constrain(map(pidOutput, 0, 5, 255, 0), 0, 255);

  OCR2A = pwmValue; // Set PWM duty cycle

  Serial.print("Car Battery Voltage: ");
  Serial.print(carBatteryVoltage);
  Serial.print(" V, Target Li-Ion Voltage: ");
  Serial.print(targetLiIonVoltage);
  Serial.print(" V, Actual Li-Ion Voltage: ");
  Serial.print(actualLiIonVoltage);
  Serial.print(" V, PWM: ");
  Serial.print(pwmValue);
  Serial.println();

  delay(100); // Wait for 100 ms before the next cycle
}

float readVoltage(int pin, float r1, float r2) {
  int rawValue = analogRead(pin);
  float voltage = (rawValue / 1023.0) * 5.0; // Convert to voltage (0-5V)
  return voltage * (r1 + r2) / r2; // Convert to actual voltage
}

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
