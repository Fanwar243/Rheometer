
#include <Wire.h>
#include <DFRobot_INA219.h>
 
const int PWM = 5;
const int IN_3 = 3;
const int IN_4 = 4;
const int HALL = 2;
const int RESOLUTION = 22; //double channel, double-edge

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);
 
int motorPWM = 35; //inital PWM value
volatile long encoderValue = 0;
float intercept = 25.00; //no-load current, y-intercept of current-torque graph

//measured values
float current;
int rpm = 0;
int listIndex = 0;
float rpmList[20];
float currentList[20];
float rateList[20]; //array of shear rates calculated
float stressList[20]; //array of shear stresses calculated
 
//timekeeping
long previousMillis = 0;
long currentMillis = 0;
const int interval = 1000;
 
//rheometer dimensions in mm
int bobRadius = 8;
int bobLength = 16;
float beakerRadius = 28.25;

float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;
 
void updateEncoder() {
  encoderValue++;
}
 
void setup() {
 
  Serial.begin(9600);
  pinMode(PWM, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(HALL), updateEncoder, CHANGE);
 
  //turn motor off initially
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);

  //initialize wattmeter
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
  Serial.print("Operating voltage: ");
  Serial.println(ina219.getBusVoltage_V());
  Serial.println("");
  delay(500);
 
  previousMillis = millis();

}

void loop() {

  //start spinning
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(PWM, motorPWM);
  delay(1000);
 
  //take measurements each second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) 
  {
    
    previousMillis = currentMillis;
    rpm = float(encoderValue * 60 / RESOLUTION);
    if ( rpm > 0 && listIndex <= 19 )
    {
      Serial.print("Voltage: ");
      current = ina219.getCurrent_mA();
      Serial.print("Reading ");
      Serial.print(listIndex + 1);
      Serial.print("\t PWM VALUE: ");
      Serial.print(motorPWM);
      Serial.print("\t PULSES: ");
      Serial.print(encoderValue);
      Serial.print("\t RPM: ");
      Serial.print(rpm);
      Serial.print("\t CURRENT: ");
      Serial.print(current);
      Serial.println(" mA");
      rpmList[listIndex] = rpm;
      currentList[listIndex] = current;
      listIndex++;
    }
    encoderValue = 0;
    if (motorPWM <= 240) {
      increaseSpeed();
      raiseIntercept();
    }

  }
 
  // output results after 20 measurements
  if (listIndex == 20) {
    Serial.println("Ten measurements have been taken.");

    // decelerate the motor
    for (int i = 245; i > 45; i -= 15) {
      analogWrite(PWM, i);
      delay(500);
    }

    for (int i = 0; i < 20; ++i) {
      rateList[i] = shearRate(rpmList[i]);
      stressList[i] = shearStress(currentList[i]);
      Serial.print("Datapoint ");
      Serial.println(i + 1);
      Serial.print("RPM: ");
      Serial.print(rpmList[i]);
      Serial.print(" \t Shear rate: ");
      Serial.print(rateList[i]);
      Serial.print(" /s \t Shear stress: ");
      Serial.print(stressList[i]);
      Serial.print(" mPa");
      Serial.print("\t Viscosity: ");
      Serial.print(stressList[i] / rateList[i]);
      Serial.println(" mPa.s");
      delay(1000);
    }
 
    // stop motor spinning
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, LOW);
    Serial.println("");
    
    Serial.println("FOR PYTHON PARSING:");
    for (int i = 0; i < 20; ++i) {
      Serial.println(rateList[i]);
      delay(100);
    }
    for (int i = 0; i < 20; ++i) {
      Serial.println(stressList[i]);
      delay(100);
    }
    for (int i = 0; i < 20; ++i) {
      Serial.println(stressList[i] / rateList[i]);
      delay(100);
    }
    Serial.println("END PARSING");
    
    delay(5000);

  }

}
 
void increaseSpeed() {
  motorPWM += 10;
}

void raiseIntercept() {
  intercept += 0.8;
}

float shearStress(float current) {
  // Returns shear stress (in mPa) at specified current (in mA).
  //float measuredTorque = ( (current - intercept) / 1104.3 ); // in kgfcm
  float measuredTorque = ( (current - intercept) / 4147.8 ); // in kgfcm
  float y = 2 * PI * sq(bobRadius * 0.1) * (bobLength * 0.1); //cm^3
  return (measuredTorque / y) * 98066.5 * 1000; //kgfcm^-2 to mPa conversion
  //return (measuredTorque / y) * 98066.5 * 1000 * 1000; //for uPa
}
 
float shearRate(float rpm) {
  // Returns shear rate (in s^-1) at specified rpm.
  float speed = 30 * rpm / PI; //rad s^-1
  return 2 * speed * sq(beakerRadius) / ( sq(beakerRadius) - sq(bobRadius) );
}
