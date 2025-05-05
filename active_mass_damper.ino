#include "MPU6050.h"    //accelerometer initialisation
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
struct MyData {
  byte X;
  byte Y;
  byte Z;
};
MyData data;

const int controlPin1 = 3;                // connected to pin 7 on the H-bridge
const int controlPin2 = 9;                // connected to pin 2 on the H-bridge
const int enablePin = 10;                  // connected to pin 1 on the H-bridge

int motorEnabled = 1;    // Turns the motor on/off
int motorDirection = 1;  // current direction of the motor
int offset = 0;

//PID constants
double kp = 2;
double ki = 0;
double kd = 0.1;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

void setup(){
  delay(1500);
  // initialize the inputs and outputs
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  mpu.initialize();
  digitalWrite(enablePin, HIGH);
  
  //open serial port
  Serial.begin(9600);
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.X = map(ax, -17000, 17000, 0, 255);
  
  //offset = data.X-123;
  offset=ax;
  setPoint = 0-offset;                  //set point at zero degrees
  earthquake();
}    

void loop(){
  while(millis()<15000){
  delayMicroseconds(120);
  //printing top and bottom line for the plot
  int boundry=4000;
  Serial.print("Upper:");
  Serial.print(boundry);
  Serial.print(", ");
  Serial.print("Lower:");
  Serial.print(-boundry);
  Serial.print(", ");
  
  //input
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //data.X = map(ax, -17000, 17000, 0, 1023 ); // X axis data
        
  input = data.X-offset;
  Serial.print("Deviation(input):");
  Serial.print(ax-offset-100);
  Serial.print(", ");
  //output
  output = computePID(ax-offset);
  //Serial.print("Motorspeed(output):");
  //Serial.print(output);
 // Serial.print(", ");
  
  
  if (millis()>10577){
  Serial.print("Motorspeed(output):");
  Serial.print(0);
  Serial.print(", ");
  
  } else {
    Serial.print("Motorspeed(output):");
  Serial.print(0);
  Serial.print(", ");
  }
  
  //motordirection
  if (output>=0) {
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);
    Serial.print("RIGHT");

  } else{
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
    Serial.print("LEFT");
  }
  Serial.println(", ");
  
  //motospeed
  if (abs(input)<300) {motorEnabled = 0;
  } else {
    motorEnabled = 0;               //<--------------------------------------------------- ON/OFF DAMPING
  }
  /*if (motorEnabled == 1) {
    analogWrite(enablePin, output);     // PWM the enable pin to vary the speed
    Serial.print("Motorspead:");
    Serial.print(output);
    Serial.println(", ");
  } else {
    analogWrite(enablePin, 0); 
    Serial.print("Motorspead:");
    Serial.print(0);
    Serial.println(", ");
  }*/
  }
  Serial.end();
}

double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = 0 - inp;                                // determine error (setpoint=0)
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative

        double out = kp*error + ki*cumError + kd*rateError;                //PID output               

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
}

void earthquake(){
  for(int i=0; i<5; i++) {
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
    analogWrite(enablePin, 1023);
    delay(520);
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);
    analogWrite(enablePin, 1023);
    delay(520);
  }
  digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
    analogWrite(enablePin, 1023);
    delay(100);
    analogWrite(enablePin, 0);
}