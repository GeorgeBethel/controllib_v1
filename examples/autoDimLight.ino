/*
 * Auto control light intensity example
 * Written by George Okoroafor july 13, 2023
 * 
 * This example is for maintaining the ligh intensity of light depending on the light in the abient environment. 
 * This behaves more like your phone backlight on auto. When out in the sun, the light intensity increases and when in the dark, it reduces to maintain the same level
 * intensity.
 * 
 * Parts used for this example
 * 
 * ** - LDR
 * ** - LED
 * ** - Resistors (use a voltage divide)
 * 
 * 
 */

#include "controller.h"

#define LED_pin 6
#define LDR_pin A0

float input = 0.0;
float output = 0.0;
float kp = 1000.0;
float kd = 0.0;
float ki = 0.0;
int freq = 2; // controller runs every 1/2 miliseconds or 0.5 miliseconds 
String type = "P";
bool haslimit = true;
double upperlim = 255.0;
double lowerlim = 0.0;
float set_point = 5.0;

controllers::PID LED_control(kp,kd,ki,freq,type,haslimit); 


void setup() {
  // put your setup code here, to run once:

  pinMode(LED_pin, OUTPUT);
  pinMode(LDR_pin, INPUT);
  LED_control.setLimits(upperlim,lowerlim);
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  input = analogRead(LDR_pin);
  
  LED_control.computeOutput(input,set_point);
  
  output = LED_control.getOutput();
  
  analogWrite(LED_pin, output);

 
}