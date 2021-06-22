/**
 * Firmware for reading slider box with 4 sliders and E-Stop.
 * 
 * Author: Julian Viereck
 * Date: 30 January 2020
 */

int ESTOP_PIN = 2;

void setup() {
  pinMode(ESTOP_PIN, INPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(50);
}

void loop() {
  // Read the ESTOP pin.
//  int estop = analogRead(A2);
//  if (estop > 10) {
//    Serial.print("0 ");
//  } else {
//    Serial.print("1 ");
//  }
//  Serial.print(analogRead(i));
  Serial.print(digitalRead(ESTOP_PIN));
  Serial.print(" ");

  // Read the analog sliders.
  for (int i = A0; i <= A1; i++) {
    Serial.print(analogRead(i));
    Serial.print(" ");
  }
  Serial.println("0 0 ");
  
  // Slow down the writing a bit.
  delayMicroseconds(500);
}
