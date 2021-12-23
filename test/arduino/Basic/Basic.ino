/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  // Serial.begin(9600);
  // Serial.begin(9600, SERIAL_5N1); // 5 data bits, no parity, 1 stop bit
  Serial.begin(9600, SERIAL_8E2); // 8 data bits, even parity, 2 stop bits
//  Serial.begin(81234); // Used to test custom baud rates
}

// the loop routine runs over and over again forever:
void loop() {
  Serial.println("Hello");
  delay(100);        // delay in between reads for stability
}
