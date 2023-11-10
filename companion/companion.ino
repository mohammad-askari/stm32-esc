void setup() {
  // Initialize the hardware serial port for communication with your computer
  Serial.begin(115200);

  // Initialize the software serial port
  Serial1.begin(115200);
  pinMode(LEDG, OUTPUT);
}

void loop() {
  // Read from the USB serial port and send to the G431B serial port
  if (Serial.available()) {
    char data = Serial.read();
    Serial1.write(data);
  }

  // Read from the G431B serial port and send to the USB serial port
  if (Serial1.available()) {
    char data = Serial1.read();
    digitalWrite(LEDG, 0);
    Serial.write(data);
  }

  if(millis()%300 == 0){
    digitalWrite(LEDG, 1);
  }
}