void setup() {
  pinMode(7, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
  }
     // wait for serial port to connect.
     digitalWrite(7,0); //LED ON
}

void loop() {
  int i=0;
  digitalWrite(7,0); //LED ON
  delay(1000);  //1초 딜레이

  for(int i=0;i<5;i++)
  {
      digitalWrite(7,1);  //LED OFF
      delay(100);  //0.1초 딜레이
      digitalWrite(7,0);
      delay(100);
  }
  digitalWrite(7,1);
  while(1)
  {
    
  }

}
