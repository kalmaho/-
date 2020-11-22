// Arduino pin assignment
#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

Servo myservo;
//1100up 1900down
//1500 수평
//dis->25->260
//myservo.attach(PIN_SERVO); 
//myservo.writeMicroseconds(1500);
void setup() {
  myservo.attach(PIN_SERVO); 
  int angle =0; 
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
int angle =0;
void loop() {
  float raw_dist = ir_distance();
  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
  delay(20);
  
  if(raw_dist>260)
  {
    if(angle>=1500)
    {
     for(angle = 1900; angle>1100; angle--)
     {
        myservo.writeMicroseconds(angle);
        Serial.println(angle);
        delay(2);
        
     }
    }
  }
  else
 {
    if(angle<=1500)
    {
      for(angle = 1100; angle<1900; angle++)
      {
        myservo.writeMicroseconds(angle);
        delay(2);
      }
    }
 }
}
