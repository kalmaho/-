#define PIN_SERVO 10
#define PIN_IR A0 
#include <Servo.h>
#define PIN_LED 9
// Framework setting
#define _DIST_TARGET 260//[3104] 탁구공을 위치 시킬 목표 
#define _DIST_MIN 100 //[3117] 거리 최소값 //[3104] 측정 거리 최소치 
#define _DIST_MAX 400//[3117] 거리 최대값//[3104] 측정 거리 최대치

// Distance sensor
#define _DIST_ALPHA 0.8 // EMA 필터링을 위한 alpha 

// Servo range
#define _DUTY_MIN 1000     //[3100] 최저 서보 위치
#define _DUTY_NEU 1450     //[3100] 중립 서보 위치
#define _DUTY_MAX 1950     //[3100] 최대 서보 위치


// Servo speed control
#define _SERVO_ANGLE 50.0 // 서보각도
#define _SERVO_SPEED 60.0 //서보속도

// Event periods
#define _INTERVAL_DIST 10 //[3099] 각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 10//20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.5//3//0.05//1.5// [3103] KP값은 개인이 설정
#define _KD 120//0.2
#define _KI 0.006
// [3108] dist 100, 400mm 일때 값, 각자 a,b로 수정
#define a 100//100
#define b 400//400
//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; //[1928] 측정된 값과 ema 필터를 적용한 값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//[3104] 각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; 
//[3104] 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval; // [3116] 주기 당 서보 duty값 변화량
int duty_target, duty_curr; //[1928] 목표 위치와 현재 위치
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//error_curr: 현재 측정값과 목표값의 차이
//error_prev: 직전에 구한 차이로, P제어에서는 사용하지 않을 것임
//control: PID제어의 결과로 얻은 제어값
//pterm: Proportional term, 현재 상태의 error값으로부터 얻은 Proportional gain을 저장하는 변수
// [3099]

void setup() {
// initialize GPIO pins for LED and attach servo 
myservo.attach(PIN_SERVO); // attach servo
pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

// initialize global variables
iterm=0;
// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);
duty_curr = _DUTY_NEU;

// initialize serial port
Serial.begin(57600);
//error_prev = 0;//*************************************************
// convert angle speed into duty change per interval.
duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (_SERVO_ANGLE)) * (_INTERVAL_SERVO / 1000.0); 
  // [3099] (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (_SERVO_ANGLE * 2) * (_INTERVAL_SERVO / 1000.0)
}
  

void loop() {
/////////////////////
// Event generator //
///////////////////// 

unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
}

if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
}

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
     dist_ema = ir_distance_filtered();
     
    error_curr = dist_ema -_DIST_TARGET ;
    
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    //iterm = 0;
    //dterm = 0;
    control = pterm + iterm + dterm;
    duty_target = _DUTY_NEU - control;//

    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    else if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    error_prev = error_curr;

  }
  
  if(event_servo) {
    event_servo=false;
    if(duty_target>duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
     }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }   

  if(event_serial) {
    event_serial = false; //[3117] // 이거 맞나요? // 저도 이렇게 했어요
    // 아래 출력문은 수정없이 모두 그대로 사용하기 바랍니다.
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
  }
}

float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (value    - a) + 100;
}
//[3099]

float ir_distance_filtered(void){ // return value unit: mm
  dist_raw = ir_distance();
  return _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
} 
