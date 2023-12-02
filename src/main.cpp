#include <Arduino.h>
#include <Wire.h>

unsigned long prev, next, interval1;
// interval1:ultrasonic

// OLED
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
Adafruit_SSD1306 display(-1);

// BNO055
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float dir, dir_o, dir_x, dir_y, dir_z;
// PID制御
float kp = 2, ki = 0, kd = 0;  // 係数
float target = 0;              // 目標
float dt, preTime;
float P, I, D, pose_out, preP, vol;

// ball sensor
#define ball1 A2
#define ball2 A3
#define ballpower 42  // センサ電源
#define mp_a 35
#define mp_b 36
#define mp_c 37
int ballval[16];
int multiplex[3] = {mp_c, mp_b, mp_a};

// line sensor
#define led 32
#define line0 A4
#define line1 A5
#define line2 A6
#define line3 A7
#define line4 A8
#define line5 A9
#define line6 A10
#define line8 A11
#define line9 A12
#define line10 A13
#define line11 A14
#define line12 A15
#define touch A1

// ultorasonic
#include <NewPing.h>
#define u_left 38
#define u_front 39
#define u_right 40
#define u_back 41
#define MAX_DISTANCE 400
int u_val[4];  // left,front,right,back
int u_x, u_y;
NewPing sonar[4] = {NewPing(u_left, u_left, MAX_DISTANCE),
                    NewPing(u_front, u_front, MAX_DISTANCE),
                    NewPing(u_right, u_right, MAX_DISTANCE),
                    NewPing(u_back, u_back, MAX_DISTANCE)};

// MD
const int md1[] = {2, 3};
const int md2[] = {6, 5};
const int md3[] = {8, 7};
const int md4[] = {12, 11};
const int md5[] = {10, 9};
int motor1, motor2, motor3, motor4, motor5;

// UI
#define button_down 22
#define button_up 23
#define button_a 24
#define button_b 25
#define button_c 26
#define tgl 27
#define item 7  // led,gyro,ball,line,touch,motor,ultorasonic
bool up_status, down_status, led_status;
int menu;

float pose() {
  vol = dir;

  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P = target - vol;
  I += P * dt;
  D = (P - preP) / dt;
  preP = P;
  return (kp * P + ki * I + kd * D);
}

void motor_out(int num, double speed) {  // motor_out(1~5,-100~100)
  if (speed > 100) {
    speed = 100;
  } else if (speed < -100) {
    speed = -100;
  }
  if (num == 1) {
    if (speed == 0) {
      digitalWrite(md1[0], LOW);
      digitalWrite(md1[1], LOW);
    } else if (speed < 0) {
      analogWrite(md1[0], speed * -2.55);
      digitalWrite(md1[1], LOW);
    } else if (speed > 0) {
      digitalWrite(md1[0], LOW);
      analogWrite(md1[1], speed * 2.55);
    }
  } else if (num == 2) {
    if (speed == 0) {
      digitalWrite(md2[0], LOW);
      digitalWrite(md2[1], LOW);
    } else if (speed < 0) {
      analogWrite(md2[0], speed * -2.55);
      digitalWrite(md2[1], LOW);
    } else if (speed > 0) {
      digitalWrite(md2[0], LOW);
      analogWrite(md2[1], speed * 2.55);
    }
  } else if (num == 3) {
    if (speed == 0) {
      digitalWrite(md3[0], LOW);
      digitalWrite(md3[1], LOW);
    } else if (speed < 0) {
      analogWrite(md3[0], speed * -2.55);
      digitalWrite(md3[1], LOW);
    } else if (speed > 0) {
      digitalWrite(md3[0], LOW);
      analogWrite(md3[1], speed * 2.55);
    }
  } else if (num == 4) {
    if (speed == 0) {
      digitalWrite(md4[0], LOW);
      digitalWrite(md4[1], LOW);
    } else if (speed < 0) {
      analogWrite(md4[0], speed * -2.55);
      digitalWrite(md4[1], LOW);
    } else if (speed > 0) {
      digitalWrite(md4[0], LOW);
      analogWrite(md4[1], speed * 2.55);
    }
  } else if (num == 5) {
    if (speed == 0) {
      digitalWrite(md5[0], LOW);
      digitalWrite(md5[1], LOW);
    } else if (speed < 0) {
      analogWrite(md5[0], speed * -2.55);
      digitalWrite(md5[1], LOW);
    } else if (speed > 0) {
      digitalWrite(md5[0], LOW);
      analogWrite(md5[1], speed * 2.55);
    }
  }
}

void set_multiplex(int pin_arr[3], int channel) {
  if (channel == 0) {
    digitalWrite(pin_arr[0], LOW);
    digitalWrite(pin_arr[1], LOW);
    digitalWrite(pin_arr[2], LOW);
  } else if (channel == 1) {
    digitalWrite(pin_arr[0], LOW);
    digitalWrite(pin_arr[1], LOW);
    digitalWrite(pin_arr[2], HIGH);
  } else if (channel == 2) {
    digitalWrite(pin_arr[0], LOW);
    digitalWrite(pin_arr[1], HIGH);
    digitalWrite(pin_arr[2], LOW);
  } else if (channel == 3) {
    digitalWrite(pin_arr[0], LOW);
    digitalWrite(pin_arr[1], HIGH);
    digitalWrite(pin_arr[2], HIGH);
  } else if (channel == 4) {
    digitalWrite(pin_arr[0], HIGH);
    digitalWrite(pin_arr[1], LOW);
    digitalWrite(pin_arr[2], LOW);
  } else if (channel == 5) {
    digitalWrite(pin_arr[0], HIGH);
    digitalWrite(pin_arr[1], LOW);
    digitalWrite(pin_arr[2], HIGH);
  } else if (channel == 6) {
    digitalWrite(pin_arr[0], HIGH);
    digitalWrite(pin_arr[1], HIGH);
    digitalWrite(pin_arr[2], LOW);
  } else if (channel == 7) {
    digitalWrite(pin_arr[0], HIGH);
    digitalWrite(pin_arr[1], HIGH);
    digitalWrite(pin_arr[2], HIGH);
  }
}
void read_ball() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(ballpower, LOW);
    delayMicroseconds(10);
    set_multiplex(multiplex, i);
    digitalWrite(ballpower, HIGH);
    delayMicroseconds(10);
    // 後ろから時計回り
    ballval[i * 2] = analogRead(ball1);
    ballval[i * 2 + 1] = analogRead(ball2);
  }
}

void read_gyro() {
  sensors_event_t event;
  bno.getEvent(&event);

  dir_x = event.orientation.x;
  dir_y = event.orientation.y;
  dir_z = event.orientation.z;
  if (digitalRead(button_a) == 0) {
    dir_o = event.orientation.x;
  }
  dir = dir_x - dir_o;
  if (dir > 180) {
    dir -= 360;
  } else if (dir < -180) {
    dir += 360;
  }
}

void read_ultrasonic() {
  unsigned long curr = millis();  // 現在時刻を取得
  if ((curr - prev) >=
      interval1) {  // 前回実行時刻から実行周期以上経過していたら
    for (int i = 0; i < 4; i++) {
      delay(50);
      u_val[i] = sonar[i].ping_cm();
    }
    prev += interval1;  // 前回実行時刻に実行周期を加算
  }
}

void UI() {
  if (digitalRead(button_up) != up_status && digitalRead(button_up) == 0) {
    menu++;
  }
  up_status = digitalRead(button_up);

  if (digitalRead(button_down) != down_status &&
      digitalRead(button_down) == 0) {
    menu--;
  }
  down_status = digitalRead(button_down);

  if (menu < 0) {
    menu = item;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  if (0) {
  } else if (menu % item == 0) {
    // led
    display.setCursor(0, 0);
    display.println("LED");
    if (digitalRead(button_a) == 0) {
      digitalWrite(led, HIGH);
      led_status = 1;
    }
    if (digitalRead(button_b) == 0) {
      digitalWrite(led, LOW);
      led_status = 0;
    }
    if (led_status == 0) {
      display.setTextSize(2);
      display.println("OFF");
    } else {
      display.setTextSize(2);
      display.println("ON");
    }
  } else if (menu % item == 1) {
    // bno055
    read_gyro();
    display.setCursor(0, 0);
    display.println("BNO055");
    display.println("");
    display.print("dir: ");
    display.println(dir);
    display.print(" X : ");
    display.println(dir_x);

    display.display();
  } else if (menu % item == 2) {
    // ball
    display.setCursor(0, 0);
    display.println("BALL");
    display.println("");

    display.println(analogRead(ball1));
  } else if (menu % item == 3) {
    // line
    display.setCursor(0, 0);
    display.println("LINE");
  } else if (menu % item == 4) {
    // touch
    display.setCursor(0, 0);
    display.println("TOUCH");
    display.println("");

    display.println(analogRead(touch));
  } else if (menu % item == 5) {
    // motor
    display.setCursor(0, 0);
    display.println("MOTOR");
    motor_out(1, 0);
    motor_out(2, 0);
    motor_out(3, 0);
    motor_out(4, 0);
    motor_out(5, 0);
    if (digitalRead(button_a) == 0) {
      motor_out(1, 100);
      motor_out(2, 100);
      motor_out(3, 100);
      motor_out(4, 100);
    }
    if (digitalRead(button_b) == 0) {
      motor_out(1, -100);
      motor_out(2, -100);
      motor_out(3, -100);
      motor_out(4, -100);
    }
  } else if (menu % item == 6) {
    // ultorasonic sensor
    read_ultrasonic();
    display.setCursor(0, 0);
    display.println("ULTORASONIC");

    display.setCursor(0, 16);
    display.print("front: ");
    display.println(u_val[1]);
    display.setCursor(0, 24);
    display.print("back : ");
    display.println(u_val[3]);
    display.setCursor(72, 16);
    display.print("left : ");
    display.println(u_val[0]);
    display.setCursor(72, 24);
    display.print("right: ");
    display.print(u_val[2]);
  }
  display.display();
}

void soccer() { pose_out = pose(); }

void setup() {
  pinMode(ball1, INPUT);
  pinMode(ball2, INPUT);
  pinMode(ballpower, OUTPUT);
  pinMode(mp_a, OUTPUT);
  pinMode(mp_b, OUTPUT);
  pinMode(mp_c, OUTPUT);

  pinMode(touch, INPUT);
  pinMode(led, OUTPUT);
  pinMode(button_a, INPUT);
  pinMode(button_b, INPUT);
  pinMode(button_c, INPUT);
  pinMode(button_up, INPUT);
  pinMode(button_down, INPUT);
  pinMode(tgl, INPUT);

  Serial.begin(115200);
  Serial.println("Hello world");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(
        "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  prev = 0;         // 前回実行時刻を初期化
  interval1 = 500;  // 実行周期を設定
  // put your setup code here, to run once:
}

void loop() {
  if (digitalRead(tgl) == 1) {
    UI();
    read_ball();
    for (int i = 0; i < 16; i++) {
      Serial.print(ballval[i]);
      Serial.print("  ");
    }
    Serial.println("");
    delay(500);
  } else {
    soccer();
    display.clearDisplay();
    display.display();
  }

  /*
   */
  // put your main code here, to run repeatedly:
}