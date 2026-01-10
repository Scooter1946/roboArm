#include <Servo.h>

Servo motor1, motor2;
int id1 = 3, id2 = 5;

void setup() {
  Serial.begin(9600);
  motor1.attach(id1, 500, 2500);
  motor2.attach(id2, 500, 2500);
  motor2.write(180);
  motor1.write(0);
}

void loop() {
  Serial.println(motor2.read() * PI / 180.0);

}
