#include <Servo.h>

//motors
Servo motor1, motor2;
int id1 = 3, id2 = 5;
//lengths of arms
double armlen = 1;
//x/y position of arm
double pos[2];
//change of distance w/respect to change of angle (size 2)
double* angleDerivs;
//target x/y position
double target[2] = {sqrt(2), sqrt(2)};
//threshold for how close is good enough
double distThresh = 0.05;

//applies a transformation matrix
void applyTransform(double orig[2], double transform[2][2]){
  double temp = orig[0];
  //the fucking applied the transformation wrong
  orig[0] = orig[0] * transform[0][0] + orig[1] * transform[0][1];
  orig[1] = temp * transform[1][0] + orig[1] * transform[1][1];
}

//ensures the target point is within the range of the arm's motion
void getBoundedTarget(){
  if(pow(target[0], 2) + pow(target[1], 2) > pow(2 * armlen, 2)){
    double ratio = (2 * armlen) / sqrt(pow(target[0], 2) + pow(target[1], 2));
    target[0] = target[0] * ratio;
    target[1] = target[1] * ratio; 
  }
}

//gets distance between two points using the distance formula
double getDist(double x1, double y1, double x2, double y2){
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

//returns the derivative of distance from the point with respect to each angle
double* findSlopes(double l, double x1, double y1, double x2, double y2, double t1, double t2){
  double* ans = (double*)malloc(sizeof(double) * 2);
  ans[0] = (-1) * ((x2 - x1) / getDist(x1, y1, x2, y2));
  ans[0] *= l * (sin(t1) + sin(t1) * cos(t2) + sin(t2) * cos(t1));
  ans[0] += (-1) * ((y2 - y1) / getDist(x1, y1, x2, y2)) * l * (-sin(t1) * sin(t2) + cos(t1) + cos(t1) * cos(t2));
  ans[1] = (-1) * ((x2 - x1) / getDist(x1, y1, x2, y2));
  ans[1] *= l * (cos(t1) * sin(t2) + sin(t1) * cos(t2));
  ans[1] += (-1) * ((y2 - y1) / getDist(x1, y1, x2, y2)) * l * (cos(t1) * cos(t2) - sin(t1) * sin(t2));
  return ans;
}

void setup() {
  motor1.attach(id1, 500, 2500);
  motor2.attach(id2, 500, 2500);
  Serial.begin(9600);
}

void loop() {
  //theta1 and theta2
  double t1 = motor1.read() * PI / 180.0;
  double t2 = motor2.read() * PI / 180.0;
  
  //LOCATE ITSELF
  //the position of the second motor relative to the origin
  pos[0] = -cos(t1) * armlen;
  pos[1] = sin(t1) * armlen;
  //second arm's 90 degree position is perpendicular to the first arm
  double transform[2][2] = {{1 + cos(t2), sin(t2)}, 
                         {-sin(t2), 1 + cos(t2)}};
  applyTransform(pos, transform);

  //FIND DERIVATIVE RELATIVE TO THE TWO ANGLES
  angleDerivs = findSlopes(armlen, pos[0], pos[1], target[0], target[1], t1, t2); 
  if(abs(getDist(pos[0], pos[1], target[0], target[1])) <= distThresh) return;
  //need to convert back into degrees
  motor1.write((t1 - angleDerivs[0] * 0.04) / PI * 180);
  motor2.write((t2 - angleDerivs[1] * 0.04) / PI * 180);
  logValues(t1, t2);
}

//logs all relevant values (for debugging)
void logValues(double t1, double t2){
  Serial.print("t1: ");
  Serial.println(t1);
  Serial.print("t2: ");
  Serial.println(t2);
  Serial.print("angle derivs: [");
  Serial.print(angleDerivs[0]);
  Serial.print(", ");
  Serial.print(angleDerivs[1]);
  Serial.println("]");
  
  Serial.print("current pos: [");
  Serial.print(pos[0]);
  Serial.print(", ");
  Serial.print(pos[1]);
  Serial.print("];  target pos: [");
  Serial.print(target[0]);
  Serial.print(", ");
  Serial.print(target[1]);
  Serial.println("]");
}
