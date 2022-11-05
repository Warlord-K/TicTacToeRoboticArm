#include <Servo.h>
#define pwm_1 10
#define dir_1 11
Servo arm1servo;
Servo arm2servo;
Servo baseservo;
Servo servo4;
Servo gripper;
int topLeftid=8;
int topMiddleid=7;
int topRightid =6;
int middleRightid=3;
double CurrentX=100;
double CurrentY=100;
double CurrentZ=100;
double diff = 50;
int rec_move=0;
double Pickup[3] = {185,0,100};
double topMiddle[3] = {-30,110,40};
double topMid[3] = {topMiddle[0]-15,topMiddle[1]-diff,topMiddle[2]};
double topRight[3] = {topMiddle[0]-25,topMiddle[1]-diff+5,topMiddle[2]};
double topLeft[3] = {topMiddle[0]-10,topMiddle[1]+diff-10,topMiddle[2]};
double middleMiddle[3] = {topMiddle[0]-diff,topMiddle[1],topMiddle[2]};
double middleRight[3] = {topMiddle[0]-diff-18,topMiddle[1]-diff-5,topMiddle[2]};
double middleLeft[3] = {topMiddle[0]-diff+5,topMiddle[1]+diff,topMiddle[2]};
double bottomMiddle[3] = {topMiddle[0]-2*diff+15,topMiddle[1]-5,topMiddle[2]};
double bottomRight[3] = {topMiddle[0]-2*diff,topMiddle[1]-diff-10,topMiddle[2]};
double bottomLeft[3] = {topMiddle[0]-2*diff+20,topMiddle[1]+diff,topMiddle[2]};
double origin[3] =  {0,0,0};
void setup() {
  // put your setup code here, to run once:
  baseservo.attach(3,460 ,2400);
  arm1servo.attach(5,460 ,2400);
  arm2servo.attach(6,460 ,2400);
  servo4.attach(9,460 ,2400);
  pinMode(pwm_1, OUTPUT);
  pinMode(dir_1, OUTPUT);
  // gripper.attach(10,460,2400);
  Serial.begin(115200);
  Serial.setTimeout(10);
  moveToPos(CurrentX,CurrentY,CurrentZ);
  delay(5000);
}

int angleToMicroseconds(double angle) {
  double val = 460.0 + (((2400.0 - 460.0) / 180.0) * angle);
  return (int)val;
}

void moveToAngle(double b, double a1, double a2, double a3) {
  arm1servo.writeMicroseconds(angleToMicroseconds(a1));
  arm2servo.writeMicroseconds(angleToMicroseconds(a2));
  baseservo.writeMicroseconds(angleToMicroseconds(b));
  servo4.writeMicroseconds(angleToMicroseconds(a3));
}

void moveToPos(double x, double y, double z) {
  double b = atan(y/x) * (180 / 3.1415); // base angle

  double l = sqrt(x*x + y*y); // x and y extension

  double h = sqrt (l*l + z*z);

  double phi = atan(z/l) * (180 / 3.1415);

  double theta = acos((h/2)/120) * (180 / 3.1415);
 
  double a1 = phi + theta; // angle for first part of the arm
  double a2 = 2*theta; // angle for second part of the arm
  double a3 = 64 + (phi - theta);
  moveToAngle(b,a1,a2,a3);
}
void openJaw(){
    gripper.writeMicroseconds(angleToMicroseconds(90));
 
  }
void closeJaw(){
    gripper.writeMicroseconds(angleToMicroseconds(50));
  }


void fromTo(double x1, double y1, double z1, double x2, double y2, double z2) {
    double dx= (x2-x1)/100;
    double dy= (y2-y1)/100;
    double dz= (z2-z1)/100;
    for(int j=0;j<101;j++){
     moveToPos(x1+dx*j,y1+dy*j,z1+dz*j);
     delay(20);
     }
 
  }
double signum(double x){
  if(x==0){
    return 0;
  }
  return x/abs(x);
}
void moveTo(double x1, double y1,double z1){
    double dx= 5;
    double dy= 5;
    double dz= 5;
    while(!(abs(-CurrentX+x1)+abs(-CurrentY+y1)+abs(-CurrentZ+z1) == 0)){
      dx=2;
      dy=2;
      dz=2;
      if(abs(-CurrentX+x1) < dx){
        dx=abs(-CurrentX+x1);
      }
      if(abs(-CurrentY+y1) < dy){
        dy=abs(-CurrentY+y1);
      }
      if(abs(-CurrentZ+z1) < dz){
        dz=abs(-CurrentZ+z1);
      }
      dx = dx *signum(-CurrentX+x1);
      dy = dy *signum(-CurrentY+y1);
      dz = dz *signum(-CurrentZ+z1);
     moveToPos(CurrentX+dx,CurrentY+dy,CurrentZ+dz);
     
     CurrentX=CurrentX+dx;
     CurrentZ=CurrentZ+dz;
     CurrentY=CurrentY+dy;
    //  Serial.print(dx);
    //  Serial.print("Current X: \n");
    //  Serial.print(CurrentX);
    //  Serial.print("Current y: \n"); 
    //  Serial.print(CurrentY);
    //  Serial.print("Current z: \n");
    //  Serial.print(CurrentZ);
    //  Serial.print("Moved\n");
     delay(25);
     }
}
void move(double Loc[3],double time=1000){
  moveTo(Pickup[0]+Loc[0]-25,Pickup[1]+Loc[1],Pickup[2]+Loc[2]);
  delay(time);
}
double* moveArm(int x){
  double* moves[9] = {bottomRight,bottomMiddle,bottomLeft,middleRight,middleMiddle,middleLeft,topRight,topMiddle,topLeft};
  return moves[x];
}
void suctionON(double dir,double tim=5500){
  if(dir ==1){
  digitalWrite(pwm_1,HIGH);
  digitalWrite(dir_1,LOW);
  delay(tim);
  }else if(dir==-1){
  digitalWrite(pwm_1,LOW);
  digitalWrite(dir_1,HIGH);
  delay(tim);
  }else{
    digitalWrite(pwm_1,LOW);
    digitalWrite(dir_1,LOW);
  }
}
void suctionOFF(){
digitalWrite(pwm_1,LOW);
digitalWrite(dir_1,LOW);
}
void calibrate(int x){
    // Serial.print("Started \n");
    // move(origin);
    // move(topRight);
    // move(topMiddle);
    // move(topLeft);
    // move(middleRight);
    // move(middleMiddle);
    // move(middleLeft);
    // move(bottomRight);
    // move(bottomMiddle);
    // move(bottomLeft);
    // Serial.print("Reached\n");
    double* loc;
    rec_move = x;
    move(origin);
    loc = moveArm(rec_move-1);
    pickpiece();
    move(loc);
    droppiece();
    move(origin);
}
double downdis = 35;
void pickpiece(){
  double down[3] = {0,0,0};
  down[2]-=downdis;
  downdis+=5;
  move(down,0);
  suctionON(1);
  suctionOFF();
  move(origin,0);  
}
void droppiece(){
  // double down[3] = {CurrentX,CurrentY,CurrentZ};
  // down[2]-=20;
  // move(down,time=0);
  suctionON(-1);
  suctionOFF();
}
void play(){
      move(origin);
    while (!Serial.available());
    rec_move = Serial.readString().toInt();
    Serial.print("Recieved Move: ");
    Serial.print(rec_move);
    Serial.print("\n");
    double* loc;
    if(rec_move!=0){
    pickpiece();
    loc = moveArm(rec_move-1);
    move(loc);
    droppiece();
    move(origin);
    rec_move=0;
    }
}

void calibratecup(){

  int pwm_value = 0;
Serial.println("Sending High");
digitalWrite(pwm_1,HIGH);
digitalWrite(dir_1,LOW);
delay(5000);
Serial.println("Sending Low");
digitalWrite(pwm_1,LOW);
digitalWrite(dir_1,HIGH);
delay(5000);
digitalWrite(pwm_1,LOW);
digitalWrite(dir_1,LOW);
}
void loop() {
play();
// for(int i=1;i<6;i++){
//   calibrate(i);
// }
// while(1){
//   continue;
// } 
// move(origin);
// pickpiece();
// droppiece();
// suctionON(-1);
// Serial.println("Pause");
// delay(2000);
}