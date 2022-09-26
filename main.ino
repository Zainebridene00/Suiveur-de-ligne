int c1 = 2;
int c2 = 3;
int c3 = 4;
int c4 = 5;
int c5 = 6;
int c6 = 7;
int c7 = 8;
int c8 = 9;
int c9 = 12;
int c10 = 13;

int ena = 10;
int enb = 11;

int in1 = A1;
int in2 = A0;
int in3 = A3;
int in4 = A2;

float bms = 160;
float rms;
float lms;

float kp = 2.7;
float kd = 8;

float kpu = 1.5;
float kdu = 0;

int maxrms = 255;
int maxlms = 255;
int ms;

int lasterror = 0;
int lasterrorultra = 0;

float bc = 1;
float ac = 0.9;

int r = 0;
int h = 0;


int distanceRight;
int distanceLeft;
int distanceFront; 

int trigPinRight = 27;
int echoPinRight = 26; 

int trigPinFront = 25;
int echoPinFront = 24; 

int trigPinLeft = 23;
int echoPinLeft = 22; 


//////////////////////////////
void setup() {
  pinMode(c1,INPUT);
  pinMode(c2,INPUT);
  pinMode(c3,INPUT);
  pinMode(c4,INPUT);
  pinMode(c5,INPUT);
  pinMode(c6,INPUT);
  pinMode(c7,INPUT);
  pinMode(c8,INPUT);
  pinMode(c9,INPUT);
  pinMode(c10,INPUT);
  
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT); 

  pinMode(echoPinRight,INPUT);
  pinMode(echoPinLeft,INPUT);
  pinMode(echoPinFront,INPUT);

  pinMode(trigPinRight, OUTPUT);
  pinMode(trigPinFront, OUTPUT);
  pinMode(trigPinLeft, OUTPUT); 
  
  Serial.begin(9600);
  h = 4;

}

void loop() {


  if( h == 0){
    PID_5fif();
  }else if( h == 1){
    first_left();
  }else if( h == 2){
    initiate_circle_protocole();
  }else if( h == 3){
    second_left();
  }else if( h == 4){
    first_right();
  }else if( h == 5){
    maze_start();
  }else if( h == 6){
    maze_portal();
  }else if( h == 7){
    maze_first_part();
  }else if( h == 8){
    maze_second_portal();
  }else if( h == 9){
    maze_second_part();
  }else if( h == 10){
    maze_third_portal();
  }else if( h == 11){
    maze_last_part();
  }else if( h == 12){
    B_PID();
  }else if( h == 15){
    sharp_right();
  }else if( h == 16){
    sharp_left();
  }else if( h == 17){
    last_right();
  }else if( h == 18){
    ba3ba3();
  }
  //ultraPID();
  /*analogWrite(enb,100);
  analogWrite(ena,100);
  forward();
  delay(1000);
  right();
  delay(1000);
  left();
  delay(1000);
  wa55er();
  delay(1000);
  stops();
  delay(1000);*/


  /*Serial.print("capt1 : ");
  Serial.println(capt1);
  Serial.print("capt2 : ");
  Serial.println(capt2);
  Serial.print("capt3 : ");
  Serial.println(capt3);
  Serial.print("capt4 : ");
  Serial.println(capt4);
  Serial.print("capt5 : ");
  Serial.println(capt5);
  Serial.print("capt6 : ");
  Serial.println(capt6);
  Serial.print("capt7 : ");
  Serial.println(capt7);
  Serial.print("capt8 : ");
  Serial.println(capt8);
  Serial.print("capt9 : ");
  Serial.println(capt9);
  Serial.print("capt10 : ");
  Serial.println(capt10);
  delay(1000);*/
}


void ultraPID(){
  distanceRight = getDist(trigPinRight,echoPinRight);
  distanceLeft = getDist(trigPinLeft,echoPinLeft);
  int errorultra = distanceLeft - distanceRight;
  ms = kpu*errorultra + kdu*(errorultra - lasterrorultra);
  lms = bms + ms;
  rms = bms - ms;
  lasterrorultra = errorultra;
  if(lms > maxlms){
    lms = maxlms;
  }if(lms < 0){
    lms = 0;
  }if(rms < 0){
    rms = 0;
  }if(rms > maxrms){
    rms = maxrms;
  }
  analogWrite(enb,0.5*lms);
  analogWrite(ena,0.4*rms);
  forward();
}

void PID(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 
 int error= 100*capt1 + 70*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 70*capt9 - 100*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1){
  analogWrite(enb,90);
  analogWrite(ena,40);
  left();
 }else if(capt10){
  analogWrite(enb,40);
  analogWrite(ena,90);
  right();
 }else if(capt4||capt5||capt6||capt7||capt3||capt8||capt9||capt2){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}

void PID_5fif(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt1&&capt2){
  h=1;
  
  analogWrite(enb,255);
  analogWrite(ena,150);
  forward();
  delay(150);
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = 1.5*error +2*(error-lasterror);
 rms = 255 - ms;
 lms = 255 + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 analogWrite(enb,bc*lms);
 analogWrite(ena,ac*rms);
 forward();
 
  
}
void first_left(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt1&&capt2&&capt3&&capt4){
  h=2;
  stops();
  delay(50);
  analogWrite(enb,100);
  analogWrite(ena,70);
  left();
  delay(370);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = 255 - ms;
 lms = 255 + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt4||capt5||capt6||capt7||capt3||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
  
}
void initiate_circle_protocole(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt3&&capt4&&capt5&&capt6&&capt7&&capt8){
  h=3;
  stops();
  delay(50);
  analogWrite(enb,80);
  analogWrite(ena,70);
  left();
  delay(320);
  analogWrite(enb,0);
  analogWrite(ena,100);
  rights();
  delay(850);
  stops();
  delay(50);
  analogWrite(enb,100);
  analogWrite(ena,100);
  lefts();
  delay(350);
  stops();
  delay(50);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt4||capt5||capt6||capt7||capt3||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}


void second_left(){
 if (r == 0){
  r = 1;
  int c = 0;
  while (c<12000){
    c++;
    PID();
    PID();
  }

 }
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt1&&capt2&&capt3&&capt4){
  h=4;
  stops();
  delay(50);
  analogWrite(enb,100);
  analogWrite(ena,70);
  left();
  delay(300);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt4||capt5||capt6||capt7||capt3||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
  
}
void first_right(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt10&&capt9&&capt8&&capt7){
  h=5;
  stops();
  delay(50);
  analogWrite(enb,50);
  analogWrite(ena,90);
  right();
  delay(430);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt4||capt5||capt6||capt7||capt3||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}

void maze_start(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt10+capt9+capt8+capt7+capt6+capt5+capt4+capt3+capt2+capt1 == 0){
  h=6;
 }else{
  int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;
  
   ms = kp*error +kd*(error-lasterror);
   rms = bms - ms;
   lms = bms + ms;
   lasterror = error;
   if(lms > maxlms){
    lms = maxlms;
   }if(lms < 0){
    lms = 0;
   }if(rms < 0){
    rms = 0;
   }if(rms > maxrms){
    rms = maxrms;
   }
   if(capt1||capt2){
    analogWrite(enb,150);
    analogWrite(ena,150);
    lefts();
   }else if(capt10||capt9){
    analogWrite(enb,150);
    analogWrite(ena,150);
    rights();
   }else if(capt4||capt5||capt6||capt7||capt3||capt8){
    analogWrite(enb,bc*lms);
    analogWrite(ena,ac*rms);
    forward();
   }
 }
}

void maze_portal(){
  analogWrite(enb,70);
  analogWrite(ena,80);
  delay(240);
  forward();
  h = 7;
  stops();
  delay(50);
  analogWrite(enb,70);
  analogWrite(ena,80);
  left();
  delay(300);
  stops();
  delay(50);
  analogWrite(enb,70);
  analogWrite(ena,80);
  forward();
  delay(150);
  int c = 0;
  while(c < 90){
    ultraPID();
    c++;
  }
  forward();
}


void maze_first_part(){
 distanceRight = getDist(trigPinRight,echoPinRight);
 distanceLeft = getDist(trigPinLeft,echoPinLeft);
 if(distanceRight > 370 && distanceRight < 600 && distanceLeft>80){
  h = 8;
  analogWrite(enb,150);
  analogWrite(ena,200);
  forward();
  delay(160);
  stops();
  delay(50);
  analogWrite(enb,100);
  analogWrite(ena,150);
  right();
  delay(300);
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
 }else{
  int errorultra = distanceLeft - distanceRight;
  ms = kpu*errorultra + kdu*(errorultra - lasterrorultra);
  lms = bms + ms;
  rms = bms - ms;
  lasterrorultra = errorultra;
  if(lms > maxlms){
    lms = maxlms;
  }if(lms < 0){
    lms = 0;
  }if(rms < 0){
    rms = 0;
  }if(rms > maxrms){
    rms = maxrms;
  }
  analogWrite(enb,0.5*lms);
  analogWrite(ena,0.4*rms);
  forward();
 }
}

void maze_second_portal(){

  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
  delay(380);
  h = 9;
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,100);
  left();
  delay(280);
  stops();
  delay(50);
  analogWrite(enb,200);
  analogWrite(ena,200);
  forward();
  delay(250);
  stops();
  delay(500000);
  forward();
}

void maze_second_part(){
 distanceRight = getDist(trigPinRight,echoPinRight);
 distanceLeft = getDist(trigPinLeft,echoPinLeft);
 if(distanceLeft > 350 && distanceRight < 600){
  h = 10;
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
  delay(160);
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,100);
  left();
  delay(300);
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,120);
  forward();
 }else{
  int errorultra = distanceLeft - distanceRight;
  ms = kpu*errorultra + kdu*(errorultra - lasterrorultra);
  lms = bms + ms;
  rms = bms - ms;
  lasterrorultra = errorultra;
  if(lms > maxlms){
    lms = maxlms;
  }if(lms < 0){
    lms = 0;
  }if(rms < 0){
    rms = 0;
  }if(rms > maxrms){
    rms = maxrms;
  }
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}

void maze_third_portal(){
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
  delay(380);
  h = 11;
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,100);
  left();
  delay(280);
  stops();
  delay(50);
  analogWrite(enb,200);
  analogWrite(ena,200);
  forward();
  delay(280);
  forward();
}

void maze_last_part(){
 distanceRight = getDist(trigPinRight,echoPinRight);
 distanceLeft = getDist(trigPinLeft,echoPinLeft);
 if(distanceRight > 350 && distanceRight < 600){
  h = 12;
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
  delay(160);
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,100);
  right();
  delay(280);
  stops();
  delay(50);
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
 }else{
  int errorultra = distanceLeft - distanceRight;
  ms = kpu*errorultra + kdu*(errorultra - lasterrorultra);
  lms = bms + ms;
  rms = bms - ms;
  lasterrorultra = errorultra;
  if(lms > maxlms){
    lms = maxlms;
  }if(lms < 0){
    lms = 0;
  }if(rms < 0){
    rms = 0;
  }if(rms > maxrms){
    rms = maxrms;
  }
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}

void maze_out(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);
 if(capt1||capt2||capt3||capt4||capt5||capt6||capt7||capt8||capt9||capt10){
  h = 13;
  int c = 0;
  while(c < 2000){
   int capt1 = !digitalRead(c1);
   int capt2 = !digitalRead(c2);
   int capt3 = !digitalRead(c3);
   int capt4 = !digitalRead(c4);
   int capt5 = !digitalRead(c5);
   int capt6 = !digitalRead(c6);
   int capt7 = !digitalRead(c7);
   int capt8 = !digitalRead(c8);
   int capt9 = !digitalRead(c9);
   int capt10 = !digitalRead(c10);
  
   
   int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;
  
   ms = kp*error +kd*(error-lasterror);
   rms = bms - ms;
   lms = bms + ms;
   lasterror = error;
   if(lms > maxlms){
    lms = maxlms;
   }if(lms < 0){
    lms = 0;
   }if(rms < 0){
    rms = 0;
   }if(rms > maxrms){
    rms = maxrms;
   }
   if(capt1||capt2){
    analogWrite(enb,150);
    analogWrite(ena,150);
    lefts();
   }else if(capt10||capt9){
    analogWrite(enb,150);
    analogWrite(ena,150);
    rights();
   }else if(capt4||capt5||capt6||capt7||capt3||capt8){
    analogWrite(enb,bc*lms);
    analogWrite(ena,ac*rms);
    forward();
   }
  }
 }else{
  analogWrite(enb,120);
  analogWrite(ena,150);
  forward();
 }
}

void B_PID(){
 int capt1 = !digitalRead(c1);
 int capt2 = !digitalRead(c2);
 int capt3 = !digitalRead(c3);
 int capt4 = !digitalRead(c4);
 int capt5 = !digitalRead(c5);
 int capt6 = !digitalRead(c6);
 int capt7 = !digitalRead(c7);
 int capt8 = !digitalRead(c8);
 int capt9 = !digitalRead(c9);
 int capt10 = !digitalRead(c10);

 if((capt1||capt10)&&(!capt4||!capt5||!capt6||!capt7)){
  h = 14;
 }else{
  int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;
  
  ms = kp*error +kd*(error-lasterror);
  rms = bms - ms;
  lms = bms + ms;
  lasterror = error;
  if(lms > maxlms){
   lms = maxlms;
  }if(lms < 0){
   lms = 0;
  }if(rms < 0){
   rms = 0;
  }if(rms > maxrms){
   rms = maxrms;
  }
  if(capt1||capt2){
   analogWrite(enb,150);
   analogWrite(ena,150);
   lefts();
  }else if(capt10||capt9){
   analogWrite(enb,150);
   analogWrite(ena,150);
   rights();
  }else if(capt4||capt5||capt6||capt7||capt3||capt8){
   analogWrite(enb,bc*lms);
   analogWrite(ena,ac*rms);
   forward();
  }
 }
}

void sharp_right(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt10||capt9){
  h=15;
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
  delay(100);
  stops();
  delay(50);
  analogWrite(enb,70);
  analogWrite(ena,100);
  right();
  delay(400);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt4||capt5||capt6||capt7||capt3||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}


void sharp_left(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt1||capt2){
  h=16;
  analogWrite(enb,150);
  analogWrite(ena,150);
  forward();
  delay(100);
  stops();
  delay(50);
  analogWrite(enb,70);
  analogWrite(ena,100);
  left();
  delay(400);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = 1.5*error +3*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
 analogWrite(enb,bc*lms);
 analogWrite(ena,ac*rms);
 forward();
}

void last_right(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 if(capt10&&capt9&&capt8){
  h=17;
  stops();
  delay(50);
  analogWrite(enb,70);
  analogWrite(ena,100);
  right();
  delay(400);
  forward();
 }
 
 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = kp*error +kd*(error-lasterror);
 rms = bms - ms;
 lms = bms + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt3||capt4||capt5||capt6||capt7||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}

void ba3ba3(){
 int capt1 = digitalRead(c1);
 int capt2 = digitalRead(c2);
 int capt3 = digitalRead(c3);
 int capt4 = digitalRead(c4);
 int capt5 = digitalRead(c5);
 int capt6 = digitalRead(c6);
 int capt7 = digitalRead(c7);
 int capt8 = digitalRead(c8);
 int capt9 = digitalRead(c9);
 int capt10 = digitalRead(c10);

 int error= 80*capt1 + 60*capt2 + 50*capt3 + 30*capt4 + 10*capt5 - 10*capt6 - 30*capt7 - 50*capt8  - 60*capt9 - 80*capt10;

 ms = 2*error +4*(error-lasterror);
 rms = 255 - ms;
 lms = 255 + ms;
 lasterror = error;
 if(lms > maxlms){
  lms = maxlms;
 }if(lms < 0){
  lms = 0;
 }if(rms < 0){
  rms = 0;
 }if(rms > maxrms){
  rms = maxrms;
 }
  
 if(capt1||capt2){
  analogWrite(enb,150);
  analogWrite(ena,150);
  lefts();
 }else if(capt10||capt9){
  analogWrite(enb,150);
  analogWrite(ena,150);
  rights();
 }else if(capt3||capt4||capt5||capt6||capt7||capt8){
  analogWrite(enb,bc*lms);
  analogWrite(ena,ac*rms);
  forward();
 }
}

void forward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}
void wa55er(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, HIGH);
}
void left(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, LOW);
}

void right(){
  digitalWrite(in1, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, HIGH);
}
void lefts(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}

void rights(){
  digitalWrite(in1, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}
void stops(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}
int getDist(int trigPin, int echoPin){
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.34 / 2; 
}
