#include <Ultrasonic.h> //INCLUDE LIBRARY TO USE ULTRASONIC SENSOR

Ultrasonic ultrasonic(9,8); // CREATE AN OBJECT FROM SENSOR (Trig PIN,Echo PIN)
//DECLARE VARS FOR ULTRASONIC SENSOR
int distancia;

//DECLARE VARS FOR BLUETOOTH COMUNICATION
int estat='c';
int estat_anterior;

/* 

POSSIBLE STATES:

  - a --> GO AHEAD
  - b --> TURN LEFT
  - c --> STOP
  - d --> TURN RIGHT
  - e --> GO BACK
  - f --> HORN
  - g --> OFF


*/

//RIGHT CONTROL MOTOR
int L298N_IN1=7;
int L298N_IN2=6;

//LEFT CONTROL MOTOR
int L298N_IN3=5;
int L298N_IN4=4;

//ENABLE PINS MOTOR
int EnableB=3;
int EnableA=2;

//VAR FOR VELOCITY OF MOTORS
int vel=200;

//PINS LEDS AND LDR
int pinLDR=A0;
int valorLDR=0; //VAR TO READ VALUE FROM LDR
int pinVermell_un=A1;
int pinVermell_dos=A2;
int pinGroc_un=A3;
int pinGroc_dos=A4;

//PIN SPEAKER
int SpeakerPin=10;
int tones=311; //VAR WITH FREQUENCY TONE
bool canviar=false;


void setup() {
  // put your setup code here, to run once:

  //INITIALIZE SERIAL PORT TO COMUNICATE BY BLUETOOTH
  Serial.begin(9600);

  //DECLARE MOTORS IN OUTPUT MODE
  pinMode(L298N_IN1,OUTPUT);
  pinMode(L298N_IN2,OUTPUT);
  pinMode(L298N_IN3,OUTPUT);
  pinMode(L298N_IN4,OUTPUT);

  //ENABLE MOTORS IN OUTPUT MODE
  pinMode(EnableB,OUTPUT);
  pinMode(EnableA,OUTPUT);

  //ASSIGN VELOCITY TO ENABLE MOTOR'S PINS
  analogWrite(EnableA,vel);
  analogWrite(EnableB,vel);

  //DECLARE LEDS AND SPEAKERS IN OUTPUT MODE
  pinMode(pinVermell_un,OUTPUT);
  pinMode(pinVermell_dos,OUTPUT);
  pinMode(pinGroc_un,OUTPUT);
  pinMode(pinGroc_dos,OUTPUT);
  pinMode(SpeakerPin,OUTPUT);

  //TURN OFF ALL OF LEDS
  digitalWrite(pinVermell_un,LOW);
  digitalWrite(pinVermell_dos,LOW);
  digitalWrite(pinGroc_un,LOW);
  digitalWrite(pinGroc_dos,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  /*WE SAVE PREVIOUS STATE
  THIS VAR WE ONLY USED FOR GO BACK
  WHEN WE PLAY A SOUND, BECAUSE IF 
  WE DON'T CHANGE THE STATE
  FOR PREVIOUS STATE, THE SOUND IS 
  PLAYING UNTIL WE SEND ANOTHER STATE*/
  estat_anterior=estat;

  //CHECK LIGHTS AND OBSTACLES
  lights();
  obstacles();

  //IF WE HAVE SIGNAL WE READ WHAT THE DEVICE CONNECTED SENDS
  if(Serial.available()>0){
      estat = Serial.read();
  }

  //IF STATE IS THE STATE TO PLAY A SOUND
  //WE PUT THE BOOL ENABLE TO CHANGE 
  //STATE FOR PREVIOUS STATE
  if(estat=='f'){
    canviar=true;
  }
  
  //GO AHEAD
  if(estat=='a'){
    startCar();
    lights();
    obstacles();
  }
  //TURN LEFT
  if(estat=='b'){
    leftTurn();
    lights();
    obstacles();
  }
  //STOP
  if(estat=='c'){
    stopCar();
    lights();
    obstacles();
  }
  //TURN RIGHT
  if(estat=='d'){
    rightTurn();
    lights();
    obstacles();
  }
  //GO BACK
  if(estat=='e'){
    backCar();
    lights();
    obstacles();
  }
  //HORN
  if(estat=='f'){
    if(canviar){
      horn();
      horn();
      estat=estat_anterior;
      canviar=false;
    }
    lights();
    obstacles();

    //AFTER PLAYING A SOUND WE NEED TO INITIALIZE ALL PINS WITH THE CORRECT STATE (IN / OUT)
    
    //DECLARE MOTORS IN OUTPUT MODE
    pinMode(L298N_IN1,OUTPUT);
    pinMode(L298N_IN2,OUTPUT);
    pinMode(L298N_IN3,OUTPUT);
    pinMode(L298N_IN4,OUTPUT);
  
    //ENABLE MOTORS IN OUTPUT MODE
    pinMode(EnableB,OUTPUT);
    pinMode(EnableA,OUTPUT);
  
    //ASSIGN VELOCITY TO ENABLE MOTOR'S PINS
    analogWrite(EnableA,vel);
    analogWrite(EnableB,vel);
  }
  //OFF --> WHEN WE PUT THE APP IN MODE OFF WE STOP THE CAR
  if(estat=='g'){
    stopCar();
  }

}

//METHOD TO GO BACK
void backCar(){
  digitalWrite(L298N_IN1,HIGH);
  digitalWrite(L298N_IN2,LOW);
  digitalWrite(L298N_IN3,HIGH);
  digitalWrite(L298N_IN4,LOW);
}

//METHOD TO GO AHEAD
void startCar(){
  digitalWrite(L298N_IN1,LOW);
  digitalWrite(L298N_IN2,HIGH);
  digitalWrite(L298N_IN3,LOW);
  digitalWrite(L298N_IN4,HIGH);
}

//METHOD TO STOP THE CAR
void stopCar(){
  digitalWrite(L298N_IN1,LOW);
  digitalWrite(L298N_IN2,LOW);
  digitalWrite(L298N_IN3,LOW);
  digitalWrite(L298N_IN4,LOW);
}

//METHOD TO TRUN RIGHT
void rightTurn(){
  digitalWrite(L298N_IN1,LOW);
  digitalWrite(L298N_IN2,HIGH);
  digitalWrite(L298N_IN3,HIGH);
  digitalWrite(L298N_IN4,LOW);
}

//METHOD TO TRUN LEFT
void leftTurn(){
  digitalWrite(L298N_IN1,HIGH);
  digitalWrite(L298N_IN2,LOW);
  digitalWrite(L298N_IN3,LOW);
  digitalWrite(L298N_IN4,HIGH);
}

//METHOD TO CHECK IF WE NEED LIGHTS
void lights(){
    //READ A VALUE FROM LDR
    valorLDR= analogRead(pinLDR);

    //IF WE HAVE ENOGH LIGHT WE TURN OFF THE LIGHTS
    if(valorLDR > 500)
    {
      digitalWrite(pinVermell_un, LOW);
      digitalWrite(pinGroc_un, LOW);
      digitalWrite(pinVermell_dos, LOW);
      digitalWrite(pinGroc_dos, LOW);
    }
    //IF WE HAVEN'T ENOUGH LIGHT WE TURN ON THE LIGHTS
    else{
      digitalWrite(pinVermell_un, HIGH);
      digitalWrite(pinGroc_un, HIGH);
      digitalWrite(pinVermell_dos, HIGH);
      digitalWrite(pinGroc_dos, HIGH);
    }
}

//METHOD FOR CAR'S HORN
void horn(){
  //PLAY A SOUND DURING 0'3 SECONDS
  tone(SpeakerPin,tones);
  delay(300);
  
  //STOP THE SOUND
  noTone(SpeakerPin);
}

void obstacles(){
  //WE DEFINE IN SENSOR THAT WE USE CM TO DETECT OBSTACLES
  distancia=ultrasonic.Ranging(CM);
  
  //WHEN THE CAR WILL BE AT LEAST 30 cm 
  //WE STOP THE CAR FROM 0'5 SECONDS
  //AND GO BACK DURING 0'5 SECONDS, 
  //FINALLY WE STOP THE CAR AND PUT THE STATE IN STOP MODE
  if(distancia<30){
    stopCar();
    delay(500);
    backCar();
    delay(500);
    stopCar();
    estat='c';
  }
}

