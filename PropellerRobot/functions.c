#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Include servo header
//#include "sensors.h"                          // sensors header for Ultrasonics
#include "ultrasonicSensors.h"                 // sensors header for Ultrasonics


//=============================================================================================
// PIN CONNECTIONS
//=============================================================================================
#define LEFT  4
#define RIGHT  6
#define CENTER 5
#define LEFT_SERVO 17
#define RIGHT_SERVO 16
#define LEFTMOST 7
#define RIGHTMOST 8
#define TURNRIGHT 0
#define leftpin 13
#define centerpin 11
#define rightpin 12

//=============================================================================================
// OPERATING VARIABLES
//=============================================================================================
#define MOTOR_KP 10.0
volatile int indicateFlag;

//volatile int turnRightflag;

/*
------------------------Hardware Functions----------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


//=============================================================================================
// float lineSensors(long *readings)
// ----- Function that reads the values from each of the IR sensors and saves them to a 
// ----- long array. This array is passed as a pointer.
//=============================================================================================
void lineSensors(long *readings){
    
    set_direction(CENTER, 1);
    set_output(CENTER, 1);
    pause(1);
    set_direction(CENTER,0);
    readings[1] = rc_time(CENTER,1);
    //print("centerSen = %d\n", readings[1]);
    
    
    set_direction(RIGHT, 1);
    set_output(RIGHT, 1);
    pause(1);
    set_direction(RIGHT,0);
    readings[2] = rc_time(RIGHT,1);
    //print("rightSen = %d\n", readings[2]);


    set_direction(LEFT, 1);
    set_output(LEFT, 1);
    pause(1);
    set_direction(LEFT,0);
    readings[0] = rc_time(LEFT,1);
   // print("leftSen = %d\n", readings[0]);
    pause(5); 

}


//=============================================================================================
// void drive(float LEFTPower, float RIGHTPower)
// ----- Function that moves the continuous servos depending on the power values supplied. 
// ----- 1500 represenents stop, however the values close to 1500 are too slow therefore
// ----- the power values begin incrementing with a starting value of 1500+-25. 
//=============================================================================================
void drive(float LEFTPower, float RIGHTPower){

  //1700 (1600)is maximum in one direction and 1300 (1400) is maximum in other direction
  //1500 is stop

  servo_set(LEFT_SERVO,(1525+(LEFTPower*2.5)));   //LEFT
  servo_set(RIGHT_SERVO,(1475-(RIGHTPower*2.5)));  // RIGHT

}


//=============================================================================================
// void checkFrontObj()
// ----- Function that checks if there is an object in right of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkRightObj(){
  
  if(getUSReadingRight()<6){
    return 1;
  }  else{
    return 0;
  }      
  
} 

//=============================================================================================
// void checkFLeftObj()
// ----- Function that checks if there is an object in left of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkLeftObj(){
  
  if(getUSReadingLeft()< 6){
    return 1;
  }  else{
    return 0;
  }      
  
} 

//=============================================================================================
// void stopMotors()
// ----- Function that stops the continuous servos. 1500 represenents stop in the servo_set
// ----- function
//=============================================================================================
void stopMotors() {
  servo_set(LEFT_SERVO,1500);
  servo_set(RIGHT_SERVO,1500);
}


//=============================================================================================
// void checkForIntersect()
// ----- Function that uses the farthest LEFT and farthest RIGHT sensor on the sensor array
// ----- to check if an intersection has been reached
//=============================================================================================
int checkForIntersect(){
    long leftInt, rightInt;

    set_direction(LEFTMOST, 1);
    set_output(LEFTMOST, 1);
    pause(1);
    set_direction(LEFTMOST,0);
    leftInt = rc_time(LEFTMOST,1);

    set_direction(RIGHTMOST, 1);
    set_output(RIGHTMOST, 1);
    pause(1);
    set_direction(RIGHTMOST,0);
    rightInt = rc_time(RIGHTMOST,1);
    pause(5);

    if(leftInt > 2500 && rightInt > 2500){
      return 1;
    }else{
      return 0;
    }
}
//=============================================================================================
// void checkForIntersect()
// ----- Function that uses the farthest LEFT and farthest RIGHT sensor on the sensor array
// ----- to check if an intersection has been reached
//=============================================================================================
int checkForIntersectBT(){
    long leftInt, rightInt;

    set_direction(LEFTMOST, 1);
    set_output(LEFTMOST, 1);
    pause(1);
    set_direction(LEFTMOST,0);
    leftInt = rc_time(LEFTMOST,1);

    set_direction(RIGHTMOST, 1);
    set_output(RIGHTMOST, 1);
    pause(1);
    set_direction(RIGHTMOST,0);
    rightInt = rc_time(RIGHTMOST,1);
    pause(5);

    if(leftInt > 2500 || rightInt > 2500){
      return 1;
    }else{
      return 0;
    }
}

void lineFollowEnd(float Kp){

  //Variables local to this function
  long leftSen;
  long midSen;
  long rightSen;
  long sensorValues[3];
  float stndMtrPwr = 10.0;
  float PosError = 0;
  float one;
  float senPos = 2.50;
  int breakCondition;

  breakCondition = checkForIntersect();

  //---------------------------Line Follwoing Loop----------------------------------

    while(!breakCondition){
    //-----------------------Reading Sensors Values---------------------------------
      lineSensors(sensorValues);
      leftSen = sensorValues[0];
      midSen = sensorValues[1];
      rightSen = sensorValues[2];

      drive(stndMtrPwr+Kp*PosError,stndMtrPwr-Kp*PosError);

      
    //-----------------------Computing Error --------------------------------

      if(midSen > 2500){
        senPos = 2.50;
      }else if (rightSen > leftSen){
        one = rightSen/2500.0;
        senPos = senPos + 2.5*one;
 
      } else if(rightSen < leftSen){
        one = leftSen/2500.0;
        senPos = senPos - 2.5*one;
        
      }
      
      PosError =  senPos - 2.50;
      senPos = 2.5;

    //-----------------------Break Conditions--------------------------------

    breakCondition = ((midSen < 1000) && (leftSen < 1000)&& (rightSen < 1000));
    }

  //Once loop ends, an intersection has been detected. Therefore stop motors
  stopMotors();
}

//=============================================================================================
// void lineFollow(float Kp, int ultraSonicCheck)
// ----- This function controls the robot to follow a line until an intersection is detected.
//=============================================================================================
void lineFollow(float Kp, int ultraSonicCheck, int splIntersectionCheck){

  //Variables local to this function
  long leftSen;
  long midSen;
  long rightSen;
  long sensorValues[3];
  float stndMtrPwr = 10.0;
  float PosError = 0;
  float one;
  float senPos = 2.50;
  int breakCondition;

  //Checking the break condition once before the loop begins
  // if(objectInRight){  
  
  //   breakCondition = checkForIntersect() || checkRightObj();

  // }else{
  //   breakCondition = checkForIntersect();
  // }
  if(splIntersectionCheck){
    breakCondition = 0;
  }else{
    breakCondition = checkForIntersect();
  }

  //---------------------------Line Follwoing Loop----------------------------------

    do{
      //Reading sensor values and applying the power values to the motor

    //-----------------------Reading Sensors Values---------------------------------
      lineSensors(sensorValues);
      leftSen = sensorValues[0];
      midSen = sensorValues[1];
      rightSen = sensorValues[2];

      drive(stndMtrPwr+Kp*PosError,stndMtrPwr-Kp*PosError);

      
    //-----------------------Computing Error --------------------------------

      if(midSen > 2500){
        //If the middle sensor is on the line then the robot needs to just continue straight
        senPos = 2.50;
        //2.5

      }else if (rightSen > leftSen){
        one = rightSen/2500.0;
        senPos = senPos + 2.5*one;
        
        
      } else if(rightSen < leftSen){
        one = leftSen/2500.0;
        senPos = senPos - 2.5*one;
        
      }
      
      PosError =  senPos - 2.50;
      senPos = 2.5;

    //-----------------------Break Conditions--------------------------------

    if(splIntersectionCheck){
      if(ultraSonicCheck){  
        breakCondition = checkForIntersectBT() || checkRightObj() || checkLeftObj();
      }else{
        // drive(stndMtrPwr+Kp*PosError,stndMtrPwr-Kp*PosError);
        // pause(100);
        breakCondition = checkForIntersectBT();
      }  
    }
    else{
      if(ultraSonicCheck){  
        breakCondition = checkForIntersect() || checkRightObj() || checkLeftObj();
      }else{
        breakCondition = checkForIntersect();
      }           
    }
    }while(!breakCondition);

  //Once loop ends, an intersection has been detected. Therefore stop motors
  stopMotors();
}

//=============================================================================================
// void turnLeft()
// ----- This function turns the robot left until the middle line sensor detects black
//=============================================================================================
void turnLeft(){
  long readings[3];

  servo_set(LEFT_SERVO,(1450));   //LEFT
  servo_set(RIGHT_SERVO,(1450));  // RIGHT
  pause(500);

  do{
    servo_set(LEFT_SERVO,(1450));   //LEFT
    servo_set(RIGHT_SERVO,(1450));  // RIGHT
    
    lineSensors(readings);
  }while(readings[1]<2200);



  stopMotors();


}

//=============================================================================================
// void turnRight()
// ----- This function turns the robot right until the middle line sensor detects black
//=============================================================================================
void turnRight(){
  long readings[3];

  servo_set(LEFT_SERVO,(1550));   //LEFT
  servo_set(RIGHT_SERVO,(1550));  // RIGHT
  pause(500);
  
  do{
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    
    lineSensors(readings);
  }while(readings[1]<2200);


  stopMotors();
}

//=============================================================================================
// void turn90()
// ----- This function turns the robot 90° right
//=============================================================================================
void turn90Right(){

  servo_set(LEFT_SERVO,(1550));   //LEFT
  servo_set(RIGHT_SERVO,(1550));  // RIGHT
  pause(1000);
  stopMotors();

}

//=============================================================================================
// void turn90()
// ----- This function turns the robot 90° left
//=============================================================================================
void turn90Left(){

  servo_set(LEFT_SERVO,(1450));   //LEFT
  servo_set(RIGHT_SERVO,(1450));  // RIGHT
  pause(1000);
  stopMotors();

}


//=============================================================================================
// void turn180()
// ----- This function turns the robot right until the right line sensor detects black
//=============================================================================================
void turn180(){
  long readings[3];

  servo_set(LEFT_SERVO,(1550));   //LEFT
  servo_set(RIGHT_SERVO,(1550));  // RIGHT
  pause(500);
  
  do{
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    
    lineSensors(readings);
  } while(readings[2]<2200);


  stopMotors();

}
//=============================================================================================
// void BotForward()
// ----- This function moves the bot forward so the wheels are over the line
//=============================================================================================
void BotForward(){
  drive(20,20);
  pause(300);
  //pause(300);
}

void BotForwardLIL(){
  drive(20,20);
  pause(100);
  //pause(300);
}

void tagFollowing(){

  long leftSen;
  long midSen;
  long rightSen;
  long sensorValues[3];

  do{
      lineSensors(sensorValues);

      leftSen = sensorValues[0];
      midSen = sensorValues[1];
      rightSen = sensorValues[2];

      if (input(leftpin)){
        tagLeft();
      }

      else if (input(rightpin)){
        tagRight(); 
      }

      else if (input(centerpin)){
        tagForward(); 
      }

      else{
        stopMotors();
      }
    }while(leftSen<2200 && midSen<2200 && rightSen<2200);
  
  stopMotors();


}
 
void tagForward() {

  print("moving forward\n");
  servo_set(LEFT_SERVO ,1550);
  servo_set(RIGHT_SERVO,1450);
  pause(100);
  stopMotors();

}
 
void tagRight() {

  print("turning right\n");
  servo_set(LEFT_SERVO ,1550);
  servo_set(RIGHT_SERVO,1550);
  pause(100); 
  stopMotors();
  //high(27);
  //pause(100);
  //low(27);

}

void tagLeft() {

  print("turning left\n");
  servo_set(LEFT_SERVO,1450);
  servo_set(RIGHT_SERVO,1450);
  pause(100);
  stopMotors();

}


//! /* ------------------------Main Functions------------------------- */

//=============================================================================================
// void getToStart()
// ----- This function drives the robot until i1. Following the line the whole time
//=============================================================================================
void getToStart(){
  
  lineFollow(MOTOR_KP,0, 0);//located in functions.c
  pause(2000);
  //i1
    
} 

void gotoC(){

  int num_iterations = 3; // change this to the desired number of iterations
  int i;

  for(i = 0; i < num_iterations; i++) {
    checkStation();
    lineFollow(MOTOR_KP,0,0);
    pause(1500);
  }

  if(input(TURNRIGHT)){
    turn90Right();
  }
  else{
    turn90Left();
  }
  tagFollowing();
}

//=============================================================================================
// void checkStation()
//=============================================================================================
void checkStation(){

  if(input(TURNRIGHT)){
    BotForward();
    turnRight();
    
    checkRight();

    turnRight();
    lineFollow(MOTOR_KP,0,0);

    BotForward();
    turnRight();
    lineFollow(MOTOR_KP,0,1);
    //i1

    checkLeft();

    turnRight();
    lineFollow(MOTOR_KP,0,0);
    BotForward();
    //BotForward();
    turnRight();
    lineFollow(MOTOR_KP,0,1);
    //i2
    BotForward();
    turnLeft();
    //BotForward();

  }
  else{
    BotForward();
    turnLeft();
    
    checkLeft();

    turnLeft();
    lineFollow(MOTOR_KP,0,0);
    BotForward();
    turnLeft();
    lineFollow(MOTOR_KP,0,1);
    //i1
    //BotForward();

    checkRight();

    turnLeft();
    lineFollow(MOTOR_KP,0,0);
    BotForward();
    turn90Left();
    lineFollow(MOTOR_KP,0,1);
    //i2
    BotForward();
    //BotForward();
    turnRight();
    //BotForward();
  }
    
}


void checkRight(){
  lineFollow(MOTOR_KP,0,0);
  BotForward();
  turnLeft();
  lineFollow(MOTOR_KP,1,0);
  BotForward();
  indicateFlag = 1;
  turn90Right();
  pause(2000);
  indicateFlag = 0;
  
}

void checkLeft(){

  //BotForward();
  lineFollow(MOTOR_KP,0,0);
  BotForward();
  turnRight();
  lineFollow(MOTOR_KP,1,0);
  BotForward();
  indicateFlag = 1;
  turn90Left();
  pause(2000);
  indicateFlag = 0;

  
}