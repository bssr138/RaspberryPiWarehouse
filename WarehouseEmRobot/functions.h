
/*
-----------------------Hardware Functions-----------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


void lineSensors(long *readings); // sensor readings function
void lineFollow(float Kp, int objectInFront); // line follow function
int checkForIntersect(); // checkForIntersection function
void drive(float LEFTPower, float RIGHTPower); //drive-bot function
void stopMotors(); //stop-bot function
void turnLeft();
void turnRight();
void turn180();
void BotForward();

