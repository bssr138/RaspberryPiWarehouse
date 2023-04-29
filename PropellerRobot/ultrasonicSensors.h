/*
With sensors.h any cog can access the current reading of the US sensor at any time 
without having to have access to the USreading variable
*/

void ultraSonic(); //measure-distance function
//Getter for reading the distance from other cogs
int getUSReadingLeft();
int getUSReadingRight();