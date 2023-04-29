#include "ping.h"                             // Incluide ultraSonic header

#define ULTRA_ONE_PIN 9
#define ULTRA_TWO_PIN 10

static volatile int usReadingRight;                // for distance measurement
static volatile int usReadingLeft; 


void ultraSonic() {                         
  while(1){
    usReadingRight = ping_cm(ULTRA_ONE_PIN);                  
    pause(10);
    usReadingLeft = ping_cm(ULTRA_TWO_PIN);                  
    pause(10);
  }
 }

int getUSReadingRight(){
  
  return usReadingRight;

}  

int getUSReadingLeft(){
  
  return usReadingLeft;

} 