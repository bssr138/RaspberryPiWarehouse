#include "fdserial.h"

//LIBRARIES
#include "simpletools.h"                  

//SELF MADE FILES
#include "functions.h"                        
#include "ultrasonicSensors.h"                          
//#include "outputs.h"

#define NONDEF_LED 14
#define DEF_LED 15
#define DEF_SIGNAL 3
#define NONDEF_SIGNAL 2
#define TURNRIGHT 0

unsigned int stack[128];
unsigned int stack2[128];
unsigned int stack3[65];
unsigned int stack4[128+64];
unsigned int stack5[128];
volatile int indicateFlag;

fdserial *term;

void serial_tx_rx(void *par);

void serial_tx_rx(void *par)
{
  while(1)
  {
    dprint(term, "L");
    pause(1000); // Wait for 1 second
  }
}


void raspiGPIOs(){
    
  while(1){
    // ArUco tag LED Signal
    if(input(DEF_SIGNAL) && indicateFlag == 1){
      high(DEF_LED);
    } else if(input(NONDEF_SIGNAL)&& indicateFlag == 1){ 
      high(NONDEF_LED);
      // pause(500); 
    }else{
       low(NONDEF_LED);
       low(DEF_LED);
    }
  
  }  
}

int main(){

  // Cog 1
  //------------------------------------------------------------------------
  cogstart(&ultraSonic, NULL, stack, sizeof(stack)); //Located in sensors.c
  
  // Cog 2
  //------------------------------------------------------------------------
  //term = fdserial_open(31, 30, 0, 115200); // RX, TX, mode, baud
  //cogstart(&serial_tx_rx, NULL, stack4, sizeof(stack4));

  // Cog 5
  //------------------------------------------------------------------------
  cogstart(&raspiGPIOs, NULL, stack5, sizeof(stack5));

  // Cog 0 
  //------------------------------------------------------------------------

  getToStart();
  gotoC();
  // i1;
} 