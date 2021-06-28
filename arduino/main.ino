#include <stdio.h>
#include "SimpleTimer.h"
#include <Arduino.h>
#include <Wire.h>                        // Include Wire library (required for I2C devices)
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <MHZ.h>







//------------------------------------------------
//--- DEBUG  FLAGS     ---------------------------
//------------------------------------------------
// Check makefile

//------------------------------------------------
//--- CONSTANTS        ---------------------------
//------------------------------------------------

#define TIMEOUT_HEATING   10000

// pin for pwm reading
#define CO2_IN 10

// pin for uart reading
#define MH_Z19_RX 8  
#define MH_Z19_TX 7  

//------------------------------------------------
//--- GLOBAL VARIABLES ---------------------------
//------------------------------------------------



//U8G2_SSD1306_128X64_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  

MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ19B);

//unsigned long time;
SimpleTimer mainTimers;

int  GLB_timerMisc=-1;
bool GLB_timerMisc_expired=false;



int GLBsensorCO2ppm_uart = 0;
int GLBsensorCO2temperature = 0;

//------------------------------------------------
//--- GLOBAL FUNCTIONS ---------------------------
//------------------------------------------------

void display_welcome();
 
void STATE_welcome();
void STATE_heating();
void STATE_CO2();

void goto_heating();

// State pointer function
void (*GLBptrStateFunc)();

void GLBcallbackLoggingCO2(void);
//------------------------------------------------
//-------    TIMER CALLBACKS     -----------------
//------------------------------------------------



//------------------------------------------------

void GLBcallbackLogging(void)
{
  Serial.println(F("DEBUG: still alive from timer logging..."));
}



//------------------------------------------------
void GLBcallbackTimeoutMisc(void)
{
  if (GLB_timerMisc != -1)     
    mainTimers.deleteTimer(GLB_timerMisc);
  GLB_timerMisc=-1;
  GLB_timerMisc_expired=true;
}


//------------------------------------------------
void resetTimerMisc(void)
{
  GLBcallbackTimeoutMisc();
  GLB_timerMisc_expired=false;
}

  
//------------------------------------------------
//------------------------------------------------
//------------------------------------------------

//-------------------------------------------------------
void display_CO2(bool heating,int rem){
  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_logisoso16_tr );
      u8g2.setCursor(0,17);
      u8g2.print("T:");
      u8g2.print(GLBsensorCO2temperature);
      if (heating){
        u8g2.print(" H:");
        u8g2.print(rem);}
      else
        u8g2.print(" CO2:");
      u8g2.setFont(u8g2_font_fub42_tr );
      u8g2.setCursor(0,64);
      u8g2.print(GLBsensorCO2ppm_uart);
  
  } while ( u8g2.nextPage() );
}
//-------------------------------------------------------
void setup() { 
  // Serial to debug AND comunication protocolo with PI              
  Serial.begin(9600);
  Serial.println(F("BOOT"));

  Serial.println(F("B u8g2"));
  u8g2.begin(); 
  u8g2.setFontMode(0);		// enable transparent mode, which is faster

  resetTimerMisc();
  GLBptrStateFunc=STATE_welcome; 

  Serial.println(F("BD"));

#ifdef SPCO2_DEBUG_STILLALIVE
  mainTimers.setInterval(2000,GLBcallbackLogging);
#endif 

#ifdef SPCO2_DEBUG_CO2
  mainTimers.setInterval(5000,GLBcallbackLoggingCO2);
#endif  

  Serial.println(F("B MHZ 19B"));
  pinMode(CO2_IN, INPUT);
  delay(100);

  // enable debug to get addition information
  //co2.setDebug(true);


}

//-------------------------------------------------
void goto_heating()
{
  
  u8g2.clearBuffer();
  #ifdef SPCO2_DEBUG_STATES
  Serial.print(F("NST_heating"));
  #endif
  resetTimerMisc();
  GLBptrStateFunc=STATE_heating; 
}


//-------------------------------------------------
void STATE_welcome()
{
  display_welcome();
  goto_heating();  
}

//-------------------------------------------------
void STATE_heating()
{
  display_CO2(co2.isPreHeatingReal(),co2.remainingPreHeating());

  if (!co2.isPreHeatingReal()){
    u8g2.clearBuffer();
    #ifdef SPCO2_DEBUG_STATES
    Serial.print(F("NST_CO2"));
    #endif
    GLBptrStateFunc=STATE_CO2; 
  }

}

//-------------------------------------------------
void STATE_CO2()
{
  display_CO2(false,0);
}



//-----------------------------------------------
void displayBig(char *msg)
{
  u8g2.firstPage();
  do { 
    u8g2.setFont(u8g2_font_fub42_tr);	
    u8g2.setCursor(10,64);
    u8g2.print(msg);
    
  } while ( u8g2.nextPage() );
}

void display_welcome(){

  displayBig("CO2");
  delay(50);
  displayBig("- ");
  delay(50);
  displayBig("--");
  delay(50);
  displayBig("---");
  delay(50);
  displayBig("----");
  delay(50);  displayBig("CO2");
  delay(200);
}


//------------------------------------------------
#ifdef SPCO2_DEBUG_CO2
void GLBcallbackLoggingCO2(void)
{
  Serial.print("PPMuart: ");
  if (GLBsensorCO2ppm_uart > 0) {
    Serial.print(GLBsensorCO2ppm_uart);
  } else {
    Serial.print("n/a");
  }

  /*Serial.print(", PPMpwm: ");
  Serial.print(GLBsensorCO2ppm_pwm);*/

  Serial.print(", Temperature: ");
  Serial.println(GLBsensorCO2temperature);

}
#endif

//-------------------------------------------------
//-------------------------------------------------
//-------------------------------------------------

void loop() { 
#ifdef SPCO2_DEBUG_LOOP  
  Serial.print("+");
#endif
  //------------- INPUT REFRESHING ----------------
  // Let use ugly global variables in those costly sensors to save ram...

#ifdef SPCO2_DEBUG_LOOP  
  Serial.print("D");
#endif

#ifdef SPCO2_DEBUG_LOOP  
  Serial.print("C");
#endif
/*
  if (co2.isPreHeating()) {
    GLBsensorCO2ppm_uart = 0;
    GLBsensorCO2temperature = 0;
  }
  else {
    GLBsensorCO2ppm_uart = co2.readCO2UART();
    GLBsensorCO2temperature = co2.getLastTemperature();
  }
*/
  GLBsensorCO2ppm_uart = co2.readCO2UART();
  GLBsensorCO2temperature = co2.getLastTemperature();


  //  bRedPressed   = (digitalRead(PIN_BUTTON_RED)  ==LOW);

  //--------- TIME TO THINK MY FRIEND -------------
  // State machine as main controller execution
#ifdef SPCO2_DEBUG_LOOP  
  Serial.print("S");
#endif
  GLBptrStateFunc();
#ifdef SPCO2_DEBUG_LOOP  
  Serial.print("T");
#endif
  mainTimers.run();

//  if (bRedPressed)    
  // ------------- OUTPUT REFRESHING ---------------

}

