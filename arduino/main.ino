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

#define PIN_SENSOR_TEMP           6


#define TIMEOUT_SHOW_HUM  1000
#define TIMEOUT_SHOW_TEMP 2000
#define TIMEOUT_SHOW_CO2  10000

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



float GLBsensorDHTTemp=0.0;
float GLBsensorDHTHum=0.0;

int GLBsensorCO2ppm_uart = 0;
int GLBsensorCO2temperature = 0;

//------------------------------------------------
//--- GLOBAL FUNCTIONS ---------------------------
//------------------------------------------------

void display_temperature();
bool display_welcome();
 

void STATE_welcome();
void STATE_idle();
void STATE_hum();
void STATE_CO2();

void STATE_button();
void goto_idle();
// State pointer function
void (*GLBptrStateFunc)();

DHT GLBsensorDHT(PIN_SENSOR_TEMP, DHT22);

void GLBcallbackLoggingTemp(void);
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
void display_temperature(){

  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_logisoso16_tr );
      u8g2.setCursor(0,17);
      u8g2.print("TEMPERATURA:");
      u8g2.setFont(u8g2_font_fub42_tr );
      u8g2.setCursor(0,64);
      u8g2.print(GLBsensorDHTTemp,1);
      /*u8g2.drawCircle(90, 5, 3, U8G2_DRAW_ALL);
      u8g2.setCursor(90,64);
      u8g2.print("C");*/
   
  } while ( u8g2.nextPage() );
}
//-------------------------------------------------------
void display_humidity(){

  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_logisoso16_tr );
      u8g2.setCursor(20,17);
      u8g2.print("HUMEDAD:");
      u8g2.setFont(u8g2_font_fub42_tr );
      u8g2.setCursor(0,64);
      u8g2.print(GLBsensorDHTHum,1);
      /*u8g2.drawCircle(90, 5, 3, U8G2_DRAW_ALL);
      u8g2.setCursor(90,64);
      u8g2.print("C");*/
   
  } while ( u8g2.nextPage() );
}

//-------------------------------------------------------
void display_CO2(){

  u8g2.firstPage();
  do {
      u8g2.setFont(u8g2_font_logisoso16_tr );
      u8g2.setCursor(0,17);
      u8g2.print("TEMP:");
      u8g2.print(GLBsensorCO2temperature);
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



  Serial.println(F("B DHT22"));
  GLBsensorDHT.begin();

//  pinMode(PIN_BUTTON_RED, INPUT_PULLUP);

  Serial.println(F("B u8g2"));
  u8g2.begin(); 
  u8g2.setFontMode(0);		// enable transparent mode, which is faster
  //u8g2.setDisplayRotation(U8G2_R2);

  resetTimerMisc();
  GLBptrStateFunc=STATE_welcome; 

  Serial.println(F("BD"));


#ifdef SPCO2_DEBUG_STILLALIVE
  mainTimers.setInterval(2000,GLBcallbackLogging);
#endif 

#ifdef SPCO2_DEBUG_DHT
  mainTimers.setInterval(5000,GLBcallbackLoggingTemp);
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
void goto_idle()
{
  
  u8g2.clearBuffer();
  #ifdef SPCO2_DEBUG_STATES
  Serial.print(F("NST_idle"));
  #endif
  resetTimerMisc();
  GLBptrStateFunc=STATE_idle; 
}


//-------------------------------------------------
void STATE_welcome()
{
  if (display_welcome())
  {
    goto_idle();  
  }
}

//-------------------------------------------------
void STATE_idle()
{
  display_temperature();

  if (GLB_timerMisc == -1)
  {
    GLB_timerMisc=mainTimers.setTimeout(TIMEOUT_SHOW_TEMP,GLBcallbackTimeoutMisc);
  }

  if (GLB_timerMisc_expired){
    u8g2.clearBuffer();
    #ifdef SPCO2_DEBUG_STATES
    Serial.print(F("NST_HUM"));
    #endif
    resetTimerMisc();
    GLBptrStateFunc=STATE_hum; 
  }

/*  if (bBluePressed || bRedPressed || bGreenPressed ){  
    GLBptrStateFunc=STATE_button; 
    u8g2.clearBuffer();
  }*/
}

//-------------------------------------------------
void STATE_hum()
{
  display_humidity();

  if   (GLB_timerMisc == -1)
  {
    GLB_timerMisc=mainTimers.setTimeout(TIMEOUT_SHOW_HUM,GLBcallbackTimeoutMisc);
  }

  if (GLB_timerMisc_expired){
    u8g2.clearBuffer();
    #ifdef SPCO2_DEBUG_STATES
    Serial.print(F("NST_CO2"));
    #endif
    resetTimerMisc();
    GLBptrStateFunc=STATE_CO2; 
  }
}

//-------------------------------------------------
void STATE_CO2()
{
  display_CO2();

  if   (GLB_timerMisc == -1)
  {
    GLB_timerMisc=mainTimers.setTimeout(TIMEOUT_SHOW_CO2,GLBcallbackTimeoutMisc);
  }

  if (GLB_timerMisc_expired){
    GLB_timerMisc_expired=false;
    goto_idle();
    return;
  }
}

//-------------------------------------------------
void STATE_button()
{
  
/*  display_button();

  if   (GLB_timerMisc == -1)
  {
    GLB_timerMisc=mainTimers.setTimeout(TIMEOUT_BUTTON,GLBcallbackTimeoutMisc);
  }

  if (bBluePressed || bRedPressed || bGreenPressed )
  {
    resetTimerMisc();
  }

  if (GLB_timerMisc_expired){
    GLB_timerMisc_expired=false;
    goto_idle();
    return;
  }*/
}





unsigned long t_starting_welcome=0;
int wsBK=0;
bool display_welcome(){
  unsigned long t;
  int ws=0;

  t=millis();
  char msg[6];
  for (int i=0;i<6;i++) msg[i]=' ';
  msg[5]=0;

  if (t_starting_welcome ==0)
    t_starting_welcome=t;
   
  t=t-t_starting_welcome;  

  if (t<100){
    msg[0]='H';  
    ws=0;}  
  else if (t<100){
    msg[1]='O';  
    ws=1;}    
  else if (t<200){
    msg[2]='L';  
    ws=2;}  
  else if (t<300){
    msg[3]='A';  
    ws=3;}  
  else if (t<40){
    msg[4]='!';  
    ws=4;}  
  else if (t<500){
    ws=5;}  
  else if (t<600){
    strcpy(msg,"HOLA!"); 
    ws=6;}  
  else if (t<800){
    ws=7;}  
  else if (t<100){
    strcpy(msg,"HOLA!"); 
    ws=8;}  
  else if (t<1200){
    return true; }

  if (wsBK != ws)   u8g2.clearBuffer();   
  wsBK=ws;
  
  u8g2.firstPage();
  do { 
    u8g2.setFont(u8g2_font_fub42_tr);	
    u8g2.setCursor(10,64);
    u8g2.print(msg);
    
  } while ( u8g2.nextPage() );
  
  return false;

}

//------------------------------------------------
#ifdef SPCO2_DEBUG_DHT
void GLBcallbackLoggingTemp(void)
{
  Serial.println(F("DEBUG: Temp..."));
  Serial.print(" DTH: Temperature:");
  Serial.print(GLBsensorDHTTemp);
  Serial.print(" Celsius. Humidity(%):");
  Serial.println(GLBsensorDHTHum);
}
#endif

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
  GLBsensorDHTTemp = GLBsensorDHT.readTemperature();
  GLBsensorDHTHum  = GLBsensorDHT.readHumidity();

#ifdef SPCO2_DEBUG_LOOP  
  Serial.print("C");
#endif

  if (co2.isPreHeating()) {
    GLBsensorCO2ppm_uart = 0;
    GLBsensorCO2temperature = 0;
  }
  else {
    GLBsensorCO2ppm_uart = co2.readCO2UART();
    //useless GLBsensorCO2ppm_pwm = co2.readCO2PWM();
    GLBsensorCO2temperature = co2.getLastTemperature();
  }



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

