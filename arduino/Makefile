BOARD_TAG    = nano
MCU = atmega328p

ARDUINO_PORT = /dev/ttyUSB0
ARDUINO_LIBS = SimpleTimer U8g2_Arduino SPI Wire Adafruit_Sensor DHT-sensor-library MH-Z-CO2-Sensors SoftwareSerial

USER_LIB_PATH = $(realpath ../SCS/arduino)

#AVRDUDE_ARD_BAUDRATE = 115200
AVRDUDE_ARD_BAUDRATE = 57600

ARDUINO_DIR=/home/jes/arduino-1.8.13
ARDMK_DIR=/usr/share/arduino


#Cheatsheet 
#CPPFLAGS -c pre procesor-, should be on for any compilation C or C++. Not for linking
#CXXFLAGS compiling and linking c++
#CFLAGS   compiling and linking c
#Some flags: -Os optimized in size, setup by default 
#Some flags: -O3 optimized in speed (others 1,2 available also)

############## CONDITIONAL COMPILATION
# As the code and ram is at limit of nano, i must play with ugly ifdefs...
# SPCO2_DEBUG_LOOP
# SPCO2_DEBUG_STATES
# SPCO2_DEBUG_DHT
# SPCO2_DEBUG_STILLALIVE
# SIMPLETIMER_ENABLE_10/5/...1 
# DHTSENSOR_ENABLE_LOWCOST22
# STRINGQUEUE_ENABLE_3/10/1
############## 
CPPFLAGS+=  -DSPCO2_DEBUG_CO2 -DSPCO2_DEBUG_LOOP -DSPCO2_DEBUG_STATES
#-DSPCO2_DEBUG_STATES -DSPCO2_DEBUG_DHT
#CPPFLAGS+=  -DSPCO2_DEBUG_COLOR -DSPCO2_DEBUG_STATES #-DSPCO2_DEBUG_LOOP #-DSPCO2_DEBUG_HCSR04   #-DSPCO2_DEBUG_STILLALIVE  #
CPPFLAGS+= -DSIMPLETIMER_ENABLE_5  


include $(ARDMK_DIR)/Arduino.mk



