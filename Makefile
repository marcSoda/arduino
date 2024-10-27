#CHILD MAKEFILE FOR ARDUINO COMMAND LINE INTERFACE
ARDUINO_DIR = /usr/share/arduino
ARDUINO_PORT = /dev/ttyACM1 #used to be /dev/ttyACM*, but it got stuck on 0. ACM1 is the top left port on the xps 13
USER_LIB_PATH = /home/marc/Development/Arduino/sketchbook/libraries
BOARD_TAG = uno

include /usr/share/arduino/Arduino.mk
