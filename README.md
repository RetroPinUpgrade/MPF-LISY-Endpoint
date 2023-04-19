# MPF-LISY-Endpoint
Turn an RPU board into an endpoint for Mission Pinball Framework, using the LISY protocol

This code expects to get MPF instructions from Serial2 on the Arduino, with the following protocol settings:  
  
  
```  
#config_version=5
hardware:
  platform: lisy
lisy:
  connection: serial
  port: /dev/cu.usbserial-1420
  baud: 115200
```
  
