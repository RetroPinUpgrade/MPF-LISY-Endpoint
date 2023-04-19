/**************************************************************************
    This pinball code is distributed in the hope that it
    will be useful, but WITHOUT ANY WARRANTY; without even the implied 
    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    See <https://www.gnu.org/licenses/>. 
*/

#include "RPU_Config.h"
#include "RPU.h"
//#include <EEPROM.h>

#define GAME_MAJOR_VERSION  2023
#define GAME_MINOR_VERSION  1
#define DEBUG_MESSAGES  1

#define LISYSerial Serial2


unsigned long CurrentTime = 0;
#define MAX_LISY_MESSAGE_LENGTH 1024
byte LISYMessage[MAX_LISY_MESSAGE_LENGTH];
int LISYBytesSeen;
int LISYBytesExpected;
boolean SwitchStateNeedsToBeCleared[256];


void setup() {

  if (DEBUG_MESSAGES) {
    // If debug is on, set up the Serial port for communication
    Serial.begin(115200);
    Serial.write("Starting\n");
  }

  CurrentTime = millis();

  // Tell the OS about game-specific switches
  // (this is for software-controlled pop bumpers and slings)
  //RPU_SetupGameSwitches(NUM_SWITCHES_WITH_TRIGGERS, NUM_PRIORITY_SWITCHES_WITH_TRIGGERS, SolenoidAssociatedSwitches);

  // Set up the chips and interrupts
  unsigned long initResult = 0;
  initResult = RPU_InitializeMPU(RPU_CMD_BOOT_NEW | RPU_CMD_INIT_AND_RETURN_EVEN_IF_ORIGINAL_CHOSEN | RPU_CMD_PERFORM_MPU_TEST/*, SW_CREDIT_RESET*/);

  if (DEBUG_MESSAGES) {
    char buf[128];
    sprintf(buf, "Return from init = 0x%04lX\n", initResult);
    Serial.write(buf);
    if (initResult&RPU_RET_6800_DETECTED) Serial.write("Detected 6800 clock\n");
    else if (initResult&RPU_RET_6802_OR_8_DETECTED) Serial.write("Detected 6802/8 clock\n");
    Serial.write("Back from init\n");    
  }
  if (initResult&RPU_RET_ORIGINAL_CODE_REQUESTED) {
    if (DEBUG_MESSAGES) {
      Serial.write("Running original code\n");
    }
    delay(100);
    while (1);
  }

  RPU_DisableSolenoidStack();
  RPU_SetDisableFlippers(true);

  if (DEBUG_MESSAGES) {
    Serial.write("Waiting for LISY command\n");
  }

  LISYBytesSeen = 0;
  LISYBytesExpected = 1;
  LISYSerial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, false);

  //LISYSerial.write("LISY PORT\n");
//  RPU_SetCoinLockout((Credits >= MaximumCredits) ? true : false);

  for (byte count=0; count<255; count++) {
    SwitchStateNeedsToBeCleared[count] = false;
  }
}

/**************
 * TODO:
 *  need to add up/down switch for Williams
 *  need to add clear high score switch for Williams
 */
byte GetChangedSwitch() {
  byte nextSwitch = RPU_PullFirstFromSwitchStack();

  if (nextSwitch!=SWITCH_STACK_EMPTY) {
    if (nextSwitch==SW_SELF_TEST_SWITCH) {
      // We'll map the self-test switch as the last switch
      nextSwitch = RPU_GetSwitchCount()-1;
      Serial.write("Self test\n");
    }
    SwitchStateNeedsToBeCleared[nextSwitch] = true;
    char buf[128];
    sprintf(buf, "Switch = 0x%02X\n", nextSwitch);
    Serial.write(buf);
    return (nextSwitch | 0x80); // b7 on means that the switch is on
  } else {
    // See if there are any switch states that need to be cleared
    for (byte count=0; count<255; count++) {
      if (SwitchStateNeedsToBeCleared[count]) {
        if (!RPU_ReadSingleSwitchState(count)) {
          // We can clear this switch
          SwitchStateNeedsToBeCleared[count] = false;
          return (byte)count;
        }
      }
    }
  }

  return 127; // no switches
}



boolean ProcessLISYMessage(byte *lisyMSG, int numBytes) {
  if (lisyMSG==NULL) return false;
  boolean messageHandled = false;

//  char buf[128];
//  sprintf(buf, "Command = 0x%02X\n", lisyMSG[0]);
//  Serial.write(buf);

  switch (lisyMSG[0]) {
    case 0x00:
      // Queried for architecture name
      Serial.write("Asked for name\n");
      LISYSerial.write("LISY 35");
      LISYSerial.write(0);    
      messageHandled = true;  
      break;
    case 0x01:
      // Queried for firmware version -- not sure how to answer, 
      // so I'm using the example from the docs
      Serial.write("Asked for firmware\n");
      LISYSerial.write("4.01");
      LISYSerial.write(0);
      messageHandled = true;
      break;
    case 0x02:
      // Queried for API Version
      // so I'm using the example from the docs
      Serial.write("Asked for API Version\n");
      LISYSerial.write("0.08");
      LISYSerial.write(0);
      messageHandled = true;
      break;
    case 0x03:
      // Queried for number of simple lamps
      LISYSerial.write((byte)RPU_GetLampCount());
      messageHandled = true;
      break;
    case 0x04:
      // Queried for solenoid count
      LISYSerial.write(RPU_GetSolenoidCount());
      messageHandled = true;
      break;
    case 0x05:
      // Queried for sound count
      // This is only returning sounds that
      // will be played on built-in hardware
      LISYSerial.write(RPU_GetSoundCount());
      messageHandled = true;
      break;
    case 0x06:
      // Queried for number of displays?
#if (RPU_MPU_ARCHITECTURE==15)
      LISYSerial.write(4);
#else      
      LISYSerial.write(5);
#endif
      break;
    case 0x07:
      // Queried for details about displays
      // I'm not completely sure how to answer
      // for System 7 displays, because they have
      // comma digits in the 1st and 4th places, 
      // but this seems to expect every digit is the same?
#if (RPU_MPU_ARCHITECTURE==15)
      LISYSerial.write(7);
      LISYSerial.write(4); // 14 segment, fully addressable
#else
      if (lisyMSG[1]<4) {
        LISYSerial.write(RPU_OS_NUM_DIGITS);
        LISYSerial.write(1); // BCD 7 because of what I said above
      } else {
        LISYSerial.write(4); // put together ball in play and credits
        LISYSerial.write(1); // BCD 7 because of what I said above
      }
#endif      
      messageHandled = true;
      break;
    case 0x08:
      // Queried for "game info" but it won't be used?
      LISYSerial.write("RPU");
      LISYSerial.write(0);
      messageHandled = true;
      break;
    case 0x09:
      // Queried for number of switches
      LISYSerial.write(RPU_GetSwitchCount());
      messageHandled = true;
      break;
    case 0x0A:
      // Queried for lamp status
      LISYSerial.write(RPU_ReadLampState(lisyMSG[1]));
      messageHandled = true;
      break;
    case 0x0B:
      // Set lamp state on
      RPU_SetLampState(lisyMSG[1], 1);
      messageHandled = true;
      break;
    case 0x0C:
      // Set lamp state off
      RPU_SetLampState(lisyMSG[1], 0);
      messageHandled = true;
      break;
    case 0x14:
      // Get solenoid status
      LISYSerial.write(RPU_GetSolenoidStatus(lisyMSG[1]));
      messageHandled = true;
      break;
    case 0x28:
      // Get switch state
      // TODO: might need to support self-test, clear high score, and up/down
      LISYSerial.write(RPU_ReadSingleSwitchState(lisyMSG[1]));
      messageHandled = true;
      break;
    case 0x29:
      // Get any changed switch (i.e. switch from stack)
      // running this through a function so I can fake
      // a message for when a switch is no longer closed
      LISYSerial.write(GetChangedSwitch());
      messageHandled = true;
      break;
    case 0x64:
      Serial.write("Asked to reset\n");
      // Reset and acknowledge
      LISYSerial.write(0);
      messageHandled = true;
      break;
    case 0x65:
      // Watchdog response
      LISYSerial.write(0);
      messageHandled = true;
      break;
  }

  (void)numBytes;

  return messageHandled;
}

int GetLISYMessageLength(byte messageType) {
  if (messageType<0x07) return 1;
  else if (messageType==0x07) return 2;
  else if (messageType<0x0A) return 1;
  else if (messageType>=0x0A && messageType<=0x0C) return 2;
  else if (messageType>=0x14 && messageType<=0x17) return 2;
  else if (messageType==0x18) return 3;
  else if (messageType==0x28) return 2;
  else if (messageType==0x29) return 1;
  else if (messageType==0x64) return 1;
  else if (messageType==0x65) return 1;
  
  return 0;
}


boolean LISYUpdate() {

//  char buf[256];

  while (LISYSerial.available()>0) {

    byte byteRead = LISYSerial.read();
    
    if (LISYBytesSeen==0) {
      LISYBytesExpected = GetLISYMessageLength(byteRead);
//      sprintf(buf, "MSG 0x%02X len: 0x%02X\n", byteRead, LISYBytesExpected);
//      Serial.write(buf);
    }
    LISYMessage[LISYBytesSeen] = byteRead;
    LISYBytesSeen += 1;
    if (LISYBytesSeen>=MAX_LISY_MESSAGE_LENGTH) {
      LISYBytesSeen = 0;
      LISYBytesExpected = 0;
      return false;
    }
    
    if (LISYBytesSeen>0 && LISYBytesExpected==LISYBytesSeen) {
      ProcessLISYMessage(LISYMessage, LISYBytesSeen);
      LISYBytesExpected = 0;
      LISYBytesSeen = 0;
    } else if (LISYBytesExpected==0) {
      LISYBytesSeen = 0;
    }
    
//    sprintf(buf, "Saw 0x%02X\n", dat);
//    Serial.write(buf);
//    if (dat==0x0A) sawSomething = true;
//    if (dat==0x64) LISYSerial.write(0);
  }

  return true;
}



//unsigned long NumLoops = 0;
//unsigned long LastLoopReportTime = 0;

boolean LISYProtocolError = false;

void loop() {

/*
  if (DEBUG_MESSAGES) {
    NumLoops += 1;
    if (CurrentTime>(LastLoopReportTime+1000)) {
      LastLoopReportTime = CurrentTime;
      char buf[128];
      sprintf(buf, "Loop running at %lu Hz\n", NumLoops);
      Serial.write(buf);
      NumLoops = 0;
    }
  }
*/
  if (!LISYProtocolError) {
    if (!LISYUpdate()) {
      LISYProtocolError = true;
    }
  } else {
    // need to halt safely because there has been an error
    digitalWrite(13, (CurrentTime/300)%2);
  }
  CurrentTime = millis();
  RPU_Update(CurrentTime);
}
