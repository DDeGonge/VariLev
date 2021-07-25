/* Pinouts are for the OSR V1 control board */
#include <OSR.h>
#include <string>


#define STEPS_PER_REV 12800
#define REV_PER_S 3
#define MAX_MSG_LEN 50

#define Serial SERIAL_PORT_USBVIRTUAL

using namespace std;

bool respondToSerial(char (&serial_data) [MAX_MSG_LEN])
{
  uint8_t index = 0;
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      char newchar = Serial.read();
      if ((newchar != '\n') and (index < MAX_MSG_LEN)) {
        serial_data[index] = newchar;
        index++;
      }
      else {
        break;
      }
    }
    return true;
  }
  return false;
}

void clear_data(char (&serial_data) [MAX_MSG_LEN])
{
  for (uint16_t i = 0; i < MAX_MSG_LEN; i++) {
    serial_data[i] = '\0';
  }
}

int32_t parse_int(string inpt)
{
  string temp_arg_char = "";
  for (uint32_t i = 0; i < inpt.length(); i++)
  {
    temp_arg_char += inpt[i];
  }

  return stoi(temp_arg_char);
}

void setup() {
  Serial.begin(250000);
}

void loop() {
  TMC2041 Driver0(D0EN, D0CS);
  TMCstep motdrive0 = TMCstep(D0S0S, D0S0D, Driver0, 0);
  motdrive0.set_run_current(20);
  motdrive0.set_hold_current(5);
  motdrive0.set_dir(1);
  motdrive0.disable();

  int32_t winds = 0;

  while(true)
  {
    char serial_data[MAX_MSG_LEN];
    clear_data(serial_data);
    uint16_t step_dwell_s = (1000000 / (REV_PER_S * STEPS_PER_REV)) - 1;
    if (respondToSerial(serial_data)) 
    {
      motdrive0.enable();
      delay(100);
      int32_t newwinds = parse_int(serial_data);
      clear_data(serial_data);
      for(int32_t wind = 0; wind < newwinds; wind++)
      {
        Serial.print("WIND: ");
        Serial.println(winds);
        winds++;
        for(int32_t s = 0; s < STEPS_PER_REV; s++)
        {
          motdrive0.step();
          delayMicroseconds(step_dwell_s);
        }
      }
      motdrive0.disable();
    }
  }
}
