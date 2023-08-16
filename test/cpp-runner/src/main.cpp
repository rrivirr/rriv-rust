#include <Arduino.h>
#include "rrivrust.h"

void * rriv_cmd;

void datalogger_set(const char *json_strang)
{
  pinMode(PA5, OUTPUT);
  digitalWrite(PA5, LOW);
  delay(950);
  digitalWrite(PA5, HIGH);
  delay(950);
  digitalWrite(PA5, LOW);
  delay(950);
  digitalWrite(PA5, HIGH);
  delay(950);
  return 0;
}

void setup(void) {
  rriv_cmd = command_service_init();
  pinMode(PA5, OUTPUT);
  digitalWrite(PA5, HIGH);
  delay(750);
  digitalWrite(PA5, LOW);
  delay(750);
  digitalWrite(PA5, HIGH);
  delay(750);
  command_service_register_command(rriv_cmd, "datalogger", "set", &datalogger_set);
}

void loop(void)
{
  command_service_run_loop_iteration(rriv_cmd);
}
