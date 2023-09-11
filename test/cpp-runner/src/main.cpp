// #include <Arduino.h>
#include "rrivrust.h"

void *rriv_cmd;

#define LED_PIN PA10

void datalogger_set(const char *json_strang)
{
  // digitalWrite(LED_PIN, LOW);
  // delay(75);
  // digitalWrite(LED_PIN, HIGH);
  // delay(75);
  // digitalWrite(LED_PIN, LOW);
  // delay(75);
  // digitalWrite(LED_PIN, HIGH);
  // delay(75);
}

void setup(void)
{

  // commented out the maple core's hidden, forced init() call
  // so the rust hal can set clocks before the maple core does it for real
  rriv_cmd = command_service_init();
  // init();

  // pinMode(LED_PIN, OUTPUT_OPEN_DRAIN);
  // digitalWrite(LED_PIN, HIGH);
  // delay(500);
  // digitalWrite(LED_PIN, LOW);
  // delay(500);
  // digitalWrite(LED_PIN, HIGH);
  // delay(500);
  // digitalWrite(LED_PIN, LOW);
  // delay(500);
  // digitalWrite(LED_PIN, HIGH);
  // delay(500);
  command_service_register_command(rriv_cmd, "datalogger", "set", &datalogger_set);
  // digitalWrite(LED_PIN, HIGH);
  // delay(200);
  // digitalWrite(LED_PIN, LOW);
  // delay(200);
  // digitalWrite(LED_PIN, HIGH);
  // delay(200);
  // digitalWrite(LED_PIN, LOW);
  // delay(200);
  // digitalWrite(LED_PIN, HIGH);
  // delay(200);
}

void loop(void)
{
  command_service_run_loop_iteration(rriv_cmd);
}
