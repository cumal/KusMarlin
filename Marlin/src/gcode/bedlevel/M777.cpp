/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../core/serial.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../module/settings.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

// Custom parameters
#define MAXREPETITIONS 10
#define MAXOFFSET 0.20
#define MAXDIFF 400
#define PRECISSION 0.1
#define STEPSPERMM 400
#define NUM_Z_MOTORS 4
#define Z_MOTORS_POS { { 20, 200 } , { 20, 40 } , { 160 , 200 } , { 160 , 40 } }
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//#define M420_C_USE_MEAN

/**
 * M777: Hardware bed leveling
 */

 void moveMotor(const int mot, float loo) {
  idle();
  // Max height diff 400
  if (abs(loo)>MAXDIFF) {
    if (sgn(loo) == 1) {
      loo = MAXDIFF;
    } else if (sgn(loo) == -1) {
      loo = -MAXDIFF;
    } else {
      SERIAL_ECHOLN("Failed to parse diff");
      return;
    }
  }
  parser.parse("G90");
  GcodeSuite::process_parsed_command();

  if (loo > 0) {
    digitalWrite(Z_DIR_PIN, HIGH); //High=up dir
  } else if (loo < 0) {
    digitalWrite(Z_DIR_PIN, LOW); //LOW=down dir
  } else {
    SERIAL_ECHOLN("Failed to detect direction");
    return;
  }

  analogWrite(Z_ENABLE_PIN, 255); //Deactivate all z stepper
  
  if (mot==0){
    // Activate motor 1 and disable rest
    analogWrite(M1_ENABLE_PIN, 0);
    analogWrite(M2_ENABLE_PIN, 255);
    analogWrite(M3_ENABLE_PIN, 255);
    analogWrite(M4_ENABLE_PIN, 255);
  }
  else if (mot==1){
    analogWrite(M2_ENABLE_PIN, 0);
    analogWrite(M1_ENABLE_PIN, 255);
    analogWrite(M3_ENABLE_PIN, 255);
    analogWrite(M4_ENABLE_PIN, 255);
  }
  else if (mot==2){
    analogWrite(M3_ENABLE_PIN, 0);
    analogWrite(M1_ENABLE_PIN, 255);
    analogWrite(M2_ENABLE_PIN, 255);
    analogWrite(M4_ENABLE_PIN, 255);
  }
  else if (mot==3){
    analogWrite(M4_ENABLE_PIN, 0);
    analogWrite(M1_ENABLE_PIN, 255);
    analogWrite(M2_ENABLE_PIN, 255);
    analogWrite(M3_ENABLE_PIN, 255);
  }

  // Move steps
  planner.synchronize();
  for (float x = 0; x < abs(loo); x++) {
    digitalWrite(Z_STEP_PIN, HIGH);
    delay (5/2);
    digitalWrite(Z_STEP_PIN, LOW);
    delay (5/2);
    idle();
  }

  // Deactivate motor
  if (mot==0){ analogWrite(M1_ENABLE_PIN, 255); }
  else if (mot==1){ analogWrite(M2_ENABLE_PIN, 255); }
  else if (mot==2){ analogWrite(M3_ENABLE_PIN, 255); }
  else if (mot==3){ analogWrite(M4_ENABLE_PIN, 255); }

  // Activate Z steppers to keep height
  analogWrite(Z_ENABLE_PIN, 0);
}

float getDesviation(){
  idle();
  float measuredDesv = 0;
  parser.parse("G91"); // Relative positioning
  GcodeSuite::process_parsed_command();

  if (digitalRead(Z_MIN_PIN) == LOW) { // Endpoint triggered. Go down
    parser.parse("G1 Z0.1 F500");
    while (digitalRead(Z_MIN_PIN) == LOW){
      GcodeSuite::process_parsed_command();
      planner.synchronize();
      measuredDesv = measuredDesv - PRECISSION;
    }
    measuredDesv = measuredDesv + PRECISSION; // last move goes out height
  } else { // Endpoint not triggered. Go up
    parser.parse("G1 Z-0.1 F500");
    while (digitalRead(Z_MIN_PIN) != LOW){
      GcodeSuite::process_parsed_command();
      planner.synchronize();
      measuredDesv = measuredDesv + PRECISSION;
    }
  }
  idle();
  parser.parse("G90"); // Absolute positioning
  GcodeSuite::process_parsed_command();
  return measuredDesv;
}

float getMin(float array[], int size){
  float minimum = array[0];
  for (int i = 0; i < size; i++)
  {
    if (array[i] < minimum) { minimum = array[i]; }
  }
  return minimum;
}

float getMax(float array[], int size){
  float maximun = array[0];
  for (int i = 0; i < size; i++) {
    if (array[i] > maximun) { maximun = array[i]; }
  }
  return maximun;
}

float getSteps(float desviation){
  // 400 steps per mm
  float toApply = STEPSPERMM*desviation;
  return toApply;
}

void aBitDown(){
  idle();
  parser.parse("G91");
  GcodeSuite::process_parsed_command();
  parser.parse("G1 Z5 F500");
  GcodeSuite::process_parsed_command();
  planner.synchronize();
  parser.parse("G90");
  GcodeSuite::process_parsed_command();
}

void printDesviationSummary(float motDesv[]) {
  SERIAL_ECHO("Desviation summary: ");
  SERIAL_ECHO("Motor0:");
  SERIAL_ECHO(motDesv[0]);
  SERIAL_ECHO(", ");
  SERIAL_ECHO("Motor1:");
  SERIAL_ECHO(motDesv[1]);
  SERIAL_ECHO(", ");
  SERIAL_ECHO("Motor2:");
  SERIAL_ECHO(motDesv[2]);
  SERIAL_ECHO(", ");
  SERIAL_ECHO("Motor3:");
  SERIAL_ECHO(motDesv[3]);
  SERIAL_ECHOLN(".");
}
 
void GcodeSuite::M777() {
  SERIAL_ECHOLN("Starting HW bed leveling...");

  int repTimes = 0;

  parser.parse("G90");
  GcodeSuite::process_parsed_command();
  
  parser.parse("G28 X Y");
  GcodeSuite::process_parsed_command();
  planner.synchronize();

  bool run = true;
  xy_pos_t motPosition[NUM_Z_MOTORS] = Z_MOTORS_POS;
  char cmd[20], str_1[16], str_2[16];
  float desvSteps;
  float heightDiff;
  
  while (run){

    float motDesv[NUM_Z_MOTORS];

    for (int i = 0; i < NUM_Z_MOTORS; i++) {
      // Center
      parser.parse("G28 Z");
      GcodeSuite::process_parsed_command();
      planner.synchronize();
      // Move to measure position 0
      parser.parse("G90");
      GcodeSuite::process_parsed_command();
      sprintf_P(cmd, PSTR("G1X%sY%sF1300"), dtostrf(motPosition[i].x, 1, 3, str_1), dtostrf(motPosition[i].y, 1, 3, str_2));
      gcode.process_subcommands_now(cmd);
      // Gets the height difference
      motDesv[i] = getDesviation();
      // Calculate steps and move
      desvSteps = getSteps(motDesv[i]);
      if (desvSteps != 0) { moveMotor(i, desvSteps); }
      planner.synchronize();
    }

    ///////

    printDesviationSummary(motDesv);

    heightDiff = getMax(motDesv, NUM_Z_MOTORS) - getMin(motDesv, NUM_Z_MOTORS);
    
    repTimes=repTimes+1;

    if ( ((int)heightDiff*100 <= (int)MAXOFFSET)*100 || (repTimes == MAXREPETITIONS) ) {
      run = false;
      SERIAL_ECHO("Maximun: ");
      SERIAL_ECHO(heightDiff);
      SERIAL_ECHO("/");
      SERIAL_ECHO(MAXOFFSET);
      SERIAL_ECHO(". Repetitions: ");
      SERIAL_ECHO(repTimes);
      SERIAL_ECHO("/");
      SERIAL_ECHOLN(MAXREPETITIONS);
    } else {
      SERIAL_ECHOLN("Not leveled.");
    }

  }
  
  SERIAL_ECHOLN("Ended HW bed leveling.");
  
}