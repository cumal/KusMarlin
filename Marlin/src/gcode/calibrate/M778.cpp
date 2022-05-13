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

#if HAS_LEVELING

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

//#define M420_C_USE_MEAN

/**
 * M778: Adjust Center Height
 */
 
void GcodeSuite::M778() {
  SERIAL_ECHOLN("Starting center height...");
  
  parser.parse("G90");
  GcodeSuite::process_parsed_command();
  
  parser.parse("G28");
  GcodeSuite::process_parsed_command();
  planner.synchronize();

  parser.parse("G1 X145 Y110 F1000");
	GcodeSuite::process_parsed_command();
	planner.synchronize();

  parser.parse("G28 Z");
	GcodeSuite::process_parsed_command();
	planner.synchronize();

  parser.parse("G28 X Y");
	GcodeSuite::process_parsed_command();
	planner.synchronize();
    
  SERIAL_ECHOLN("Ended center height.");
  
}

#endif // HAS_LEVELING
