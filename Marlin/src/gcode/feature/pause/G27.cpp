/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


 #include "../../../inc/MarlinConfig.h"

#if ENABLED(NOZZLE_PARK_FEATURE)

#include "../../gcode.h"
#include "../../../libs/nozzle.h"
#include "../../../module/motion.h"
#include "../../module/endstops.h"
/**
 * G27: Park the nozzle
 */
void GcodeSuite::G27() {
  // Don't allow nozzle parking without homing first
  if (homing_needed_error()) return;

  // Define park_point based on active extruder with direct initialization
  xyz_pos_t park_point;
  if (active_extruder == 0) {
    park_point = xyz_pos_t{NOZZLE_PARK_POINT_T0};
  } else {
    park_point = xyz_pos_t{NOZZLE_PARK_POINT_T1};
  }

  #if ENABLED(DUAL_X_CARRIAGE)
    if (active_extruder == 1) endstops.enable_globally(false); // Allow T1 to reach X=331
  #endif
  nozzle.park(parser.ushortval('P'), park_point);
  #if ENABLED(DUAL_X_CARRIAGE)
    if (active_extruder == 1) endstops.enable_globally(true); // Restore endstops
  #endif
}

#endif // NOZZLE_PARK_FEATURE