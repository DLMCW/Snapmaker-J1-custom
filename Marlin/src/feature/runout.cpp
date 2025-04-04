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

/**
 * feature/runout.cpp - Runout sensor support
 */

 #include "../inc/MarlinConfigPre.h"
 #include "../../../snapmaker/module/print_control.h"
 #include "../../../snapmaker/module/system.h"
 
 #if HAS_FILAMENT_SENSOR
 
 #include "runout.h"
 #include "../../snapmaker/module/filament_sensor.h" // Custom ADC sensor
 #include "../MarlinCore.h"
 #include "../module/planner.h"               // For planner.synchronize()
 #include "../feature/pause.h"
 #include "../gcode/queue.h"
 
 #if ENABLED(HOST_ACTION_COMMANDS)
   #include "host_actions.h"
 #endif
 
 #if ENABLED(EXTENSIBLE_UI)
   #include "../lcd/extui/ui_api.h"
 #endif
 
 FilamentMonitor runout;
 
 bool FilamentMonitorBase::enabled = true,
      FilamentMonitorBase::filament_ran_out;
 
 #if ENABLED(HOST_ACTION_COMMANDS)
   bool FilamentMonitorBase::host_handling;
 #endif
 
 bool FilamentMonitor::triggered[NUM_RUNOUT_SENSORS] = { false };
 
 void FilamentMonitor::setup() {
   filament_sensor.init(); // Initialize your custom ADC sensor
   reset();
 }
 
 void FilamentMonitor::reset() {
   filament_ran_out = false;
   ZERO(triggered); // Clear all triggered flags
 }
 
 void FilamentMonitor::filament_present(const uint8_t extruder) {
   triggered[extruder] = false; // Clear runout state for this extruder
   filament_ran_out = false;    // Reset global runout flag
   filament_sensor.reset(extruder); // Reset your sensor’s internal state
 }
 
 void FilamentMonitor::runout_detected(uint8_t e) {
   if (is_hmi_printing) return;  // Skip for HMI prints
   if (is_triggered(e)) return;  // Avoid re-triggering
   SERIAL_ECHOLNPAIR("Runout detected on extruder ", e);
   triggered[e] = true;
   event_filament_runout(e);     // Call the standard event handler
 }
 
 void event_filament_runout(const uint8_t extruder) {
   #if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
     if (migration.in_progress) {
       DEBUG_ECHOLNPGM("Migration Already In Progress");
       return;  // Action already in progress. Purge triggered repeated runout.
     }
     if (migration.automode) {
       DEBUG_ECHOLNPGM("Migration Starting");
       if (extruder_migration()) return;
     }
   #endif
 
   TERN_(EXTENSIBLE_UI, ExtUI::onFilamentRunout(ExtUI::getTool(extruder)));
 
   #if ANY(HOST_PROMPT_SUPPORT, HOST_ACTION_COMMANDS, MULTI_FILAMENT_SENSOR)
     const char tool = '0' + TERN0(MULTI_FILAMENT_SENSOR, extruder);
   #endif
 
   #if ENABLED(HOST_PROMPT_SUPPORT)
     host_action_prompt_begin(PROMPT_FILAMENT_RUNOUT, PSTR("FilamentRunout T"), tool);
     host_action_prompt_show();
   #endif
 
   const bool run_runout_script = !runout.host_handling;
 
   #if ENABLED(HOST_ACTION_COMMANDS)
     if (run_runout_script
       && (strstr(FILAMENT_RUNOUT_SCRIPT, "M600")
         || strstr(FILAMENT_RUNOUT_SCRIPT, "M125")
         || TERN0(ADVANCED_PAUSE_FEATURE, strstr(FILAMENT_RUNOUT_SCRIPT, "M25")))
     ) {
       host_action_paused(false);
     }
     else {
       #ifdef ACTION_ON_FILAMENT_RUNOUT
         host_action(PSTR(ACTION_ON_FILAMENT_RUNOUT " T"), false);
         SERIAL_CHAR(tool);
         SERIAL_EOL();
       #endif
       host_action_pause(false);
     }
     SERIAL_ECHOPGM(" " ACTION_REASON_ON_FILAMENT_RUNOUT " ");
     SERIAL_CHAR(tool);
     SERIAL_EOL();
   #endif // HOST_ACTION_COMMANDS
 
   if (run_runout_script) {
     #if MULTI_FILAMENT_SENSOR
       char script[strlen(FILAMENT_RUNOUT_SCRIPT) + 1];
       sprintf_P(script, PSTR(FILAMENT_RUNOUT_SCRIPT), tool);
       #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
         SERIAL_ECHOLNPAIR("Runout Command: ", script);
       #endif
       queue.inject(script);
     #else
       #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
         SERIAL_ECHOPGM("Runout Command: ");
         SERIAL_ECHOLNPGM(FILAMENT_RUNOUT_SCRIPT);
       #endif
       queue.inject_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
     #endif
   }
 }
 
 #endif // HAS_FILAMENT_SENSOR