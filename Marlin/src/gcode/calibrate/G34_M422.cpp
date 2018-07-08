/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ENABLED(Z_STEPPER_AUTO_ALIGN)

#include "../gcode.h"
#include "../../module/delta.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/endstops.h"
#include "../../lcd/ultralcd.h"

#if HOTENDS > 1
  #include "../../module/tool_change.h"
#endif

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if HOMING_Z_WITH_PROBE || ENABLED(BLTOUCH)
  #include "../../module/probe.h"
#endif

// Data
float z_auto_align_xpos[] = Z_STEPPER_ALIGN_XPOS;
float z_auto_align_ypos[] = Z_STEPPER_ALIGN_YPOS;

/**
 * G34 - Z-Stepper automatic alignment
 *
 * Parameters:
 *
 */
void GcodeSuite::G34() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOLNPGM(">>> G34");
      log_machine_info();
    }
  #endif

  #if ENABLED(DELTA)
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM("> Auto Z-Stepper alignment not supported for Delta.");
        SERIAL_ECHOLNPGM("<<< G34");
      }
    #endif
    return;
  #endif

  if (!TEST(axis_known_position, X_AXIS) || !TEST(axis_known_position, Y_AXIS)) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM("> required to have at least X & Y homed.");
        SERIAL_ECHOLNPGM("<<< G34");
      }
    #endif
    return;
  }

  uint8_t z_auto_align_iterations = Z_STEPPER_ALIGN_ITERATIONS;

  const int8_t iterations = parser.intval('I', z_auto_align_iterations);
  if (0 == iterations || iterations > 30) {
    SERIAL_ECHOLNPGM("?Z-Stepper (I)teration definition out of bounds (1, 30).");
    SERIAL_ECHOLNPGM("<<< G34");
    return;
  }
  else {
    z_auto_align_iterations = iterations;
  }

  // Wait for planner moves to finish!
  planner.synchronize();

  // Disable the leveling matrix before homing
  #if HAS_LEVELING
    #if ENABLED(RESTORE_LEVELING_AFTER_G34)
      const bool leveling_was_active = planner.leveling_active;
    #endif
    set_bed_leveling_enabled(false);
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    workspace_plane = PLANE_XY;
  #endif

  #if ENABLED(BLTOUCH)
    bltouch_command(BLTOUCH_RESET);
    set_bltouch_deployed(false);
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    const uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif

  #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
    extruder_duplication_enabled = false;
  #endif

  // start calibration iterations
  for (uint8_t iteration = 0; iteration < iterations; ++iteration) {
    // for each iteration we move through all probe positions (one per Z-Stepper)
    
  }

  // Restore the active tool after homing
  #if HOTENDS > 1
    #if ENABLED(PARKING_EXTRUDER)
      #define NO_FETCH false // fetch the previous toolhead
    #else
      #define NO_FETCH true
    #endif
    tool_change(old_tool_index, 0, NO_FETCH);
  #endif

  #if ENABLED(RESTORE_LEVELING_AFTER_G34)
    set_bed_leveling_enabled(leveling_was_active);
  #endif

  // we are finished
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< G34");
  #endif
}

/**
 * M422 - Z-Stepper automatic alignment parameter selection
 *
 */
void GcodeSuite::M422() {
  if (!parser.seen('A')) {
    SERIAL_PROTOCOLLNPGM("?Z-Stepper (A)xis definition required.");
    return;
  }

  const int8_t z_stepper = parser.intval('A', 0);
  if (z_stepper <= 0 || z_stepper > Z_STEPPER_COUNT) {
    SERIAL_PROTOCOLLNPGM("?Z-Stepper (A)xis definition invalid.");
    return;
  }

  const float x_pos = parser.floatval('X', z_auto_align_xpos[z_stepper-1]);
  if (!WITHIN(x_pos, X_MIN_POS, X_MAX_POS)) {
    SERIAL_PROTOCOLLNPGM("?(X)-Position is implausible out of limits.");
    return;
  }
  else {
    z_auto_align_xpos[z_stepper-1] = x_pos;
  }

  const float y_pos = parser.floatval('Y', z_auto_align_ypos[z_stepper-1]);
  if (!WITHIN(y_pos, Y_MIN_POS, Y_MAX_POS)) {
    SERIAL_PROTOCOLLNPGM("?(Y)-Position is implausible out of limits.");
    return;
  }
  else {
    z_auto_align_ypos[z_stepper-1] = y_pos;
  }
}

#endif // Z_STEPPER_AUTO_ALIGN
