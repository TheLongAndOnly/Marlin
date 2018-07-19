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
 * Parameters: I<iterations> T<accuracy> A<amplification>
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

  const int8_t temp_iterations = parser.intval('I', z_auto_align_iterations);
  if (0 == temp_iterations || temp_iterations > 30) {
    SERIAL_ECHOLNPGM("?Z-Stepper (I)teration definition out of bounds (1, 30).");
    SERIAL_ECHOLNPGM("<<< G34");
    return;
  }
  else {
    z_auto_align_iterations = temp_iterations;
  }

  float z_auto_align_accuracy = Z_STEPPER_ALIGN_ACC;
  const float temp_accuracy = parser.floatval('T', z_auto_align_accuracy);
  if (0.01f > temp_accuracy || temp_accuracy > 1.0f) {
    SERIAL_ECHOLNPGM("?Z-Stepper (T)arget accuracy definition is out of bounds (0.01, 1.0).");
    SERIAL_ECHOLNPGM("<<< G34");
    return;
  }
  else {
    z_auto_align_accuracy = temp_accuracy;
  }

  float z_auto_align_amplification = Z_STEPPER_ALIGN_AMP;
  const float temp_amplification = parser.floatval('A', z_auto_align_amplification);
  if (0.5f > temp_amplification || temp_amplification > 2.0f) {
    SERIAL_ECHOLNPGM("?Z-Stepper (A)mplification definition is out of bounds (0.5, 2.0).");
    SERIAL_ECHOLNPGM("<<< G34");
    return;
  }
  else {
    z_auto_align_amplification = temp_amplification;
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
  float z_measured[Z_STEPPER_COUNT] = { 0.0f };
  // remember correction from iteration to iteration to determine errors
  float last_z_align_move[Z_STEPPER_COUNT] = { 10000.0f };
  for (uint8_t iteration = 0; iteration < z_auto_align_iterations; ++iteration) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOLNPGM("> probing all positions.");
      }
    #endif

    // reset minimum value
    float z_measured_min = 100000.0f;
    // for each iteration we move through all probe positions (one per Z-Stepper)
    uint8_t zstepper;
    for (zstepper = 0; zstepper < Z_STEPPER_COUNT; ++zstepper) {
      float pos_x = z_auto_align_xpos[zstepper];
      float pos_y = z_auto_align_ypos[zstepper];

      // remember the measured z height per stepper
      z_measured[zstepper] = probe_pt(pos_x, pos_y, PROBE_PT_RAISE, false);

      if (isnan(z_measured[zstepper])) {
        // we need to stop here
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("> probing failed.");
            SERIAL_ECHOLNPGM("<<< G34");
          }
        #endif
        return;        
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("> measure Z position for ", zstepper+1);
          SERIAL_ECHOLNPAIR(" is ", z_measured[zstepper]);
        }
      #endif

      // remember the maximum position to calculate the correction
      z_measured_min = MIN(z_measured_min, z_measured[zstepper]);
    }

    // remember the current z position to return to
    float z_original_position = current_position[Z_AXIS];

    // we can stop iterations early, if all corrections are smaller than our accuracy
    bool breakEarly = true;
    // correct stepper offsets and re-iterate
    for (zstepper = 0; zstepper < Z_STEPPER_COUNT; ++zstepper) {
      // ensure 
      stepper.set_separate_multi_axis(true);
  
      // we enable each stepper separately
      stepper.set_z_lock(true);
      stepper.set_z2_lock(true);
      #if ENABLED(Z_TRIPLE_STEPPER_DRIVERS)
        stepper.set_z3_lock(true);
      #endif

      // calculate current stepper move
      float z_align_move = z_measured[zstepper] - z_measured_min;

      // check, if we loose accuracy compared to last move
      if (last_z_align_move[zstepper] + 1.0f < ABS(z_align_move)) {
        // we need to stop here
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_ECHOLNPGM("> detected decreasing accuracy.");
            SERIAL_ECHOLNPGM("<<< G34");
          }
        #endif
        return;
      }
      else {
        last_z_align_move[zstepper] = ABS(z_align_move);
      }

      // stop early, if all measured points achieve accuracy targets
      breakEarly &= (ABS(z_align_move) <= z_auto_align_accuracy);

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("> correcting Z for stepper ", zstepper+1);
          SERIAL_ECHOLNPAIR(" by ", z_align_move);
        }
      #endif

      switch(zstepper) {
        case 0:
          stepper.set_z_lock(false);
          break;
        case 1:
          stepper.set_z2_lock(false);
          break;
      #if ENABLED(Z_TRIPLE_STEPPER_DRIVERS)
        case 2:
          stepper.set_z3_lock(false);
          break;
      #endif
      }

      // we will be losing home position and need to re-home
      do_blocking_move_to_z(z_auto_align_amplification * z_align_move + current_position[Z_AXIS]);
    }
    stepper.set_z_lock(true);
    stepper.set_z2_lock(true);
    #if ENABLED(Z_TRIPLE_STEPPER_DRIVERS)
      stepper.set_z3_lock(true);
    #endif

    // reset z position to previous position
    do_blocking_move_to_z(z_original_position);

    stepper.set_z_lock(false);
    stepper.set_z2_lock(false);
    #if ENABLED(Z_TRIPLE_STEPPER_DRIVERS)
      stepper.set_z3_lock(false);
    #endif

    stepper.set_separate_multi_axis(false);

    if (breakEarly) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOLNPGM("> achieved target accuracy.");
       }
      #endif
      break;
    }
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

  #if HAS_LEVELING  
    #if ENABLED(RESTORE_LEVELING_AFTER_G34)
      set_bed_leveling_enabled(leveling_was_active);
    #endif
  #endif

  // after this operation we have lost the z homing position
  set_axis_is_not_at_home(Z_AXIS);
  
  // rehome
  gcode.G28(false);

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

  const float y_pos = parser.floatval('Y', z_auto_align_ypos[z_stepper-1]);
  if (!WITHIN(y_pos, Y_MIN_POS, Y_MAX_POS)) {
    SERIAL_PROTOCOLLNPGM("?(Y)-Position is implausible out of limits.");
    return;
  }

  z_auto_align_xpos[z_stepper-1] = x_pos;
  z_auto_align_ypos[z_stepper-1] = y_pos;
}

#endif // Z_STEPPER_AUTO_ALIGN
