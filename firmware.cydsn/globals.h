// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         Jul 29, 2013
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED



//=================================================================     includes
#include <device.h>
#include "stdlib.h"
#include "math.h"
#include "commands.h"
#include "utils.h"


//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION         "QBMMP v5.6.1"

#define NUM_OF_MOTORS           2
#define NUM_OF_SENSORS          3
#define NUM_OF_ANALOG_INPUTS    3

//==============================================================================
//                                                                       CONTROL
//==============================================================================

#define PWM_MAX_VALUE   100         // PWM is from 0 to 100, this value is used
                                    // to limit this value
#define PWM_DEAD        0           // deadband value, is directly added to the
                                    // value of PWM always limited to 100

// WARNING POS_INTEGRAL_SAT_LIMIT need to be lower than 2^17 or you have to modify
// the code in the motor control function
#define POS_INTEGRAL_SAT_LIMIT   100000  // Anti wind-up
#define CURR_INTEGRAL_SAT_LIMIT  100000  // Anti wind-up
#define CALIB_CURRENT            1000    // Max current for calibration (mA)
#define DEFAULT_CURRENT_LIMIT    1500    // Current limit when using CURR_AND_POS_CONTROL


//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

// Main frequency 1000 Hz

#define CALIBRATION_DIV         100     // 10 Hz

#define DIV_INIT_VALUE          1

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE           0
#define TRUE            1

#define DEFAULT_EEPROM_DISPLACEMENT 8 // in pages

//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references

struct st_ref {

    int32 pos[NUM_OF_MOTORS];       // motor reference position
    uint8 onoff;                    // enable flags

};

//=============================================================     measurements

struct st_meas {

    int32 pos[NUM_OF_SENSORS];      // sensor position
    int32 curr[NUM_OF_MOTORS];      // motor currents
    int32 rot[NUM_OF_SENSORS];      // sensor rotations
    int16 vel[NUM_OF_SENSORS];      // sensor velocity

};

//==============================================================     data packet

struct st_data {

    uint8   buffer[128];                    // CMD/DATA/CHECKSUM
    int16   length;                         // length
    int16   ind;                            // index
    uint8   ready;                          // Flag

};

//============================================     settings stored on the memory

struct st_mem {

    uint8   flag;                       // Device has been configured               1
    uint8   id;                         // device ID                                1

    int32   k_p;                        // Proportional constant                    4
    int32   k_i;                        // Derivative constant                      4
    int32   k_d;                        // Integrative constant                     4

    int32   k_p_c;                      // Proportional constant current            4
    int32   k_i_c;                      // Derivative constant current              4
    int32   k_d_c;                      // Integrative constant current             4

    int16   current_limit;              // Limit for absorbed current               2

    uint8   activ;                      // Activation upon startup                  1
    uint8   input_mode;                 // Input mode                               1       30
    uint8   control_mode;               // Control mode                             1

    uint8   res[NUM_OF_SENSORS];        // Angle resolution                         1 (3)
    int32   m_off[NUM_OF_SENSORS];      // Measurement offset                       4 (12)
    float   m_mult[NUM_OF_SENSORS];     // Measurement multiplier                   4 (12)  28
    uint8   pos_lim_flag;               // Position limit active/inactive           1
    int32   pos_lim_inf[NUM_OF_MOTORS]; // Inferior position limit for motors       4 (8)
    int32   pos_lim_sup[NUM_OF_MOTORS]; // Superior position limit for motors       4 (8)

    uint16  max_stiffness;              // Max stiffness value obtained
                                        // during calibration                       2       
    uint8   deflection_control;         // Deflection control flag                  1       
    int32   max_step_neg;               // Maximum velocity for negative inputs     4       
    int32   max_step_pos;               // Maximum velocity for positive inputs     4       28

                                                                                        //  86
};

//=================================================     device related variables

struct st_dev {

    int32   tension;                // Power supply tension
    float   tension_conv_factor;    // Used to calculate input tension
    uint8   tension_valid;
    uint8   pwm_limit;

};

//=================================================     counters to debug commands sent

struct st_count {

    uint16  activ;
    uint16  set_inputs;
    uint16  set_pos_stiff;
    uint16  get_meas;
    uint16  get_curr;
    uint16  get_curr_meas;
    uint16  get_vel;
    uint16  get_activ;
    uint16  get_inputs;
    uint16  get_info;
    uint16  set_param;
    uint16  get_param;
    uint16  ping;
    uint16  store_param;
    uint16  store_default;
    uint16  restore;
    uint16  init;
    uint16  bootloader;
    uint16  calibrate;
    uint16  get_counts;

};

//=================================================     calibration status

enum calibration_status {

    STOP        = 0,
    START       = 1,
    CONTINUE_1  = 2,
    CONTINUE_2  = 3,
    PAUSE_1     = 4,
    PAUSE_2     = 5

};

//====================================      external global variables declaration


extern struct st_ref    g_ref;          // motor variables
extern struct st_meas   g_meas;         // measurements
extern struct st_data   g_rx;           // income data
extern struct st_mem    g_mem, c_mem;   // memory
extern struct st_dev    device;         //device related variables
extern struct st_count  g_count;

extern uint32 timer_value;

extern uint8 calibration_flag;

extern uint8 reset_last_value_flag;
extern int32 ref_input[NUM_OF_MOTORS];

// -----------------------------------------------------------------------------

#endif

//[] END OF FILE