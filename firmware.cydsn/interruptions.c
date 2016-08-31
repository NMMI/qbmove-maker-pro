// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         interruptions.c
*
* \brief        Interruption functions are in this file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/


//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>

#include "globals.h"
#include "utils.h"

//===================================================================     global

uint8 timer_flag = 0;

// PWM vaules needed to obtain 8 Volts given a certain input tension
// Numbers are sperimentally calculated //[index] (milliampere)
static const uint8 hitech_pwm_preload_values[36] = {100,   //0 (8000)
                                                     76,
                                                     71,
                                                     69,
                                                     67,
                                                     65,     //5 (10500)
                                                     63,
                                                     61,
                                                     60,
                                                     58,
                                                     57,     //10 (13000)
                                                     56,
                                                     55,
                                                     54,
                                                     53,
                                                     52,     //15 (15500)
                                                     51,
                                                     50,
                                                     49,
                                                     49,
                                                     48,     //20 (18000)
                                                     47,
                                                     47,
                                                     46,
                                                     45,
                                                     45,     //25 (20500)
                                                     45,
                                                     44,
                                                     44,
                                                     43,
                                                     43,     //30 (23000)
                                                     43,
                                                     43,
                                                     42,
                                                     42,
                                                     42};    //35 (25500)

//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================
// Processing RS-485 data frame:
//
// - 0:     Waits for beginning characters
// - 1:     Waits for ID;
// - 2:     Data length;
// - 3:     Receive all bytes;
// - 4:     Wait for another device end of transmission;
//
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt) {

    //===========================================     local variables definition

    static uint8    state = 0;                          // state
    static struct   st_data data_packet;                // local data packet
    static uint8    rx_queue[3];                        // last 3 bytes received

    static uint8    rx_data;                            // RS485 UART rx data
    static uint8    rx_data_type;                       // my id?

    //======================================================     receive routine

    // get data while rx fifo is not empty
    while (UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_FIFO_NOTEMPTY) {
        rx_data = UART_RS485_GetChar();
        switch (state) {
            //-----     wait for frame start     -------------------------------
            case 0:

                rx_queue[0] = rx_queue[1];
                rx_queue[1] = rx_queue[2];
                rx_queue[2] = rx_data;

                if ((rx_queue[1] == ':') && (rx_queue[2] == ':')) {
                    rx_queue[0] = 0;
                    rx_queue[1] = 0;
                    rx_queue[2] = 0;
                    state       = 1;
                } else if (
                (rx_queue[0] == 63) &&      //ASCII - ?
                (rx_queue[1] == 13) &&      //ASCII - CR
                (rx_queue[2] == 10)) {      //ASCII - LF
                    infoGet(INFO_ALL);
                }
                break;

            //-----     wait for id     ----------------------------------------
            case  1:

                // packet is for my ID or is broadcast
                if (rx_data == c_mem.id || rx_data == 0) {
                    rx_data_type = 0;
                } else {                //packet is for others
                    rx_data_type = 1;
                }
                data_packet.length = -1;
                state = 2;
                break;

            //-----     wait for length     ------------------------------------
            case  2:

                data_packet.length = rx_data;
                // check validity of pack length
                if (data_packet.length <= 1) {
                    data_packet.length = -1;
                    state = 0;
                } else if (data_packet.length > 128) {
                    data_packet.length = -1;
                    state = 0;
                } else {
                    data_packet.ind = 0;
                    if(rx_data_type == 0) {
                        state = 3;          // packet for me or broadcast
                    } else {
                        state = 4;          // packet for others
                    }
                }
                break;

            //-----     receving     -------------------------------------------
            case 3:

                data_packet.buffer[data_packet.ind] = rx_data;
                data_packet.ind++;
                // check end of transmission
                if (data_packet.ind >= data_packet.length) {
                    // verify if frame ID corresponded to the device ID
                    if (rx_data_type == 0) {
                        // copying data from buffer to global packet
                        memcpy(g_rx.buffer, data_packet.buffer, data_packet.length);
                        g_rx.length = data_packet.length;
                        g_rx.ready  = 1;
                        commProcess();
                    }
                    data_packet.ind    = 0;
                    data_packet.length = -1;
                    state              = 0;
                }
                break;

            //-----     other device is receving     ---------------------------
            case 4:
                if (!(--data_packet.length)) {
                    data_packet.ind    = 0;
                    data_packet.length = -1;
                    RS485_CTS_Write(1);
                    RS485_CTS_Write(0);
                    state              = 0;
                }
                break;
        }
    }
}

//==============================================================================
//                                                            FUNCTION SCHEDULER
//==============================================================================
// Call all the function with the right frequency
//==============================================================================

void function_scheduler(void) {
    // Base frequency 1000 Hz

    static uint16 counter_calibration = DIV_INIT_VALUE;

    // Optimized manual scheduling
    analog_read_init(0);
    encoder_reading(0);
    encoder_reading(1);
    analog_read_end(0);

    analog_read_init(1);
    encoder_reading(2);
    motor_control(0);
    analog_read_end(1);

    analog_read_init(2);
    motor_control(1);
    analog_read_end(2);


    // Divider 100, freq = 10 Hz
    if (calibration_flag != STOP) {
        if (counter_calibration == CALIBRATION_DIV) {
            calibration();
            counter_calibration = 0;
        }
        counter_calibration++;
    }


    timer_value = (uint32)MY_TIMER_ReadCounter();
    MY_TIMER_WriteCounter(5000000);
}


//==============================================================================
//                                                                MOTORS CONTROL
//==============================================================================

void motor_control(uint8 index) {
    static int32 curr_ref;

    static int32 pwm_input;
    static int32 pos_error;
    static int32 curr_error;
    static int32 defl_input[NUM_OF_MOTORS];

    static uint8 direction;

    static int32 prev_pos[NUM_OF_MOTORS];
    static int32 prev_curr[NUM_OF_MOTORS];

    static int32 pos_error_sum[NUM_OF_MOTORS];
    static int32 curr_error_sum[NUM_OF_MOTORS];

    static int32 prev_curr_err[NUM_OF_MOTORS];

    // check index value
    if (index >= NUM_OF_MOTORS)
        return;

    // --------------- USE THIRD ENCODER AS INPUT FOR BOTH MOTORS --------------
    if (c_mem.input_mode == INPUT_MODE_ENCODER3) {

        g_ref.pos[index] = g_meas.pos[2];

        // position limit
        if (c_mem.pos_lim_flag) {
            if (g_ref.pos[index] < c_mem.pos_lim_inf[index]) g_ref.pos[index] = c_mem.pos_lim_inf[index];
            if (g_ref.pos[index] > c_mem.pos_lim_sup[index]) g_ref.pos[index] = c_mem.pos_lim_sup[index];
        }
    }

    if (c_mem.max_step_pos != 0 && c_mem.max_step_neg != 0) {
        if ((ref_input[index] - g_ref.pos[index]) > c_mem.max_step_pos)
            g_ref.pos[index] += c_mem.max_step_pos;
        else { 
            if ((ref_input[index] - g_ref.pos[index]) < c_mem.max_step_neg)
                g_ref.pos[index] += c_mem.max_step_neg;
            else 
                g_ref.pos[index] = ref_input[index];
        }
    }
    else
        ref_input[index] = g_ref.pos[index]; 

    switch(c_mem.control_mode) {
        case CONTROL_ANGLE:
            // position error

            if(g_mem.deflection_control) {
                defl_input[index] = g_ref.pos[index] + g_meas.pos[2];

                pos_error = defl_input[index] - g_meas.pos[index];
                pos_error_sum[index] += pos_error;
            }
            else { 
                pos_error = g_ref.pos[index] - g_meas.pos[index];
                pos_error_sum[index] += pos_error;
            }

            //anti wind-up
            if (pos_error_sum[index] > POS_INTEGRAL_SAT_LIMIT) {
                pos_error_sum[index] = POS_INTEGRAL_SAT_LIMIT;
            } else if (pos_error_sum[index] < -POS_INTEGRAL_SAT_LIMIT) {
                pos_error_sum[index] = -POS_INTEGRAL_SAT_LIMIT;
            }

            // pwm_input init
            pwm_input = 0;

            // Proportional
            if (c_mem.k_p != 0) {
                if ((pos_error > 131072) || (pos_error < -131072)) {  //if grater than 2^17
                    pwm_input += (int32)(c_mem.k_p * (pos_error >> 8)) >> 8;
                } else {
                    pwm_input += (int32)(c_mem.k_p * pos_error) >> 16;
                }
            }

            // Integral
            if (c_mem.k_i != 0) {
                pwm_input += (int32)((c_mem.k_i >> 6) * pos_error_sum[index]) >> 10;
            }

            // Derivative
            if (c_mem.k_d != 0) {
                pwm_input += (int32)(c_mem.k_d * (prev_pos[index] - g_meas.pos[index])) >> 16;
            }

            // Update measure
            prev_pos[index] = g_meas.pos[index];

            switch(index) {
                case 0:
                    if (pwm_input >= 0) {
                        direction = direction | 0x01;
                    } else {
                        direction = direction & 0xFE;
                    }
                    break;

                case 1:
                    if (pwm_input >= 0) {
                        direction = direction | 0x02;
                    } else {
                        direction = direction & 0xFD;
                    }
                    break;

                default:
                    break;
            }

            break;

        case CURR_AND_POS_CONTROL:

            if(g_mem.deflection_control) {
                defl_input[index] = g_ref.pos[index] + g_meas.pos[2];

                pos_error = defl_input[index] - g_meas.pos[index];
                pos_error_sum[index] += pos_error;
            }
            else { 
                pos_error = g_ref.pos[index] - g_meas.pos[index];
                pos_error_sum[index] += pos_error;
            } 

            pos_error = g_ref.pos[index] - g_meas.pos[index];
            pos_error_sum[index] += pos_error;

            // error_sum saturation
            if (pos_error_sum[index] > POS_INTEGRAL_SAT_LIMIT) {
                pos_error_sum[index] = POS_INTEGRAL_SAT_LIMIT;
            } else if (pos_error_sum[index] < -POS_INTEGRAL_SAT_LIMIT) {
                pos_error_sum[index] = -POS_INTEGRAL_SAT_LIMIT;
            }

            // ------ position PID control -----

            curr_ref = 0;

            // Proportional
            if (c_mem.k_p != 0) {
                if ((pos_error > 131072) || (pos_error < -131072)) {
                    curr_ref += (int32)(c_mem.k_p * (pos_error >> 8)) >> 8;
                } else {
                    curr_ref += (int32)(c_mem.k_p * pos_error) >> 16;
                }
            }

            // Integral
            if (c_mem.k_i != 0) {
                curr_ref += (int32)(c_mem.k_i * pos_error_sum[index]) >> 16;
            }

            // Derivative
            if (c_mem.k_d != 0) {
                curr_ref += (int32)(c_mem.k_d * (prev_pos[index] - g_meas.pos[index])) >> 16;
            }

            // motor direction depends on curr_ref
            switch(index) {
                case 0:
                    if (curr_ref >= 0) {
                        direction = direction | 0x01;
                    } else {
                        direction = direction & 0xFE;
                    }
                    break;

                case 1:
                    if (curr_ref >= 0) {
                        direction = direction | 0x02;
                    } else {
                        direction = direction & 0xFD;
                    }
                    break;

                default:
                    break;
            }

            // saturate max current
            if (curr_ref > c_mem.current_limit) {
                curr_ref = c_mem.current_limit;
            }
            else {
                if(curr_ref < -c_mem.current_limit)
                   curr_ref = -c_mem.current_limit;
            }

            // current error
            curr_error = curr_ref - g_meas.curr[index];

            curr_error_sum[index] += curr_error;

            if (curr_error_sum[index] > CURR_INTEGRAL_SAT_LIMIT) {
                curr_error_sum[index] = CURR_INTEGRAL_SAT_LIMIT;
            } else if (curr_error_sum[index] < -CURR_INTEGRAL_SAT_LIMIT) {
                curr_error_sum[index] = -CURR_INTEGRAL_SAT_LIMIT;
            }

            // ----- current PID control -----

            pwm_input = 0;

            // Proportional
            if (c_mem.k_p_c != 0) {
                pwm_input += (int32)(c_mem.k_p_c * curr_error) >> 16;
            }

            // Integral
            if (c_mem.k_i_c != 0) {
                pwm_input += (int32)(c_mem.k_i_c * curr_error_sum[index]) >> 16;
            }

            // Derivative
            if (c_mem.k_d_c != 0) {
                pwm_input += (int32)(c_mem.k_d_c * (prev_curr[index] - g_meas.curr[index])) >> 16;
            }

            // Update position
            prev_pos[index] = g_meas.pos[index];

            // Update current
            prev_curr[index] = g_meas.curr[index];

            break;

        case CONTROL_CURRENT:

            if(g_ref.onoff && device.tension_valid) {
                // Current ref from pos ref
                curr_ref = g_ref.pos[index] >> c_mem.res[index];

                // saturate max current
                if (curr_ref > c_mem.current_limit) {
                    curr_ref = c_mem.current_limit;
                }
                else {
                    if (curr_ref < -c_mem.current_limit)
                        curr_ref = -c_mem.current_limit;
                }


                // Current error
                curr_error = (curr_ref) - g_meas.curr[index];

                // Error sum for integral
                curr_error_sum[index] += curr_error;

                //anti wind-up
                if (curr_error_sum[index] > CURR_INTEGRAL_SAT_LIMIT) {
                    curr_error_sum[index] = CURR_INTEGRAL_SAT_LIMIT;
                } else if (curr_error_sum[index] < -CURR_INTEGRAL_SAT_LIMIT) {
                    curr_error_sum[index] = -CURR_INTEGRAL_SAT_LIMIT;
                }

                // pwm_input init
                pwm_input = 0;

                // Proportional
                if (c_mem.k_p_c != 0) {
                    pwm_input += (int32)(c_mem.k_p_c * curr_error) >> 16;
                }

                // Integral
                if (c_mem.k_i_c != 0) {
                    pwm_input += (int32)(c_mem.k_i_c * (curr_error_sum[index] >> 6)) >> 10;
                }

                // Derivative
                if (c_mem.k_d_c != 0) {
                    pwm_input += (int32)(c_mem.k_d_c * (curr_error - prev_curr_err[index])) >> 16;
                }

                // Update measure
                prev_curr_err[index] = g_meas.curr[index];

                switch(index) {
                    case 0:
                        if (curr_ref >= 0) {
                            direction = direction | 0x01;
                        } else {
                            direction = direction & 0xFE;
                        }
                        break;

                    case 1:
                        if (curr_ref >= 0) {
                            direction = direction | 0x02;
                        } else {
                            direction = direction & 0xFD;
                        }
                        break;

                    default:
                        break;
                }

                break;
            }

        case CONTROL_PWM:
            // Direct PWM value
            pwm_input = g_ref.pos[index] >> c_mem.res[index];

            switch(index) {
                case 0:
                    if (pwm_input >= 0) {
                        direction = direction | 0x01;
                    } else {
                        direction = direction & 0xFE;
                    }
                    break;

                case 1:
                    if (pwm_input >= 0) {
                        direction = direction | 0x02;
                    } else {
                        direction = direction & 0xFD;
                    }
                    break;

                default:
                    break;
            }

            break;

        default:
            break;
    }

    // abs(pwm_input) must be lower or equal to PWM_MAX_VALUE
    if(pwm_input >  PWM_MAX_VALUE) pwm_input =  PWM_MAX_VALUE;
    if(pwm_input < -PWM_MAX_VALUE) pwm_input = -PWM_MAX_VALUE;

    // remap pwm_input on pwm_limit based on input tension to have maximum 8 volts
    if (c_mem.control_mode != CONTROL_PWM) {
        pwm_input = (((pwm_input << 10) / PWM_MAX_VALUE) * device.pwm_limit) >> 10;
    }

    // drive direction and pwm duty cylce
    MOTOR_DIR_Write(direction);
    switch(index) {
        case 0:
            PWM_MOTORS_WriteCompare1(abs(pwm_input));
            pwm_sign[0] = SIGN(pwm_input);
            break;

        case 1:
            PWM_MOTORS_WriteCompare2(abs(pwm_input));
            pwm_sign[1] = SIGN(pwm_input);
            break;

        default:
            break;
    }
}

//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================

void analog_read_init(uint8 index) {

    // should I execute the function for this index?
    if(index >= NUM_OF_ANALOG_INPUTS)
        return;

    AMUX_FastSelect(index);
    ADC_StartConvert();
}


void analog_read_end(uint8 index) {

    static int32 value;

    // should I execute the function for this index?
    if(index >= NUM_OF_ANALOG_INPUTS)
        return;

    if (ADC_IsEndConversion(ADC_WAIT_FOR_RESULT)) {

        value = (int32) ADC_GetResult16();
        ADC_StopConvert();

        value -= 1638;  
        switch(index) {

            // --- Input tension ---
            case 0:
                device.tension = value * device.tension_conv_factor;
                //until there is no valid input tension repeat this measurement
                if (device.tension < 0) {
                    device.tension_valid = FALSE;
                } else {
                    device.tension_valid = TRUE;
                    pwm_limit_search();
                }
                break;

            // --- Current motor 1 ---
            case 1:
                if (device.tension_valid) {
                    g_meas.curr[0] = filter_i1(((value * 25771) >> 13) * pwm_sign[0]);
                } else {
                    g_meas.curr[0] = 0;
                }
                break;

            // --- Current motor 2 ---
            case 2:
                if (device.tension_valid) {
                    g_meas.curr[1] = filter_i2(((value * 25771) >> 13) * pwm_sign[1]);
                } else {
                    g_meas.curr[1] = 0;
                }
                break;
        }
    }
}

//==============================================================================
//                                                               ENCODER READING
//==============================================================================


void encoder_reading(uint8 index)
{
    static uint8 jj;

    static uint32 data_encoder;
    static int32 value_encoder;
    static int32 aux;

    static int32 last_value_encoder[NUM_OF_SENSORS];

    // static int32 l_value[NUM_OF_SENSORS];   //last value for vel
    // static int32 ll_value[NUM_OF_SENSORS];  //last last value for vel
    // static int32 lll_value[NUM_OF_SENSORS];  //last last last value for vel
    static int8 only_first_time = 1;

    static uint8 error[NUM_OF_SENSORS];

    if (index >= NUM_OF_SENSORS) {
        return;
    }

    if (reset_last_value_flag) {
        for (jj = 0; jj < NUM_OF_SENSORS; jj++) {
            last_value_encoder[jj] = 0;
        }
        reset_last_value_flag = 0;
    }

    //======================================================     reading sensors
    switch(index) {
        case 0:
            data_encoder = SHIFTREG_ENC_1_ReadData();
            break;

        case 1:
            data_encoder = SHIFTREG_ENC_2_ReadData();
            break;

        case 2:
            data_encoder = SHIFTREG_ENC_3_ReadData();
            break;

        case 3:
            data_encoder = SHIFTREG_ENC_4_ReadData();
            break;
    }

    data_encoder = data_encoder & 0x3FFFF;          // reset first 14 bits

    if (check_enc_data(&data_encoder)) {

        aux = data_encoder & 0x3FFC0;               // reset last 6 bit
        value_encoder = 32768 - (aux >> 2);         // shift to have 16 bit val and
                                                    // subtract half of max value and
                                                    // invert sign of sensor

        // Add offset and crop to 16bit
        value_encoder  = (int16)(value_encoder + g_mem.m_off[index]);

        // Initialize last_value_encoder
        if (only_first_time) {
            last_value_encoder[index] = value_encoder;
            if (index == NUM_OF_SENSORS - 1) {
                only_first_time = 0;
            }
        }

        // take care of rotations
        aux = value_encoder - last_value_encoder[index];

        // ====================== 1 TURN ======================
        // -32768                    0                    32767 -32768                   0                     32767
        // |-------------------------|-------------------------|-------------------------|-------------------------|
        //              |                         |      |           |      |                         |
        //           -16384                     16383    |           |   -16384                     16383
        //                                               |           |
        //                                           24575           -24576
        //                                               |___________|
        //                                                   49152

        // if we are in the right interval, take care of rotation
        // and update the variable only if the difference between
        // one measure and another is less than 1/4 of turn

        // Considering we are sampling at 1kHz, this means that our shaft needs
        // to go slower than 1/4 turn every ms -> 1 turn every 4ms
        // equal to 250 turn/s -> 15000 RPM

        if (aux > 49152) {
            g_meas.rot[index]--;
        } else  if (aux < -49152) {
            g_meas.rot[index]++;
        } else if (abs(aux) > 16384) { // if two measure are too far
            error[index]++;
            if (error[index] < 10) {
                // Discard
                return;
            }
            error[index] = 0;
        }
        error[index] = 0;


        last_value_encoder[index] = value_encoder;

        value_encoder += (int32)g_meas.rot[index] << 16;

        if (c_mem.m_mult[index] != 1.0) {
            value_encoder *= c_mem.m_mult[index];
        }

        g_meas.pos[index] = value_encoder;
    }

    // // velocity calculation
    // switch(i) {
    //     case 0: {
    //         g_meas.vel[i] = (int16)filter_vel_1((3*value_encoder + l_value[i] - ll_value[i] - 3*lll_value[i]) / 10);
    //         break;
    //     }
    //     case 1: {
    //         g_meas.vel[i] = (int16)filter_vel_2((3*value_encoder + l_value[i] - ll_value[i] - 3*lll_value[i]) / 10);
    //         break;
    //     }
    //     case 2: {
    //         g_meas.vel[i] = (int16)filter_vel_3((3*value_encoder + l_value[i] - ll_value[i] - 3*lll_value[i]) / 10);
    //         break;
    //     }
    // }

    // // update old values
    // lll_value[i] = ll_value[i];
    // ll_value[i] = l_value[i];
    // l_value[i] = value_encoder;
}


//==============================================================================
//                                                          CALIBRATION FUNCTION
//==============================================================================

void calibration()
{
    static int32 old_k_p;
    static int32 old_k_i;
    static int32 old_k_d;

    static uint8 pause_counter = 0;

    switch(calibration_flag) {
        case START:
            ISR_RS485_RX_Disable();

            // save old PID values
            old_k_p = c_mem.k_p;
            old_k_i = c_mem.k_i;
            old_k_d = c_mem.k_d;

            // goto to zero position
            g_ref.pos[0] = 0;
            g_ref.pos[1] = 0;

            // Activate motors
            if (!(g_ref.onoff & 0x03)) {
                MOTOR_ON_OFF_Write(0x03);
            }

            // wait for motors to reach zero position
            calibration_flag = PAUSE_1;
            break;

        case PAUSE_1:
            pause_counter++;

            if (pause_counter == 10) {
                pause_counter = 0;

                // set new temp values for PID parameters
                c_mem.k_p = 0.1 * 65536;
                c_mem.k_i = 0;
                c_mem.k_d = 0.3 * 65536;

                calibration_flag = CONTINUE_1;
            }
            break;

        case CONTINUE_1:
            // increment of 0.5 degree
            g_ref.pos[0] += 65536 / 720;
            g_ref.pos[1] -= 65536 / 720;

            // check if one of the motors reach the threashold
            if ((g_meas.curr[0] > CALIB_CURRENT) || (g_meas.curr[1] > CALIB_CURRENT)) {
                // save current value as MAX_STIFFNESS
                g_mem.max_stiffness = g_ref.pos[0];

                // reset old values for PID parameters
                c_mem.k_p = old_k_p;
                c_mem.k_i = old_k_i;
                c_mem.k_d = old_k_d;

                // go back to zero position
                g_ref.pos[0] = 0;
                g_ref.pos[1] = 0;

                // wait for motors to reach zero position
                calibration_flag = PAUSE_2;
            }
            break;

        case PAUSE_2:
            pause_counter++;

            if (pause_counter == 10) {
                pause_counter =0;

                calibration_flag = CONTINUE_2;
            }
            break;

        case CONTINUE_2:
            // Deactivate motors
            if (!(g_ref.onoff & 0x03)) {
                MOTOR_ON_OFF_Write(0x00);
            }

            // store memory to save MAX_STIFFNESS as default value
            memStore(DEFAULT_EEPROM_DISPLACEMENT);
            memStore(0);

            calibration_flag = STOP;

            ISR_RS485_RX_Enable();
            break;

        case STOP:
        default:
            break;
    }
}


//==============================================================================
//                                                              PWM_LIMIT_SEARCH
//==============================================================================

void pwm_limit_search() {
    uint8 index;

    if (device.tension > 25500) {
        device.pwm_limit = 0;
    } else if (device.tension < 8000) {
        device.pwm_limit = 100;
    } else {
        index = (uint8)((device.tension - 8000) / 500);
        device.pwm_limit = hitech_pwm_preload_values[index];
    }
}


/* [] END OF FILE */
