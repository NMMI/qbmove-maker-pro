// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017, Centro "E.Piaggio"
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         interruptions.c
*
* \brief        Interruption functions are in this file.
* \date         Dic. 1, 2015
* \author       qbrobotics
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/


//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>

#include "globals.h"
#include "utils.h"

//===================================================================     global


// PWM vaules needed to obtain 8 Volts given a certain input tension
// Numbers are sperimentally calculated //[index] (milliampere)
CYCODE uint8 hitech_pwm_preload_values[36] = {  100,   //0 (8000)
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
//                                                            WATCHDOG INTERRUPT
//==============================================================================

CY_ISR(ISR_WATCHDOG_Handler){

    // Set WDT flag
    
    watchdog_flag = TRUE;

}

//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt) {

    // Set RS485 flag
    
    interrupt_flag = TRUE;
     
}

//==============================================================================
//                                                             INTERRUPT MANAGER
//==============================================================================
// Manage interrupt from RS485 
//==============================================================================
// Processing RS-485 data frame:
//
// - WAIT_START:    Waits for beginning characters;
// - WAIT_ID:       Waits for ID;
// - WAIT_LENGTH:   Data length;
// - RECEIVE:       Receive all bytes;
// - UNLOAD:        Wait for another device end of transmission;
//
//==============================================================================

void interrupt_manager(){

    
    //===========================================     local variables definition

    static uint8 CYDATA state = WAIT_START;                      // state
    
    //------------------------------------------------- local data packet
    static uint8 CYDATA data_packet_index;
    static uint8 CYDATA data_packet_length;
    static uint8 data_packet_buffer[128];                     
    static uint8 CYDATA rx_queue[3];                    // last 2 bytes received
    //-------------------------------------------------

    uint8 CYDATA    rx_data;                            // RS485 UART rx data
    CYBIT           rx_data_type;                       // my id?
    uint8 CYDATA    package_count = 0;                     

    //======================================================     receive routine
    
    // Get data until buffer is not empty 
    
    while(UART_RS485_GetRxBufferSize() && (package_count < 6)){  
        // 6 stima di numero massimo di pacchetti che riesco a leggere senza bloccare l'esecuzione del firmware
        
        // Get next char
        rx_data = UART_RS485_GetChar();
        
        switch (state) {
            //-----     wait for frame start     -------------------------------
            case WAIT_START:
            
                rx_queue[0] = rx_queue[1];
                rx_queue[1] = rx_queue[2];
                rx_queue[2] = rx_data;
                
                // Check for header configuration package
                if ((rx_queue[1] == 58) && (rx_queue[2] == 58)) {
                    rx_queue[0] = 0;
                    rx_queue[1] = 0;
                    rx_queue[2] = 0;
                    state       = WAIT_ID;                    
                }else
                    if ((rx_queue[0] == 63) &&      //ASCII - ?
                        (rx_queue[1] == 13) &&      //ASCII - CR
                        (rx_queue[2] == 10))        //ASCII - LF)
                        infoGet(INFO_ALL);
                break;

            //-----     wait for id     ----------------------------------------
            case  WAIT_ID:

                // packet is for my ID or is broadcast
                if (rx_data == c_mem.id || rx_data == 0)
                    rx_data_type = FALSE;
                else                //packet is for others
                    rx_data_type = TRUE;
                
                data_packet_length = 0;
                state = WAIT_LENGTH;
                break;

            //-----     wait for length     ------------------------------------
            case  WAIT_LENGTH:

 
                data_packet_length = rx_data;
                // check validity of pack length
                if (data_packet_length <= 1) {
                    data_packet_length = 0;
                    state = WAIT_START;
                } else if (data_packet_length > 128) {
                    data_packet_length = 0;
                    state = WAIT_START;
                } else {
                    data_packet_index = 0;
                    
                    if(rx_data_type == FALSE)
                        state = RECEIVE;          // packet for me or broadcast
                    else
                        state = UNLOAD;           // packet for others
                }
                break;

            //-----     receiving     -------------------------------------------
            case RECEIVE:

                data_packet_buffer[data_packet_index] = rx_data;
                data_packet_index++;
                
                // check end of transmission
                if (data_packet_index >= data_packet_length) {
                    // verify if frame ID corresponded to the device ID
                    if (rx_data_type == FALSE) {
                        // copying data from buffer to global packet
                        memcpy(g_rx.buffer, data_packet_buffer, data_packet_length);
                        g_rx.length = data_packet_length;
                        g_rx.ready  = 1;
                        commProcess();
                    }
                    
                    data_packet_index  = 0;
                    data_packet_length = 0;
                    state              = WAIT_START;
                    package_count++;
                
                }
                break;

            //-----     other device is receving     ---------------------------
            case UNLOAD:
                if (!(--data_packet_length)) {
                    data_packet_index  = 0;
                    data_packet_length = 0;
                    RS485_CTS_Write(1);
                    RS485_CTS_Write(0);
                    state              = WAIT_START;
                    package_count++;
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
// Base frequency 1000 Hz
//==============================================================================

void function_scheduler(void) {
 
    static uint16 counter_calibration = DIV_INIT_VALUE;
    
    // Start ADC Conversion, SOC = 1

    timer_value0 = (uint32)MY_TIMER_ReadCounter();
    
    ADC_SOC_Write(0x01); 
    
    // Check Interrupt 

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
  
    //---------------------------------- Get Encoders

    encoder_reading(0); 
    
    // Check Interrupt     
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }   
    
    encoder_reading(1);
    
    // Check Interrupt 
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
    
    encoder_reading(2);
    
    // Check Interrupt 
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    //---------------------------------- Control Motors
    
    motor_control(0);

    // Check Interrupt 

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
    
    motor_control(1);
    
    // Check Interrupt 
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    //---------------------------------- Read conversion buffer - LOCK function

    analog_read_end();

    //---------------------------------- Calibration 

    // Divider 100, freq = 10 Hz
    if (!(calibration_flag == STOP)) {
        if (counter_calibration == CALIBRATION_DIV) {
            calibration();
            counter_calibration = 0;
        }
        counter_calibration++;
    }
    // Check Interrupt 
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
   
    //---------------------------------- Update States
    
    // Load k-1 state
    memcpy( &g_measOld, &g_meas, sizeof(g_meas) );
    memcpy( &g_refOld, &g_ref, sizeof(g_ref) );

    // Load k+1 state
    memcpy( &g_ref, &g_refNew, sizeof(g_ref) );

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    //CyDelayUs(100);

    timer_value = (uint32)MY_TIMER_ReadCounter();
    MY_TIMER_WriteCounter(5000000);

}


//==============================================================================
//                                                                MOTORS CONTROL
//==============================================================================

void motor_control(const uint8 idx) {
    
    uint8 CYDATA index = idx;
 
    int32 CYDATA curr_ref;

    int32 CYDATA pwm_input = 0;
    
    int32 CYDATA pos_error;
    int32 CYDATA curr_error;
    int32 CYDATA defl_input[NUM_OF_MOTORS];
    
    int32 CYDATA k_p = c_mem.k_p;  
    int32 CYDATA k_i = c_mem.k_i; 
    int32 CYDATA k_d = c_mem.k_d; 

    int32 CYDATA k_p_c = c_mem.k_p_c;  
    int32 CYDATA k_i_c = c_mem.k_i_c; 
    int32 CYDATA k_d_c = c_mem.k_d_c;   

    int32 CYDATA k_p_dl = c_mem.k_p_dl;  
    int32 CYDATA k_i_dl = c_mem.k_i_dl; 
    int32 CYDATA k_d_dl = c_mem.k_d_dl; 

    int32 CYDATA k_p_c_dl = c_mem.k_p_c_dl;  
    int32 CYDATA k_i_c_dl = c_mem.k_i_c_dl; 
    int32 CYDATA k_d_c_dl = c_mem.k_d_c_dl;  

    // Static Variables
    
    static CYBIT dirM0, dirM1;
    
    static int32 pos_error_sum[NUM_OF_MOTORS];
    static int32 curr_error_sum[NUM_OF_MOTORS];
    static int32 prev_curr_err[NUM_OF_MOTORS];

    // check index value
    if (index >= NUM_OF_MOTORS)
        return;
    // --------------- USE THIRD ENCODER AS INPUT FOR BOTH MOTORS --------------
    if (c_mem.input_mode == INPUT_MODE_ENCODER3) {

        g_ref.pos[index] = g_measOld.pos[2];

        // position limit
        if (c_mem.pos_lim_flag) {
            if (g_ref.pos[index] < c_mem.pos_lim_inf[index]) 
                g_ref.pos[index] = c_mem.pos_lim_inf[index];
            if (g_ref.pos[index] > c_mem.pos_lim_sup[index]) 
                g_ref.pos[index] = c_mem.pos_lim_sup[index];
        }
    }

    switch(c_mem.control_mode) {
        case CONTROL_ANGLE:
        case DEFLECTION_CONTROL:
        
            if(c_mem.control_mode == DEFLECTION_CONTROL){
				//for deflection control the shaft position must be summed to reference
                defl_input[index] = (g_ref.pos[index] + g_meas.pos[2]);
                pos_error = defl_input[index] - g_meas.pos[index];
            }
            else 
                // position error
                pos_error = g_ref.pos[index] - g_meas.pos[index];
           
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            
            // error sum for integral
            pos_error_sum[index] += pos_error;

            //anti wind-up
            if (pos_error_sum[index] > POS_INTEGRAL_SAT_LIMIT)
                pos_error_sum[index] = POS_INTEGRAL_SAT_LIMIT;
            else{
                if (pos_error_sum[index] < -POS_INTEGRAL_SAT_LIMIT)
                    pos_error_sum[index] = -POS_INTEGRAL_SAT_LIMIT;
            }
              
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            // pwm_input init
            pwm_input = 0;
            // Proportional
            if (k_p != 0) {
                    
                if ((pos_error > 131072) || (pos_error < -131072))  //if grater than 2^17
                    pwm_input = (int32)(k_p * (pos_error >> 8)) >> 8;
                else
                    pwm_input = (int32)(k_p * pos_error) >> 16;
            }
            
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
                      
            // Integral
            if (k_i != 0) 
                pwm_input += (int32)((k_i >> 6) * pos_error_sum[index]) >> 10;
            
            // Derivative
            if (k_d != 0)
                pwm_input += (int32)(k_d * (g_measOld.pos[index] - g_meas.pos[index])) >> 16;
         
    
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
             
            // Update measure
            //prev_pos[index] = g_meas.pos[index];

            if (index == 0){
                if (pwm_input >= 0)
                    dirM0 = 1;
                    //direction |= 0x01;
                else
                    dirM0 = 0;
                    //direction &= 0xFE;
            }else{ // index == 1
                if (pwm_input >= 0)
                    dirM1 = 1;
                   // direction |= 0x02;
                else
                    dirM1 = 0;
                    //direction &= 0xFD;
            }
            
            break;

        case CURR_AND_POS_CONTROL:
        case DEFL_CURRENT_CONTROL:

            if(c_mem.control_mode == DEFL_CURRENT_CONTROL){
                //for deflection control the shaft position must be summed to reference
                defl_input[index] = (g_ref.pos[index] + g_meas.pos[2]);
                pos_error = defl_input[index] - g_meas.pos[index];
            }
            else 
                pos_error = g_ref.pos[index] - g_meas.pos[index];

            pos_error_sum[index] += pos_error;

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // error_sum saturation
            if (pos_error_sum[index] > POS_INTEGRAL_SAT_LIMIT)
                pos_error_sum[index] = POS_INTEGRAL_SAT_LIMIT;
            else{
                if (pos_error_sum[index] < -POS_INTEGRAL_SAT_LIMIT)
                    pos_error_sum[index] = -POS_INTEGRAL_SAT_LIMIT;
            }
            // ------ position PID control -----

            curr_ref = 0;

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // Proportional
            if (k_p_dl != 0) {
                if ((pos_error > 131072) || (pos_error < -131072))
                    curr_ref += (int32)(k_p_dl * (pos_error >> 8)) >> 8;
                else
                    curr_ref += (int32)(k_p_dl * pos_error) >> 16;
            }

            // Integral
            if (k_i_dl != 0)
                curr_ref += (int32)(k_i_dl * pos_error_sum[index]) >> 16;
            
            // Derivative
            if (k_d_dl != 0)
                curr_ref += (int32)(k_d_dl * (g_measOld.pos[index] - g_meas.pos[index])) >> 16;
                        
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // saturate max current
            if (curr_ref > c_mem.current_limit)
                curr_ref = c_mem.current_limit;
            else {
                if (curr_ref < -c_mem.current_limit)
                    curr_ref = -c_mem.current_limit;
            }

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // current error and error sum
            curr_error = curr_ref - g_meas.curr[index];
            curr_error_sum[index] += curr_error;

            if (curr_error_sum[index] > CURR_INTEGRAL_SAT_LIMIT)
                curr_error_sum[index] = CURR_INTEGRAL_SAT_LIMIT;
            else{
                if (curr_error_sum[index] < -CURR_INTEGRAL_SAT_LIMIT)
                    curr_error_sum[index] = -CURR_INTEGRAL_SAT_LIMIT;
            }

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // ----- current PID control -----

            pwm_input = 0;

            // Proportional
            if (k_p_c_dl != 0)
                pwm_input += (int32)(k_p_c_dl * curr_error) >> 16;
            
            // Integral
            if (k_i_c_dl != 0)
                pwm_input += (int32)(k_i_c_dl * curr_error_sum[index]) >> 16;
            
            // Derivative
            if (k_d_c_dl != 0)
                pwm_input += (int32)(k_d_c_dl * (g_measOld.curr[index] - g_meas.curr[index])) >> 16;

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            if (index == 0){
                if (pwm_input >= 0)
                    dirM0 = 1;
                    //direction |= 0x01;
                else
                    dirM0 = 0;
                    //direction &= 0xFE;
            }else{ // index == 1
                if (pwm_input >= 0)
                    dirM1 = 1;
                   // direction |= 0x02;
                else
                    dirM1 = 0;
                    //direction &= 0xFD;
            }

            break;

        case CONTROL_CURRENT:

            // Current ref from pos ref
            curr_ref = g_ref.pos[index] >> g_mem.res[index];

            // saturate max current
            if (curr_ref > c_mem.current_limit)
                curr_ref = c_mem.current_limit;
            else {
                if(curr_ref < -c_mem.current_limit)
                   curr_ref = -c_mem.current_limit;
            }
            // Current error
            curr_error = curr_ref - g_meas.curr[index];

            // Error sum for integral
            curr_error_sum[index] += curr_error;

            //anti wind-up
            if (curr_error_sum[index] > CURR_INTEGRAL_SAT_LIMIT)
                curr_error_sum[index] = CURR_INTEGRAL_SAT_LIMIT;
            else {
                if (curr_error_sum[index] < -CURR_INTEGRAL_SAT_LIMIT)
                    curr_error_sum[index] = -CURR_INTEGRAL_SAT_LIMIT;
            }

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // pwm_input init
            pwm_input = 0;

            // Proportional
            if (k_p_c != 0)
                pwm_input += (int32)(k_p_c * curr_error) >> 16;

            // Integral
            if (k_i_c != 0)
                pwm_input += (int32)(k_i_c * (curr_error_sum[index] >> 6)) >> 10;
            
            // Derivative
            if (k_d_c != 0)
                pwm_input += (int32)(k_d_c * (curr_error - prev_curr_err[index])) >> 16;

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }

            // Update measure
            prev_curr_err[index] = curr_error;
            
            if (index == 0){
                if (pwm_input >= 0)
                    dirM0 = 1;
                    //direction |= 0x01;
                else
                    dirM0 = 0;
                    //direction &= 0xFE;
            }else{ // index == 1
                if (pwm_input >= 0)
                    dirM1 = 1;
                   // direction |= 0x02;
                else
                    dirM1 = 0;
                    //direction &= 0xFD;
            }

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            
            break;

        case CONTROL_PWM:
            // Direct PWM value
            pwm_input = g_ref.pos[index] >> g_mem.res[index];

            if (index == 0){
                if (pwm_input >= 0)
                    dirM0 = 1;
                    //direction |= 0x01;
                else
                    dirM0 = 0;
                    //direction &= 0xFE;
            }else{ // index == 1
                if (pwm_input >= 0)
                    dirM1 = 1;
                   // direction |= 0x02;
                else
                    dirM1 = 0;
                    //direction &= 0xFD;
            }

            break;

        default:
            break;
    }
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
        
    // abs(pwm_input) must be lower or equal to PWM_MAX_VALUE
    if(pwm_input >  PWM_MAX_VALUE) 
        pwm_input =  PWM_MAX_VALUE;
    if(pwm_input < -PWM_MAX_VALUE) 
        pwm_input = -PWM_MAX_VALUE;
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }   

    // remap pwm_input on pwm_limit based on input tension to have maximum 8 volts
    if (c_mem.control_mode != CONTROL_PWM)
        pwm_input = (((pwm_input << 10) / PWM_MAX_VALUE) * dev_pwm_limit) >> 10;

    // drive direction and pwm duty cycle
        
    if (dirM0){
        if (dirM1)
            MOTOR_DIR_Write(0x03);
        else
            MOTOR_DIR_Write(0x01);
    }
    else{
        if (dirM1)
            MOTOR_DIR_Write(0x02);
        else
            MOTOR_DIR_Write(0x00);
    }

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
    
    if (index == 0) {
        pwm_sign[0] = SIGN(pwm_input);
        PWM_MOTORS_WriteCompare1(abs(pwm_input));
    }
    else { // index == 1
        pwm_sign[1] = SIGN(pwm_input);
        PWM_MOTORS_WriteCompare2(abs(pwm_input));
    }
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
}

//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================

void analog_read_end() {
       
    /* =========================================================================
    /   Ideal formulation to calculate tension and current
    /
    /   tension = ((read_value_mV - offset) * 101) / gain -> [mV]
    /   current = ((read_value_mV - offset) * 375) / (gain * resistor) -> [mA]
    /
    /   Definition:
    /   read_value_mV = counts_read / 0.819 -> conversion from counts to mV
    /   offset = 2000 -> hardware amplifier bias in mV
    /   gain = 8.086 -> amplifier gain
    /   resistor = 18 -> resistor of voltage divider in KOhm 
    /
    /   Real formulation: tradeoff in good performance and accurancy, ADC_buf[] 
    /   and offset unit of measurment is counts, instead dev_tension and
    /   g_meas.curr[] are converted in properly unit.
    /  =========================================================================
    */
    
    // Wait for conversion end
    while(!ADC_STATUS_Read()){
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }
    }
    
    // Convert tension read
    dev_tension = ((int32)(ADC_buf[0] - 1638) * 1952) >> 7;
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    // Until there is no valid input tension repeat this measurement
    
    if (dev_tension > 0){
        
        // Set PWM depends on tension
        pwm_limit_search();
        
        // Check Interrupt 
    
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }

        // Filter and Set currents
        g_meas.curr[0] = filter_i1((int16) pwm_sign[0] * (((int32)(ADC_buf[1] - 1638) * 25771) >> 13));
        
        // Check Interrupt 
    
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }
        g_meas.curr[1] = filter_i2((int16) pwm_sign[1] * (((int32)(ADC_buf[2] - 1638) * 25771) >> 13));
        
        // Check Interrupt 
    
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }
    }
    else {
        g_meas.curr[0] = 0;
        g_meas.curr[1] = 0;
    }
}

//==============================================================================
//                                                               ENCODER READING
//==============================================================================

void encoder_reading(const uint8 idx)
{
    uint8 CYDATA index = idx;
    
    uint8 jj;
    
    uint32 data_encoder;
    int32 value_encoder;
    int32 aux;

    static int32 last_value_encoder[NUM_OF_SENSORS];

    static uint8 error[NUM_OF_SENSORS];
    
    static CYBIT only_first_time = TRUE;

    // static int32 l_value[NUM_OF_SENSORS];   //last value for vel
    // static int32 ll_value[NUM_OF_SENSORS];  //last last value for vel
    // static int32 lll_value[NUM_OF_SENSORS];  //last last last value for vel

    if (index >= NUM_OF_SENSORS)
        return;
    
    if (reset_last_value_flag) {
        for (jj = NUM_OF_SENSORS; jj--;) {
            last_value_encoder[jj] = 0;
        }
        reset_last_value_flag = 0;
    }

    //======================================================     reading sensors
    if (index == 0)
            data_encoder = SHIFTREG_ENC_1_ReadData() & 0x3FFFF;
    else {
        if (index == 1)
            data_encoder = SHIFTREG_ENC_2_ReadData() & 0x3FFFF;
        else {
            if (index == 2)
                data_encoder = SHIFTREG_ENC_3_ReadData() & 0x3FFFF;
            else // index == 3
                data_encoder = SHIFTREG_ENC_4_ReadData() & 0x3FFFF;
        }
    }    

    if (check_enc_data(&data_encoder)) {

        value_encoder = (int16)(32768 - ((data_encoder & 0x3FFC0) >> 2) + g_mem.m_off[index]);  

        // Initialize last_value_encoder
        if (only_first_time) {
            last_value_encoder[index] = value_encoder;
            
            if (index == (NUM_OF_SENSORS - 1))
                only_first_time = 0;
        }

        // Take care of rotations
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

        if (aux > 49152)
            g_meas.rot[index]--;
        else{ 
            if (aux < -49152)
                g_meas.rot[index]++;
            else{
                if (abs(aux) > 16384) { // if two measure are too far
                    error[index]++;
                    if (error[index] < 10)
                        // Discard
                        return;
                }
            }
        }
        
        error[index] = 0;
        
        last_value_encoder[index] = value_encoder;

        value_encoder += (int32)g_meas.rot[index] << 16;

        if (c_mem.m_mult[index] != 1.0)
            value_encoder *= c_mem.m_mult[index];
      
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
            g_refNew.pos[0] = 0;
            g_refNew.pos[1] = 0;

            // Activate motors
            if (!(g_refNew.onoff & 0x03)) {
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
            g_refNew.pos[0] += 65536 / 720;
            g_refNew.pos[1] -= 65536 / 720;

            // check if one of the motors reach the threashold
            if ((g_meas.curr[0] > CALIB_CURRENT) || (g_meas.curr[1] > CALIB_CURRENT)) {
                // save current value as MAX_STIFFNESS
                g_mem.max_stiffness = g_ref.pos[0];

                // reset old values for PID parameters
                c_mem.k_p = old_k_p;
                c_mem.k_i = old_k_i;
                c_mem.k_d = old_k_d;

                // go back to zero position
                g_refNew.pos[0] = 0;
                g_refNew.pos[1] = 0;

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
            if (!(g_refNew.onoff & 0x03)) {
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
    
    uint8 CYDATA index;

    if (dev_tension > 25500)    // Max value 
        dev_pwm_limit = 0;
    else{ 
        if (dev_tension < 8000) // Min value
            dev_pwm_limit = 100;
        else {                  
            index = (uint8)((dev_tension - 8000) >> 9);
            dev_pwm_limit = hitech_pwm_preload_values[index];
        }
    }
}


/* [] END OF FILE */
