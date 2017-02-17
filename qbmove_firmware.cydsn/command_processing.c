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
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         Dic. 1, 2015
* \author       qbrobotics
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include <stdio.h>
#include <interruptions.h>
#include <utils.h>

#include "commands.h"

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(){
    
    uint8 CYDATA rx_cmd;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }

    switch(rx_cmd){
//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            cmd_get_measurements();
            break;
            
//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            cmd_get_inputs();
            break;
//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            cmd_get_currents();
            break;

//====================================================     CMD_GET_CURR_AND_MEAS

        case CMD_GET_CURR_AND_MEAS:
            cmd_get_curr_and_meas();
            break;
            
//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            cmd_set_inputs();
            break;
            
//========================================================     CMD_SET_POS_STIFF

        case CMD_SET_POS_STIFF:
            cmd_set_pos_stiff();
            break;
            
//=======================================================     CMD_GET_VELOCITIES

        case CMD_GET_VELOCITIES:
            cmd_get_velocities();
            break;

//=============================================================     CMD_ACTIVATE
        case CMD_ACTIVATE:
            cmd_activate();
            break;
            
//=============================================================     CMD_WATCHDOG
            
        case CMD_SET_WATCHDOG:
            cmd_set_watchdog();
            break;
            
//=========================================================     CMD_GET_ACTIVATE
            
        case CMD_GET_ACTIVATE:
            cmd_get_activate();
            break;
            
//=========================================================     CMD_SET_BAUDRATE
            
        case CMD_SET_BAUDRATE:
            cmd_set_baudrate();
            break;  
            
//=============================================================     CMD_GET_INFO
            
        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]));
            break;

//============================================================     CMD_SET_PARAM
            
        case CMD_SET_ZEROS:
            setZeros();
            break;

//============================================================     CMD_GET_PARAM
            
        case CMD_GET_PARAM_LIST:
            get_param_list( *((uint16 *) &g_rx.buffer[1]));
            break;

//=================================================================     CMD_PING
            
        case CMD_PING:
            cmd_ping();
            break;

//=========================================================     CMD_STORE_PARAMS
            
        case CMD_STORE_PARAMS:
            cmd_store_params();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS
            
        case CMD_STORE_DEFAULT_PARAMS:
            if ( memStore(DEFAULT_EEPROM_DISPLACEMENT) ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            if ( memRestore() ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
            break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if ( memInit() ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
            break;

//===========================================================     CMD_BOOTLOADER
        case CMD_BOOTLOADER:
            sendAcknowledgment(ACK_OK);
            CyDelay(1000);
            FTDI_ENABLE_REG_Write(0x00);
            CyDelay(1000);
            Bootloadable_Load();
            break;

//============================================================     CMD_CALIBRATE
        case CMD_CALIBRATE:
            calibration_flag = START;
            sendAcknowledgment(ACK_OK);
            break;

//=========================================================== ALL OTHER COMMANDS
        default:
            break;
    }
}

//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type){
    unsigned char packet_string[1100];

//======================================     choose info type and prepare string

    switch (info_type) {
        case INFO_ALL:
            infoPrepare(packet_string);
            UART_RS485_PutString(packet_string);
            break;
        default:
            break;
    }
}

//==============================================================================
//                                                                     SET ZEROS
//==============================================================================

void setZeros()
{
    uint8 CYDATA i;        // iterator

    for(i = 0; i < NUM_OF_SENSORS; ++i) {
        g_mem.m_off[i] = *((int16 *) &g_rx.buffer[1 + i * 2]);
        g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

        g_meas.rot[i] = 0;
    }
    reset_last_value_flag = 1;

    sendAcknowledgment(ACK_OK);
}

//==============================================================================
//                                                            GET PARAMETER LIST
//==============================================================================

void get_param_list(uint16 index)
{
    //Package to be sent variables
    uint8 packet_data[1201] = "";
    uint16 packet_lenght = 1201;

    //Auxiliary variables
    uint16 CYDATA i;
    uint8 string_lenght;
    int32 aux_int;

    //Parameters menu string definitions
    char id_str[15]             = "1 - Device ID:";
    char pos_pid_str[28]        = "2 - Position PID [P, I, D]:";
    char curr_pid_str[27]       = "3 - Current PID [P, I, D]:";
    char startup_str[28]        = "4 - Startup Activation:";
    char input_str[27]          = "5 - Input mode:";
    char contr_str[39]          = "6 - Control mode:";
    char res_str[17]            = "7 - Resolutions:";
    char m_off_str[25]          = "8 - Measurement Offsets:";
    char mult_str[17]           = "9 - Multipliers:";
    char pos_lim_flag_str[28]   = "10 - Pos. limit active:";
    char pos_lim_str[29]        = "11 - Pos. limits [inf, sup]:";
    char max_step_str[27]       = "12 - Max steps [neg, pos]:";
    char curr_limit_str[20]     = "13 - Current limit:";

    //Parameters menus
    char input_mode_menu[52] = "0 -> Usb\n1 -> Shaft's position controls the motors\n";
    char control_mode_menu[99] = "0 -> Position\n1 -> PWM\n2 -> Current\n3 -> Position-Current\n4 -> Deflection\n5 -> Deflection-Current\n";
    char yes_no_menu[42] = "0 -> Deactivate [NO]\n1 -> Activate [YES]\n";

    //Strings lenghts
    uint8 CYDATA id_str_len = strlen(id_str);
    uint8 CYDATA pos_pid_str_len = strlen(pos_pid_str);
    uint8 CYDATA curr_pid_str_len = strlen(curr_pid_str);

    uint8 CYDATA res_str_len = strlen(res_str);
    uint8 CYDATA m_off_str_len = strlen(m_off_str);
    uint8 CYDATA mult_str_len = strlen(mult_str);

    uint8 CYDATA pos_lim_str_len = strlen(pos_lim_str);
    uint8 CYDATA curr_limit_str_len = strlen(curr_limit_str);

    uint8 CYDATA input_mode_menu_len = strlen(input_mode_menu);
    uint8 CYDATA control_mode_menu_len = strlen(control_mode_menu);
    uint8 CYDATA yes_no_menu_len = strlen(input_mode_menu);

    packet_data[0] = CMD_GET_PARAM_LIST;
    packet_data[1] = NUM_OF_PARAMS;

    switch(index) {
        case 0:         //List of all parameters with relative types
            /*-----------------ID-----------------*/

            packet_data[2] = TYPE_UINT8;
            packet_data[3] = 1;
            packet_data[4] = c_mem.id;
            for(i = id_str_len; i != 0; i--)
                packet_data[5 + id_str_len - i] = id_str[id_str_len - i];

            /*-------------POSITION PID-----------*/

            packet_data[52] = TYPE_FLOAT;
            packet_data[53] = 3;
            if(c_mem.control_mode != CURR_AND_POS_CONTROL && c_mem.control_mode != DEFL_CURRENT_CONTROL) {
                *((float *) (packet_data + 54)) = (float) c_mem.k_p / 65536;
                *((float *) (packet_data + 58)) = (float) c_mem.k_i / 65536;
                *((float *) (packet_data + 62)) = (float) c_mem.k_d / 65536;
            }
            else {
                *((float *) (packet_data + 54)) = (float) c_mem.k_p_dl / 65536;
                *((float *) (packet_data + 58)) = (float) c_mem.k_i_dl / 65536;
                *((float *) (packet_data + 62)) = (float) c_mem.k_d_dl / 65536;
            }
            for(i = pos_pid_str_len; i != 0; i--)
                packet_data[66 + pos_pid_str_len - i] = pos_pid_str[pos_pid_str_len - i];

            /*--------------CURRENT PID-----------*/

            packet_data[102] = TYPE_FLOAT;
            packet_data[103] = 3;
            if(c_mem.control_mode != CURR_AND_POS_CONTROL && c_mem.control_mode != DEFL_CURRENT_CONTROL) {
                *((float *) (packet_data + 104)) = (float) c_mem.k_p_c / 65536;
                *((float *) (packet_data + 108)) = (float) c_mem.k_i_c / 65536;
                *((float *) (packet_data + 112)) = (float) c_mem.k_d_c / 65536;
            }
            else {
                *((float *) (packet_data + 104)) = (float) c_mem.k_p_c_dl / 65536;
                *((float *) (packet_data + 108)) = (float) c_mem.k_i_c_dl / 65536;
                *((float *) (packet_data + 112)) = (float) c_mem.k_d_c_dl / 65536;
            }
            for(i = curr_pid_str_len; i != 0; i--)
                packet_data[116 + curr_pid_str_len - i] = curr_pid_str[curr_pid_str_len - i];

            /*----------STARTUP ACTIVATION--------*/

            packet_data[152] = TYPE_FLAG;
            packet_data[153] = 1;
            packet_data[154] = c_mem.activ;
            if(c_mem.activ) {
                strcat(startup_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(startup_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[155 + string_lenght - i] = startup_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[155 + string_lenght]  = 3;

            /*--------------INPUT MODE------------*/
            
            packet_data[202] = TYPE_FLAG;
            packet_data[203] = 1;
            packet_data[204] = c_mem.input_mode;
            switch(c_mem.input_mode) {
                case INPUT_MODE_EXTERNAL:
                    strcat(input_str, " Usb\0");
                    string_lenght = 20;
                break;
                case INPUT_MODE_ENCODER3:
                    strcat(input_str, " Encoder 3\0");
                    string_lenght = 26;
                break;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[205 + string_lenght - i] = input_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[205 + string_lenght] = 1;
            
            /*-------------CONTROL MODE-----------*/
            
            packet_data[252] = TYPE_FLAG;
            packet_data[253] = 1;
            packet_data[254] = c_mem.control_mode;
            switch(c_mem.control_mode){
                case CONTROL_ANGLE:
                    strcat(contr_str, " Position\0");
                    string_lenght = 27;
                break;
                case CONTROL_PWM:
                    strcat(contr_str, " PWM\0");
                    string_lenght = 22;
                break;
                case CONTROL_CURRENT:
                    strcat(contr_str, " Current\0");
                    string_lenght = 26;
                break;
                case CURR_AND_POS_CONTROL:
                    strcat(contr_str, " Position and current\0");
                    string_lenght = 39;
                break;
                case DEFLECTION_CONTROL:
                    strcat(contr_str, " Deflection\0");
                    string_lenght = 29;
                break;
                case DEFL_CURRENT_CONTROL:
                    strcat(contr_str, " Deflection and current\0");
                    string_lenght = 41;
                break;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[255 + string_lenght - i] = contr_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[255 + string_lenght] = 2;
            
            /*-------------RESOLUTIONS------------*/
            
            packet_data[302] = TYPE_UINT8;
            packet_data[303] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                packet_data[i + 304] = c_mem.res[i];
            for(i = res_str_len; i != 0; i--)
                packet_data[307 + res_str_len - i] = res_str[res_str_len - i];
            
            /*----------MEASUREMENT OFFSET--------*/
            
            packet_data[352] = TYPE_INT16;
            packet_data[353] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++) 
                *((int16 *) ( packet_data + 354 + (i * 2) )) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);
            for(i = m_off_str_len; i != 0; i--)
                packet_data[360 + m_off_str_len - i] = m_off_str[m_off_str_len - i];
            
            /*------------MULTIPLIERS-------------*/
            
            packet_data[402] = TYPE_FLOAT;
            packet_data[403] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                *((float *) ( packet_data + 404 + (i * 4) )) = c_mem.m_mult[i];
            for(i = 0; i < strlen(mult_str); i++)
                packet_data[416 + i] = mult_str[i];

            /*-----------POS LIMIT FLAG-----------*/
            
            packet_data[452] = TYPE_FLAG;
            packet_data[453] = 1;
            packet_data[454] = c_mem.pos_lim_flag;
            if(c_mem.pos_lim_flag) {
                strcat(pos_lim_flag_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(pos_lim_flag_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[455 + string_lenght - i] = pos_lim_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[455 + string_lenght] = 3;
            
            /*-----------POSITION LIMITS----------*/
            
            packet_data[502] = TYPE_INT32;
            packet_data[503] = 4;
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                *((int32 *)( packet_data + 504 + (i * 2 * 4) )) = (c_mem.pos_lim_inf[i] >> c_mem.res[i]);
                *((int32 *)( packet_data + 504 + (i * 2 * 4) + 4)) = (c_mem.pos_lim_sup[i] >> c_mem.res[i]);
            }
            for(i = pos_lim_str_len; i != 0; i--)
                packet_data[520 + pos_lim_str_len - i] = pos_lim_str[pos_lim_str_len - i];

            /*--------------MAX STEPS-------------*/
            
            packet_data[552] = TYPE_INT32;
            packet_data[553] = 2;
            *((int32 *)(packet_data + 554)) = c_mem.max_step_neg;
            *((int32 *)(packet_data + 558)) = c_mem.max_step_pos;
            for(i = 0; i < strlen(max_step_str); i++)
                packet_data[562 + i] = max_step_str[i];

            /*------------CURRENT LIMIT-----------*/

            packet_data[602] = TYPE_INT16;
            packet_data[603] = 1;
            *((int16 *)(packet_data + 604)) = c_mem.current_limit;
            for(i = curr_limit_str_len; i != 0; i--)
                packet_data[606 + curr_limit_str_len - i] = curr_limit_str[curr_limit_str_len - i];

            /*------------PARAMETERS MENU-----------*/

            for(i = input_mode_menu_len; i != 0; i--)
                packet_data[652 + input_mode_menu_len - i] = input_mode_menu[input_mode_menu_len - i];

            for(i = control_mode_menu_len; i != 0; i--)
                packet_data[802 + control_mode_menu_len - i] = control_mode_menu[control_mode_menu_len - i];

            for(i = yes_no_menu_len; i!= 0; i--)
                packet_data[952 + yes_no_menu_len - i] = yes_no_menu[yes_no_menu_len - i];

            packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);
            UART_RS485_ClearTxBuffer();
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;
        
//=======================================================     set_pid_parameters
        case 2:         //Position PID - float[3]
            if(c_mem.control_mode != CURR_AND_POS_CONTROL && c_mem.control_mode != DEFL_CURRENT_CONTROL) {
                g_mem.k_p = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            else {
                g_mem.k_p_dl = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_dl = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_dl = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
        break;

//==================================================     set_curr_pid_parameters
        case 3:         //Current PID - float[3]
            if(c_mem.control_mode != CURR_AND_POS_CONTROL && c_mem.control_mode != DEFL_CURRENT_CONTROL) {              
                g_mem.k_p_c = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_c = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_c = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            else {
                g_mem.k_p_c_dl = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_c_dl = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_c_dl = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
        break;

//===================================================     set_startup_activation        
        case 4:         //Startup flag - uint8
            g_mem.activ = g_rx.buffer[3];
        break;

//===========================================================     set_input_mode        
        case 5:         //Input mode - uint8
            g_mem.input_mode = g_rx.buffer[3];
        break;
        
//=========================================================     set_control_mode
        case 6:         //Control mode - uint8
            g_mem.control_mode = g_rx.buffer[3];
        break;
        
//===========================================================     set_resolution
        case 7:         //Resolution - uint8[3]
            for (i =0; i < NUM_OF_SENSORS; i++)
                g_mem.res[i] = g_rx.buffer[i+3];
        break;
        
//===============================================================     set_offset
        case 8:         //Measurement Offset - int32[3] 
            for(i = 0; i < NUM_OF_SENSORS; ++i) {
                g_mem.m_off[i] = *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            reset_last_value_flag = 1;
        break;
        
//===========================================================     set_multiplier
        case 9:         //Multipliers - float[3]
            for(i = 0; i < NUM_OF_SENSORS; ++i)
                g_mem.m_mult[i] = *((float *) &g_rx.buffer[3 + i * 4]);
        break;
        
//=====================================================     set_pos_limit_enable
        case 10:        //Position limit flag - uint8
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
        break;

//============================================================     set_pos_limit
        case 11:        //Position limits - int32[4]
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                g_mem.pos_lim_inf[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4)]);
                g_mem.pos_lim_sup[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4) + 4]);

                g_mem.pos_lim_inf[i] = g_mem.pos_lim_inf[i] << g_mem.res[i];
                g_mem.pos_lim_sup[i] = g_mem.pos_lim_sup[i] << g_mem.res[i];
            }
        break;

//==================================================     set_max_steps_per_cycle
        case 12:        //Max steps - int32[2]
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int <= 0)
                g_mem.max_step_neg = aux_int;

            aux_int = *((int32 *) &g_rx.buffer[3 + 4]);
            if (aux_int >= 0) 
                g_mem.max_step_pos = aux_int;
        break;
        
//========================================================     set_current_limit
        case 13:        //Current limit - int16
            g_mem.current_limit = *((int16*) &g_rx.buffer[3]);
        break;
    }
}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int CYDATA i;

    unsigned char str[50];
    
    strcpy(info_string, "");
    strcat(info_string, "\r\n");
    strcat(info_string, "Firmware version: ");
    strcat(info_string, VERSION);
    strcat(info_string, ".\r\n\r\n");

    strcat(info_string,"DEVICE INFO\r\n");
    sprintf(str,"ID: %d\r\n",(int) c_mem.id);
    strcat(info_string,str);
    sprintf(str,"Number of sensors: %d\r\n",(int) NUM_OF_SENSORS);
    strcat(info_string,str);
    sprintf(str,"PWM Limit: %d\r\n",(int) dev_pwm_limit);
    strcat(info_string,str);
    strcat(info_string,"\r\n");

    strcat(info_string, "MOTOR INFO\r\n");
    strcat(info_string, "Motor references: ");
    
    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "%d ", (int)(g_refOld.pos[i] >> c_mem.res[i]));
        strcat(info_string,str);
    }
    strcat(info_string,"\r\n");

    sprintf(str, "Motor enabled: ");
    
    if (g_refOld.onoff & 0x03) {
        strcat(str,"YES\r\n");
    } else {
        strcat(str,"NO\r\n");
    }
    strcat(info_string, str);


    strcat(info_string,"\r\nMEASUREMENTS INFO\r\n");
    strcat(info_string, "Sensor value:\r\n");
    for (i = 0; i < NUM_OF_SENSORS; i++) {
        sprintf(str,"%d -> %d", i+1,
            (int)(g_measOld.pos[i] >> c_mem.res[i]));
        strcat(info_string, str);
        strcat(info_string, "\r\n");
    }
    sprintf(str,"Voltage (mV): %ld", (int32) dev_tension );
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str,"Current 1 (mA): %ld", (int32) g_measOld.curr[0] );
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str,"Current 2 (mA): %ld", (int32) g_measOld.curr[1] );
    strcat(info_string, str);
    strcat(info_string,"\r\n");


    strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");

    strcat(info_string, "PID Controller:\r\n");
    if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
        sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p / 65536));
        strcat(info_string, str);
        sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i / 65536));
        strcat(info_string, str);
        sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d / 65536));
        strcat(info_string, str);
    }
    else {
        sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p_dl / 65536));
        strcat(info_string, str);
        sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i_dl / 65536));
        strcat(info_string, str);
        sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d_dl / 65536));
        strcat(info_string, str);
    }

    strcat(info_string, "Current PID Controller:\r\n");
    if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
        sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p_c / 65536));
        strcat(info_string, str);
        sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i_c / 65536));
        strcat(info_string, str);
        sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d_c / 65536));
        strcat(info_string, str);
    }
    else {
        sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p_c_dl / 65536));
        strcat(info_string, str);
        sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i_c_dl / 65536));
        strcat(info_string, str);
        sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d_c_dl / 65536));
        strcat(info_string, str);
    }

    strcat(info_string,"\r\n");

    if (c_mem.activ == 0x03) {
        strcat(info_string, "Startup activation: YES\r\n");
    } else {
        strcat(info_string, "Startup activation: NO\r\n");
    }

    switch(c_mem.input_mode) {
        case 0:
            strcat(info_string, "Input mode: USB\r\n");
        break;
        case 1:
            strcat(info_string, "Input mode: Sensor 3\r\n");
        break;
    }
    
    strcat(info_string, "Control Mode: ");
    switch(c_mem.control_mode) {
        case CONTROL_ANGLE:
            strcat(info_string, "Position\r\n");
        break;
        case CONTROL_PWM:
            strcat(info_string, "PWM\r\n");
            break;
        case CONTROL_CURRENT:
            strcat(info_string, "Current\r\n");
            break;
        case CURR_AND_POS_CONTROL:
            strcat(info_string, "Position and current\r\n");
            break;
        case DEFLECTION_CONTROL: 
            strcat(info_string, "Deflection\r\n");
            break;
        case DEFL_CURRENT_CONTROL:
            strcat(info_string, "Deflection and current\r\n");
            break;
    }

    strcat(info_string, "Sensor resolution:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %d", (int) (i + 1),
            (int) c_mem.res[i]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
    }


    strcat(info_string, "Measurement Offset:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %ld", (int) (i + 1),
            (int32) c_mem.m_off[i] >> c_mem.res[i]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
    }

    strcat(info_string, "Measurement Multiplier:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %f", (int)(i + 1),
            (double) c_mem.m_mult[i]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
    }

    sprintf(str, "Position limit active: %d", (int)g_mem.pos_lim_flag);
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "Position limit motor %d: inf -> %ld  ", (int)(i + 1),
                (int32)g_mem.pos_lim_inf[i] >> g_mem.res[i]);
        strcat(info_string, str);

        sprintf(str, "sup -> %ld\r\n",
                (int32)g_mem.pos_lim_sup[i] >> g_mem.res[i]);
        strcat(info_string, str);
    }

    sprintf(str, "Max stiffness: %d", (int)g_mem.max_stiffness >> g_mem.res[0]);
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str, "Current limit: %d", (int)g_mem.current_limit);
    strcat(info_string, str);
    strcat(info_string,"\r\n");
    
    sprintf(str, "debug: %ld", (uint32) timer_value0 - (uint32) timer_value);
    strcat(info_string, str);
    strcat(info_string, "\r\n");
}

//==============================================================================
//                                                     WRITE FUNCTIONS FOR RS485
//==============================================================================

void commWrite_old_id(uint8 *packet_data, uint16 packet_lenght, uint8 old_id)
{
    uint16 CYDATA index;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    //if(old_id)
        UART_RS485_PutChar(old_id);
    //else
        //UART_RS485_PutChar(g_mem.id);
        
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index) {
        UART_RS485_PutChar(packet_data[index]);
    }

    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}

void commWrite(uint8 *packet_data,const uint16 packet_lenght)
{
    uint16 CYDATA index;

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    
    // frame - ID
    UART_RS485_PutChar(g_mem.id);
    
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index)
        UART_RS485_PutChar(packet_data[index]);
    
    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}

//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment(const uint8 value) {

    // Packet: header + crc
    
    uint8 CYDATA packet_data[2];

    // Header
    packet_data[0] = value;
    
    // Payload/CRC
    packet_data[1] = value;

    commWrite(packet_data, 2);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================
/**
* This function stores current memory settings on the eeprom with the specified
* displacement
**/

uint8 memStore(int displacement) {

    uint8 writeStatus;
    int i;
    int pages;
    uint8 ret_val = 1;

    PWM_MOTORS_WriteCompare1(0);
    PWM_MOTORS_WriteCompare2(0);

    // Retrieve temperature for better writing performance
    EEPROM_UpdateTemperature();

    memcpy( &c_mem, &g_mem, sizeof(g_mem) );

    pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);

    for(i = 0; i < pages; ++i) {
        writeStatus = EEPROM_Write(&g_mem.flag + 16 * i, i + displacement);
        if(writeStatus != CYRET_SUCCESS) {
            ret_val = 0;
            break;
        }
    }

    memcpy( &g_mem, &c_mem, sizeof(g_mem) );

    return ret_val;
}


//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================
/**
* This function loads user settings from the eeprom.
**/

void memRecall(void) {

    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) 
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
   

    //check for initialization
    if (g_mem.flag == FALSE) 
        memRestore();
    else 
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================
/**
* This function loads default settings from the eeprom.
**/

uint8 memRestore(void) {

    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) 
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    

    //check for initialization
    if (g_mem.flag == FALSE) 
        return memInit();
     else 
        return memStore(0);
   
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================
/**
* This function initialize memory when eeprom is compromised.
**/

uint8 memInit(void) {

    uint8 CYDATA i;
    //initialize memory settings
    g_mem.id                =   1;
    g_mem.k_p               =   0.1 * 65536;
    g_mem.k_i               =   0 * 65536;
    g_mem.k_d               =   0.8 * 65536;
    g_mem.k_p_c             =   5 * 65536;
    g_mem.k_i_c             =   0 * 65536;
    g_mem.k_d_c             =   0 * 65536;

    g_mem.k_p_dl            =   0 * 65536;
    g_mem.k_i_dl            =   0 * 65536;
    g_mem.k_d_dl            =   0 * 65536;
    g_mem.k_p_c_dl          =   0 * 65536;
    g_mem.k_i_c_dl          =   0 * 65536;
    g_mem.k_d_c_dl          =   0 * 65536;
    
    g_mem.activ             =   0;
    g_mem.input_mode        =   0;
    g_mem.control_mode      =   0;
    g_mem.watchdog_period   =   0; //MAX_WATCHDOG_TIMER;

    g_mem.pos_lim_flag = 1;

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        g_mem.pos_lim_inf[i] = -30000;
        g_mem.pos_lim_sup[i] =  30000;
    }

    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        g_mem.m_mult[i] = 1;
        g_mem.res[i] = 1;
    }

    g_mem.m_off[0] = (int32)0 << g_mem.res[0];
    g_mem.m_off[1] = (int32)0 << g_mem.res[1];
    g_mem.m_off[2] = (int32)0 << g_mem.res[2];

    g_mem.max_stiffness = (int32)3000 << g_mem.res[0];

    g_mem.current_limit = DEFAULT_CURRENT_LIMIT;

    //set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;

    //write that configuration to EEPROM
    return ( memStore(0) && memStore(DEFAULT_EEPROM_DISPLACEMENT) );
}

//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================
/**
* Bunch of functions used on request from UART communication
**/

void cmd_get_measurements(){
   
    
    uint8 CYDATA index;
   
    // Packet: header + measure(int16) + crc
    
    #if (NUM_OF_SENSORS == 4)
        uint8 packet_data[10];
    #endif
    #if  (NUM_OF_SENSORS == 3)
        uint8 packet_data[8]; 
    #endif

    //Header package
    packet_data[0] = CMD_GET_MEASUREMENTS;   
    
    for (index = NUM_OF_SENSORS; index--;) 
        *((int16 *) &packet_data[(index << 1) + 1]) = (int16)(g_measOld.pos[index] >> g_mem.res[index]);
            
    // Calculate Checksum and send message to UART 
        
    #if (NUM_OF_SENSORS == 4)
        packet_data[9] = LCRChecksum (packet_data, 9);
        commWrite(packet_data, 10);
    #endif
    #if  (NUM_OF_SENSORS == 3)
        packet_data[7] = LCRChecksum (packet_data, 7);
        commWrite(packet_data, 8);
    #endif
    
}

void cmd_get_inputs(){

    // Packet: header + motor_measure(int16) + crc

    uint8 packet_data[6]; 
    
    //Header package

    packet_data[0] = CMD_GET_INPUTS;
    
    *((int16 *) &packet_data[1]) = (int16) (g_refOld.pos[0]  >> g_mem.res[0]);
    *((int16 *) &packet_data[3]) = (int16) (g_refOld.pos[1]  >> g_mem.res[1]);
    
    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum(packet_data, 5);

    commWrite(packet_data, 6);

}

void cmd_get_currents(){
    
    // Packet: header + motor_measure(int16) + crc
    
    uint8 packet_data[6]; 
    
    //Header package

    packet_data[0] = CMD_GET_CURRENTS;

    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr[0];
    *((int16 *) &packet_data[3]) = (int16) g_measOld.curr[1];

    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum (packet_data, 5);

    commWrite(packet_data, 6);
}

void cmd_get_curr_and_meas(){
    
    uint8 CYDATA index;
   
    //Packet: header + curr_meas(int16) + pos_meas(int16) + CRC
    
    #if (NUM_OF_SENSORS == 4)
        uint8 packet_data[14];
    #endif
    #if  (NUM_OF_SENSORS == 3)
        uint8 packet_data[12]; 
    #endif

    //Header package
    packet_data[0] = CMD_GET_CURR_AND_MEAS;
    
    // Currents
    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr[0];
    *((int16 *) &packet_data[3]) = (int16) g_measOld.curr[1];

    // Positions
    for (index = NUM_OF_SENSORS; index--;) 
        *((int16 *) &packet_data[(index << 2) + 5]) = (int16) (g_measOld.pos[index] >> g_mem.res[index]);
        
    // Calculate Checksum and send message to UART 
        
    #if (NUM_OF_SENSORS == 4)
        packet_data[13] = LCRChecksum (packet_data, 13);
        commWrite(packet_data, 14);
    #endif
    #if  (NUM_OF_SENSORS == 3)
        packet_data[11] = LCRChecksum (packet_data, 11);
        commWrite(packet_data, 12);
    #endif
}

void cmd_set_inputs(){
    
    // Store position setted in right variables
    g_refNew.pos[0] = *((int16 *) &g_rx.buffer[1]);   // motor 1
    g_refNew.pos[0] = g_refNew.pos[0] << g_mem.res[0];

    g_refNew.pos[1] = *((int16 *) &g_rx.buffer[3]);   // motor 2
    g_refNew.pos[1] = g_refNew.pos[1] << g_mem.res[1];

    // Check Position Limit cmd
    if (c_mem.pos_lim_flag) {                      
        
        if (g_refNew.pos[0] < c_mem.pos_lim_inf[0]) 
            g_refNew.pos[0] = c_mem.pos_lim_inf[0];
        if (g_refNew.pos[1] < c_mem.pos_lim_inf[1]) 
            g_refNew.pos[1] = c_mem.pos_lim_inf[1];

        if (g_refNew.pos[0] > c_mem.pos_lim_sup[0]) 
            g_refNew.pos[0] = c_mem.pos_lim_sup[0];
        if (g_refNew.pos[1] > c_mem.pos_lim_sup[1]) 
            g_refNew.pos[1] = c_mem.pos_lim_sup[1];
    }
}

void cmd_set_pos_stiff(){
    
    int32 CYDATA pos, stiff;
    
    // Load position e stiffness

    pos = *((int16 *) &g_rx.buffer[1]);      // equilibrium position
    stiff = *((int16 *) &g_rx.buffer[3]);    // stiffness

    // Convert position in ticks
    pos = pos << g_mem.res[0];

    // Check position limits
    if (pos > (c_mem.pos_lim_sup[0] - c_mem.max_stiffness))
        pos = c_mem.pos_lim_sup[0] - c_mem.max_stiffness;

    if (pos < (c_mem.pos_lim_inf[0] + c_mem.max_stiffness))
        pos = c_mem.pos_lim_inf[0] + c_mem.max_stiffness;

    // Stiffness is intended between -32768 and 32767
    // remap  stiff value between -MAX_STIFFNESS and MAX_STIFFNESS
        
    stiff = (int32)(((float) c_mem.max_stiffness / 32768.0) * stiff);

    // Store pos/stiff rule
    g_refNew.pos[0] = pos + stiff;
    g_refNew.pos[1] = pos - stiff;
}

void cmd_get_velocities(){
    
    uint8 CYDATA index;
    
    // Packet: header + measure(int16) + crc

    #if (NUM_OF_SENSORS == 4)
        uint8 packet_data[10];
    #endif
    #if (NUM_OF_SENSORS == 3)
        uint8 packet_data[8]; 
    #endif
    
    //Header package
  
    packet_data[0] = CMD_GET_VELOCITIES;   
   
    for (index = NUM_OF_SENSORS; index--;)
        *((int16 *) &packet_data[(index << 2) + 1]) = (int16)(g_measOld.vel[index]);

    // Calculate Checksum and send message to UART 

    #if (NUM_OF_SENSORS == 4)
        packet_data[9] = LCRChecksum (packet_data, 9);
        commWrite(packet_data, 10);
    #endif
    #if  (NUM_OF_SENSORS == 3)
        packet_data[7] = LCRChecksum (packet_data, 7);
        commWrite(packet_data, 8);
    #endif
}

void cmd_activate(){
    
    // Store new value reads
    g_refNew.onoff = g_rx.buffer[1];
    
    // Check type of control mode enabled
    if (g_mem.control_mode == CONTROL_ANGLE || g_mem.control_mode == CURR_AND_POS_CONTROL
     || g_mem.control_mode == DEFLECTION_CONTROL || g_mem.control_mode == DEFL_CURRENT_CONTROL) {
        g_refNew.pos[0] = g_meas.pos[0];
        g_refNew.pos[1] = g_meas.pos[1];
    }
    
    // Safety start position (eliminate "lost rotation" problem)    
    if((g_meas.pos[0] > 26000) || (g_meas.pos[0] < -26000) || 
       (g_meas.pos[1] > 26000) || (g_meas.pos[1] < -26000) ||
       (g_meas.pos[2] > 26000) || (g_meas.pos[2] < -26000))
        g_refNew.onoff = 0x00;
    
    // Activate/Disactivate motors
    MOTOR_ON_OFF_Write(g_refNew.onoff);

}

void cmd_set_watchdog(){
      
    if (g_rx.buffer[1] <= 0){
        // Deactivate Watchdog
        WATCHDOG_ENABLER_Write(1); 
        g_mem.watchdog_period = 0;   
    }
    else{
        // Activate Watchdog        
        if (g_rx.buffer[1] > MAX_WATCHDOG_TIMER)
            g_rx.buffer[1] = MAX_WATCHDOG_TIMER;
            
        // Period * Time_CLK = WDT
        // Period = WTD / Time_CLK =     (WTD    )  / ( ( 1 / Freq_CLK ) )
        // Set request watchdog period - (WTD * 2)  * (250 / 1024        )
        g_mem.watchdog_period = (uint8) (((uint32) g_rx.buffer[1] * 2 * 250 ) >> 10);   
        WATCHDOG_COUNTER_WritePeriod(g_mem.watchdog_period); 
        WATCHDOG_ENABLER_Write(0); 
    }
}

void cmd_get_activate(){
    
    uint8 packet_data[3];

    // Header        
    packet_data[0] = CMD_GET_ACTIVATE;
    
    // Fill payload
    packet_data[1] = g_ref.onoff;
    
    // Calculate checksum
    packet_data[2] = LCRChecksum(packet_data, 2);
    
    // Send package to UART
    commWrite(packet_data, 3);

}

void cmd_ping(){

    uint8 packet_data[2];

    // Header
    packet_data[0] = CMD_PING;
    
    // Load Payload
    packet_data[1] = CMD_PING;

    // Send Package to uart
    commWrite(packet_data, 2);
}

void cmd_store_params(){
    
    uint8 CYDATA packet_lenght = 2;
    uint8 CYDATA packet_data[2];
    uint8 CYDATA old_id;
    
    // Check input mode enabled
    if( c_mem.input_mode == INPUT_MODE_EXTERNAL ){
    
        if (c_mem.m_mult[0] != g_mem.m_mult[0]){
            // Old m_mult
            g_refNew.pos[0] /= c_mem.m_mult[0];
            // New m_mult
            g_refNew.pos[0] *= g_mem.m_mult[0];
        }
        
        if (c_mem.m_mult[1] != g_mem.m_mult[1]){
            // Old m_mult
            g_refNew.pos[1] /= c_mem.m_mult[1];
            // New m_mult
            g_refNew.pos[1] *= g_mem.m_mult[1];
        }
        
        if (c_mem.m_off[0] != g_mem.m_off[0])
            g_refNew.pos[0] += g_mem.m_off[0] - c_mem.m_off[0];

        if (c_mem.m_off[1] != g_mem.m_off[1])
            g_refNew.pos[1] += g_mem.m_off[1] - c_mem.m_off[1];
            
        // Check position Limits
        if (c_mem.pos_lim_flag) {                   // position limiting
            if (g_refNew.pos[0] < c_mem.pos_lim_inf[0]) g_refNew.pos[0] = c_mem.pos_lim_inf[0];
            if (g_refNew.pos[1] < c_mem.pos_lim_inf[1]) g_refNew.pos[1] = c_mem.pos_lim_inf[1];

            if (g_refNew.pos[0] > c_mem.pos_lim_sup[0]) g_refNew.pos[0] = c_mem.pos_lim_sup[0];
            if (g_refNew.pos[1] > c_mem.pos_lim_sup[1]) g_refNew.pos[1] = c_mem.pos_lim_sup[1];
        }
    }
    
    // Store params 
    if (c_mem.id != g_mem.id) {     //If a new id is going to be set we will lose communication 
        old_id = c_mem.id;          //after the memstore(0) and the ACK won't be recognised
        if(memStore(0)) {
            packet_data[0] = ACK_OK;
            packet_data[1] = ACK_OK;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }    
        else{
            packet_data[0] = ACK_ERROR;
            packet_data[1] = ACK_ERROR;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }
    }    
    else {
        if ( memStore(0) ) {
            sendAcknowledgment(ACK_OK);
        } else {
            sendAcknowledgment(ACK_ERROR);
        }
    }
}

void cmd_set_baudrate(){
    
    // Set BaudRate
    c_mem.baud_rate = g_rx.buffer[1];
    
    switch(g_rx.buffer[1]){
        case 13:
            CLOCK_UART_SetDividerValue(13);
            break;
        default:
            CLOCK_UART_SetDividerValue(3);
    }
}

/* [] END OF FILE */