// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/**
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include <stdio.h>
#include <interruptions.h>

#include "commands.h"
#include "utils.h"

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(void){
    int i;              //iterator
    uint8 rx_cmd;
    uint8 aux_checksum;
    uint8 packet_data[16];
    uint8 packet_lenght;
    int32 pos_1, pos_2;
    int32 pos, stiff;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum
    aux_checksum = LCRChecksum(g_rx.buffer, g_rx.length - 1);
    if (!(aux_checksum == g_rx.buffer[g_rx.length - 1])){
        // wrong checksum
        g_rx.ready = 0;
        return;
    }


    switch(rx_cmd){
//=============================================================     CMD_ACTIVATE
        case CMD_ACTIVATE:
            g_ref.onoff = g_rx.buffer[1];

            if (g_mem.control_mode == CONTROL_ANGLE) {
                g_ref.pos[0] = g_meas.pos[0];
                g_ref.pos[1] = g_meas.pos[1];
            }

            if(abs(g_meas.pos[0]) > 26000 || abs(g_meas.pos[1]) > 26000 || abs(g_meas.pos[2]) > 26000){
                g_ref.onoff = 0x00;
            }
            MOTOR_ON_OFF_Write(g_ref.onoff);

            break;
//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            g_ref.pos[0] = *((int16 *) &g_rx.buffer[1]);   // motor 1
            g_ref.pos[0] = g_ref.pos[0] << g_mem.res[0];

            g_ref.pos[1] = *((int16 *) &g_rx.buffer[3]);   // motor 2
            g_ref.pos[1] = g_ref.pos[1] << g_mem.res[1];

            if (c_mem.pos_lim_flag) {                      // pos limiting
                if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];
                if (g_ref.pos[1] < c_mem.pos_lim_inf[1]) g_ref.pos[1] = c_mem.pos_lim_inf[1];

                if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
                if (g_ref.pos[1] > c_mem.pos_lim_sup[1]) g_ref.pos[1] = c_mem.pos_lim_sup[1];
            }

            break;

//========================================================     CMD_SET_POS_STIFF

        case CMD_SET_POS_STIFF:
            pos = *((int16 *) &g_rx.buffer[1]);      // equilibrium position
            stiff = *((int16 *) &g_rx.buffer[3]);    // stiffness

            // position in ticks
            pos = pos << g_mem.res[0];

            // position limit
            if (pos > (c_mem.pos_lim_sup[0] - c_mem.max_stiffness))
                pos = c_mem.pos_lim_sup[0] - c_mem.max_stiffness;

            if (pos < (c_mem.pos_lim_inf[0] + c_mem.max_stiffness))
                pos = c_mem.pos_lim_inf[0] + c_mem.max_stiffness;

            // stiffness is intended between -32768 and 32767
            // remap  stiff value between -MAX_STIFFNESS and MAX_STIFFNESS
            stiff = (int32)(((float)c_mem.max_stiffness / 32768.0) * stiff);

            // pos stiff rule
            g_ref.pos[0] = pos + stiff;
            g_ref.pos[1] = pos - stiff;

            break;

//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            // Packet: header + measure(int16) + crc

            packet_lenght = 1 + (NUM_OF_SENSORS * 2) + 1;

            packet_data[0] = CMD_GET_MEASUREMENTS;   //header

            for (i = 0; i < NUM_OF_SENSORS; i++) {
                *((int16 *) &packet_data[(i*2) + 1]) = (int16)
                (g_meas.pos[i] >> g_mem.res[i]);
            }

            packet_data[packet_lenght - 1] =
                    LCRChecksum (packet_data, packet_lenght - 1);

            commWrite(packet_data, packet_lenght);

        break;

//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            //Packt: header + measure(int16) + CRC
            packet_lenght = 6;

            packet_data[0] = CMD_GET_CURRENTS;

            *((int16 *) &packet_data[1]) = (int16) g_meas.curr[0];
            *((int16 *) &packet_data[3]) = (int16) g_meas.curr[1];

            packet_data[5] = LCRChecksum (packet_data, packet_lenght - 1);

            commWrite(packet_data, packet_lenght);
        break;

//====================================================     CMD_GET_CURR_AND_MEAS

        case CMD_GET_CURR_AND_MEAS:
            //Packet: header + curr_meas(int16) + pos_meas(int16) + CRC
            //packet_lenght = 1 + 2 * 2 + (NUM_OF_SENSORS * 2) + 1;
            packet_lenght = 6 + (NUM_OF_SENSORS * 2);

            packet_data[0] = CMD_GET_CURR_AND_MEAS;

            // Currents
            *((int16 *) &packet_data[1]) = (int16) g_meas.curr[0];
            *((int16 *) &packet_data[3]) = (int16) g_meas.curr[1];

            // Positions
            for (i = 0; i < NUM_OF_SENSORS; i++) {
                *((int16 *) &packet_data[(i*2) + 5]) = (int16)
                (g_meas.pos[i] >> g_mem.res[i]);
            }

            packet_data[packet_lenght - 1] =
                LCRChecksum (packet_data,packet_lenght - 1);

            //commWrite(packet_data, packet_lenght);
            commWrite(packet_data, packet_lenght);
        break;

//=======================================================     CMD_GET_VELOCITIES

        case CMD_GET_VELOCITIES:
            // Packet: header + measure(int16) + crc
            packet_lenght = 1 + (NUM_OF_SENSORS * 2) + 1;

            packet_data[0] = CMD_GET_VELOCITIES;   //header

            for (i = 0; i < NUM_OF_SENSORS; i++) {
                *((int16 *) &packet_data[(i*2) + 1]) = (int16)(g_meas.vel[i]);
            }

            packet_data[packet_lenght - 1] =
                    LCRChecksum (packet_data,packet_lenght - 1);

            commWrite(packet_data, packet_lenght);

        break;

//=========================================================     CMD_GET_ACTIVATE

        case CMD_GET_ACTIVATE:
            packet_lenght = 3;

            packet_data[0] = CMD_GET_ACTIVATE;
            packet_data[1] = g_ref.onoff;
            packet_data[2] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);

            break;

//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            packet_lenght = 6;

            pos_1 = g_ref.pos[0]  >> g_mem.res[0];
            pos_2 = g_ref.pos[1]  >> g_mem.res[1];

            *((int16 *) &packet_data[1]) = (int16) (pos_1);
            *((int16 *) &packet_data[3]) = (int16) (pos_2);
            packet_data[5] = LCRChecksum(packet_data,packet_lenght - 1);

            commWrite(packet_data, packet_lenght);
            break;

//=============================================================     CMD_GET_INFO
        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]));
            break;

//============================================================     CMD_SET_PARAM
        case CMD_SET_PARAM:
            paramSet( *((uint16 *) &g_rx.buffer[1]) );
            break;

//============================================================     CMD_GET_PARAM
        case CMD_GET_PARAM:
            paramGet( *((uint16 *) &g_rx.buffer[1]) );
            break;

//=================================================================     CMD_PING
        case CMD_PING:
            packet_lenght = 2;

            packet_data[0] = CMD_PING;
            packet_data[1] = CMD_PING;

            commWrite(packet_data, packet_lenght);
            break;

//=========================================================     CMD_STORE_PARAMS
        case CMD_STORE_PARAMS:
            // Before storing parameters, c_mem and g_mem are different.
            // We need to adjust reference positions according to new values (g_mem)
            
            if((c_mem.m_mult[0] != g_mem.m_mult[0]) || (c_mem.m_mult[1] != g_mem.m_mult[1])) {
                g_ref.pos[0] /= c_mem.m_mult[0];
                g_ref.pos[1] /= c_mem.m_mult[1];
                g_ref.pos[0] *= g_mem.m_mult[0];
                g_ref.pos[1] *= g_mem.m_mult[1];
            }

            if((c_mem.m_off[0] != g_mem.m_off[0]) || c_mem.m_off[1] != g_mem.m_off[1]) {
                g_ref.pos[0] +=  g_mem.m_off[0] - c_mem.m_off[0];
                g_ref.pos[1] +=  g_mem.m_off[1] - c_mem.m_off[1];
            }

            if (c_mem.pos_lim_flag) {                   // position limiting
                if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];
                if (g_ref.pos[1] < c_mem.pos_lim_inf[1]) g_ref.pos[1] = c_mem.pos_lim_inf[1];

                if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
                if (g_ref.pos[1] > c_mem.pos_lim_sup[1]) g_ref.pos[1] = c_mem.pos_lim_sup[1];
            }

            if ( memStore(0) ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
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
    }

    g_rx.ready = 0;
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
//                                                        COMMAND SET PARAMETER
//==============================================================================


void paramSet(uint16 param_type)
{
    uint8 i;

    switch(param_type)
    {
        //-----------------------------------------------------------     Set Id
        case PARAM_ID:
            g_mem.id = g_rx.buffer[3];
            break;

        //----------------------------------------------------     Set PID Param
        case PARAM_PID_CONTROL:
            g_mem.k_p = *((double *) &g_rx.buffer[3]) * 65536;
            g_mem.k_i = *((double *) &g_rx.buffer[3 + 4]) * 65536;
            g_mem.k_d = *((double *) &g_rx.buffer[3 + 8]) * 65536;
            break;

        //-----------------------------------------------     Set Curr PID Param
        case PARAM_PID_CURR_CONTROL:
            g_mem.k_p_c = *((double *) &g_rx.buffer[3]) * 65536;
            g_mem.k_i_c = *((double *) &g_rx.buffer[3 + 4]) * 65536;
            g_mem.k_d_c = *((double *) &g_rx.buffer[3 + 8]) * 65536;
            break;

        //--------------------------------------     Set Startup Activation Flag
        case PARAM_STARTUP_ACTIVATION:
            g_mem.activ = g_rx.buffer[3];
            break;

        //---------------------------------------------------     Set Input Mode
        case PARAM_INPUT_MODE:
            g_mem.input_mode = g_rx.buffer[3];
            break;

        //-------------------------------------------------     Set Control Mode
        case PARAM_CONTROL_MODE:
            g_mem.control_mode = g_rx.buffer[3];
            break;

        //---------------------------------------------------     Set Resolution
        case PARAM_POS_RESOLUTION:
            for (i =0; i < NUM_OF_SENSORS; i++) {
                g_mem.res[i] = g_rx.buffer[i+3];
            }
            break;

        //------------------------------------------------------     Set Offsets
        case PARAM_MEASUREMENT_OFFSET:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                g_mem.m_off[i] = *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            reset_last_value_flag = 1;
            break;

        //--------------------------------------------------     Set Multipliers
        case PARAM_MEASUREMENT_MULTIPLIER:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                g_mem.m_mult[i] =
                    *((double *) &g_rx.buffer[3 + i * 4]);
            }
            break;

        //------------------------------------------     Set Position Limit Flag
        case PARAM_POS_LIMIT_FLAG:
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
            break;

        //-----------------------------------------------     Set Position Limit
        case PARAM_POS_LIMIT:
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                g_mem.pos_lim_inf[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4)]);
                g_mem.pos_lim_sup[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4) + 4]);

                g_mem.pos_lim_inf[i] = g_mem.pos_lim_inf[i] << g_mem.res[i];
                g_mem.pos_lim_sup[i] = g_mem.pos_lim_sup[i] << g_mem.res[i];

            }
            break;

        //------------------------------------------------     Set Current Limit
        case PARAM_CURRENT_LIMIT:
            g_mem.current_limit = *((int16*) &g_rx.buffer[3]);
            break;

    }
    sendAcknowledgment(ACK_OK);
}


//==============================================================================
//                                                         COMMAND GET PARAMETER
//==============================================================================

void paramGet(uint16 param_type)
{
    uint8 packet_data[20];
    uint16 packet_lenght;
    uint8 i;

    packet_data[0] = CMD_GET_PARAM;

    switch(param_type)
    {
        //-----------------------------------------------------------     Get Id
        case PARAM_ID:
            packet_data[1] = c_mem.id;
            packet_lenght = 3;
            break;

        //----------------------------------------------------     Get PID Param
        case PARAM_PID_CONTROL:
            *((double *) (packet_data + 1)) = (double) c_mem.k_p / 65536;
            *((double *) (packet_data + 5)) = (double) c_mem.k_i / 65536;
            *((double *) (packet_data + 9)) = (double) c_mem.k_d / 65536;
            packet_lenght = 14;
            break;

        //-----------------------------------------------     Get Curr PID Param
        case PARAM_PID_CURR_CONTROL:
            *((double *) (packet_data + 1)) = (double) c_mem.k_p_c / 65536;
            *((double *) (packet_data + 5)) = (double) c_mem.k_i_c / 65536;
            *((double *) (packet_data + 9)) = (double) c_mem.k_d_c / 65536;
            packet_lenght = 14;
            break;

        //--------------------------------------     Get Startup Activation Flag
        case PARAM_STARTUP_ACTIVATION:
            packet_data[1] = c_mem.activ;
            packet_lenght = 3;
            break;

        //---------------------------------------------------     Get Input Mode
        case PARAM_INPUT_MODE:
            packet_data[1] = c_mem.input_mode;
            packet_lenght = 3;
            break;

        //-------------------------------------------------     Get Control Mode
        case PARAM_CONTROL_MODE:
            packet_data[1] = c_mem.control_mode;
            packet_lenght = 3;
            break;

        //---------------------------------------------------     Get Resolution
        case PARAM_POS_RESOLUTION:
            for (i = 0; i < NUM_OF_SENSORS; i++) {
                packet_data[i+1] = c_mem.res[i];
            }
            packet_lenght = NUM_OF_SENSORS + 2;
            break;

        //------------------------------------------------------     Get Offsets
        case PARAM_MEASUREMENT_OFFSET:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                *((int16 *) ( packet_data + 1 + (i * 2) )) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);

            }

            packet_lenght = 2 + NUM_OF_SENSORS * 2;
            break;

        //--------------------------------------------------     Get Multipliers
        case PARAM_MEASUREMENT_MULTIPLIER:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                *((double *) ( packet_data + 1 + (i * 4) )) =
                    c_mem.m_mult[i];
            }

            packet_lenght = 2 + NUM_OF_SENSORS * 4;
            break;

        //------------------------------------------     Get Position Limit Flag
        case PARAM_POS_LIMIT_FLAG:
            packet_data[1] = c_mem.pos_lim_flag;
            packet_lenght = 3;
            break;

        //-----------------------------------------------     Get Position Limit
        case PARAM_POS_LIMIT:
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                *((int32 *)( packet_data + 1 + (i * 2 * 4) )) =
                    c_mem.pos_lim_inf[i];
                *((int32 *)( packet_data + 1 + (i * 2 * 4) + 4)) =
                    c_mem.pos_lim_sup[i];
            }
            packet_lenght = 2 + (NUM_OF_MOTORS * 2 * 4);
            break;

        //------------------------------------------------     Get Current Limit
        case PARAM_CURRENT_LIMIT:
            *((int16 *)(packet_data + 1)) = c_mem.current_limit;
            packet_lenght = 4;
            break;

    }

    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int i;

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
    sprintf(str,"PWM Limit: %d\r\n",(int) device.pwm_limit);
    strcat(info_string,str);
    strcat(info_string,"\r\n");

    strcat(info_string, "MOTOR INFO\r\n");
    strcat(info_string, "Motor references: ");
    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "%d ", (int)(g_ref.pos[i] >> c_mem.res[i]));
        strcat(info_string,str);
    }
    strcat(info_string,"\r\n");


    sprintf(str, "Motor enabled: ");
    if (g_ref.onoff & 0x03) {
        strcat(str,"YES\r\n");
    } else {
        strcat(str,"NO\r\n");
    }
    strcat(info_string, str);


    strcat(info_string,"\r\nMEASUREMENTS INFO\r\n");
    strcat(info_string, "Sensor value:\r\n");
    for (i = 0; i < NUM_OF_SENSORS; i++) {
        sprintf(str,"%d -> %d", i+1,
            (int)(g_meas.pos[i] >> c_mem.res[i]));
        strcat(info_string, str);
        strcat(info_string, "\r\n");
    }
    sprintf(str,"Voltage (mV): %ld", (int32) device.tension );
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str,"Current 1 (mA): %ld", (int32) g_meas.curr[0] );
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str,"Current 2 (mA): %ld", (int32) g_meas.curr[1] );
    strcat(info_string, str);
    strcat(info_string,"\r\n");


    strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");

    strcat(info_string, "PID Controller:\r\n");
    sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p / 65536));
    strcat(info_string, str);
    sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i / 65536));
    strcat(info_string, str);
    sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d / 65536));
    strcat(info_string, str);

    strcat(info_string, "Current PID Controller:\r\n");
    sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p_c / 65536));
    strcat(info_string, str);
    sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i_c / 65536));
    strcat(info_string, str);
    sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d_c / 65536));
    strcat(info_string, str);

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
            strcat(info_string, "Current and position\r\n");
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

    sprintf(str, "debug: %ld", 5000001 - (uint32)timer_value);
    strcat(info_string, str);
    strcat(info_string,"\r\n");
}

//==============================================================================
//                                                      WRITE FUNCTION FOR RS485
//==============================================================================

void commWrite(uint8 *packet_data, uint16 packet_lenght)
{
    uint16 i;

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    UART_RS485_PutChar(g_mem.id);
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(i = 0; i < packet_lenght; ++i)
    {
        UART_RS485_PutChar(packet_data[i]);
    }

    i = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && i++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}


//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment(uint8 value) {

    int packet_lenght = 2;
    uint8 packet_data[2];

    packet_data[0] = value;
    packet_data[1] = value;

    commWrite(packet_data, packet_lenght);
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

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        memRestore();
    } else {
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    }
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================

/**
* This function loads default settings from the eeprom.
**/

uint8 memRestore(void) {

    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        return memInit();
    } else {
        return memStore(0);
    }
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================

/**
* This function initialize memory when eeprom is compromised.
**/

uint8 memInit(void) {

    uint8 i;
    //initialize memory settings
    g_mem.id            =   1;
    g_mem.k_p           =   0.1 * 65536;
    g_mem.k_i           =   0 * 65536;
    g_mem.k_d           =   0.8 * 65536;
    g_mem.k_p_c         =   5 * 65536;
    g_mem.k_i_c         =   0 * 65536;
    g_mem.k_d_c         =   0.8 * 65536;
    g_mem.activ         =   0;
    g_mem.input_mode    =   0;
    g_mem.control_mode  =   0;

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

/* [] END OF FILE */