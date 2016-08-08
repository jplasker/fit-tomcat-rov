#include "Tomcat_V_driver.h"

void interrupt isr(void) {
    GLOBAL_INT = 0;
    if (OSCFIF){
        LEAK_1 = 1;
    }
    if (TMR0_INT) {
        TMR0_INT = 0;
        TMR0_RESET();
        tmr0_flag = 1;
        time++;
        //CLRWDT();
    }
    if (TMR1_INT) {
        TMR1_INT = 0;
        TMR1_RESET();
        comms_time++;
        if (comms_time > COMMS_TIMEOUT) LED_COMS = 0;
        tmr1_flag = 1;
        //CLRWDT();
    }
    //if (TMR2_INT) {
    //TMR2_INT = 0;
    //TMR2_RESET();
    //tmr2_flag = 1;
    //}
    if (RX1_INT) {
        RX1_INT = 0;
        rx1_buff[rx1_count] = RCREG;
        if (rx1_count > 43) rx1_count = 0;
        if (rx1_buff[rx1_count] == 0x0a && rx1_buff[0] == '$'){
            for (rx1_count = rx1_count + 1; rx1_count < 45; rx1_count++) rx1_buff[rx1_count] = NULL;
            if(rx1_flag == 0) strcpy(rx_temp_buff, rx1_buff);
            rx1_flag = 1;
            rx1_count = 0;
        }
        if (rx1_buff[rx1_count++] == '$'){
            rx1_buff[0] = '$';
            rx1_count = 1;
        }

        //cahnged to looking for "$" ended by newline

        //        if (rx1_count == SUR_PACK_LEN)rx1_flag = 1;
        //        if (rx1_buff[rx1_count] == 0xff)
        //        {
        //            rx1_count = 1;
        //            rx1_255_flag=1;
        //        }
        //        if (rx1_255_flag)rx1_count++;
    }
    if (RX2_INT) {
        RX2_INT = 0;
        rx2_flag = 1;
    }
    if (LEAK1_INT) {
        LEAK1_INT = 0;
        leak1_flag = 1;
        Tomcat_TX_error(LEAK);
    }
    GLOBAL_INT = 1;
}

void main(void) {

    int heading, pitch, roll;
    int depth, int_press, ex_temp, int_temp;
    int main_current = 0, v_current = 0;
    char comms_timed_out = 0;
    char surf_buff[60];
    int input[15] = {0};
    char *token;
    const char delimit[2] = ",";
    char in_count = 0;
    unsigned int check = 0;
    unsigned int check_recv = 0;


    Nop();
    Nop();
    CLRWDT();
    RCON = 0b01111111;
    Tomcat_Setup();
    Thruster_Driver(0, 1); //port
    Thruster_Driver(0, 2); //stbd
    Thruster_Driver(0, 3); //vert
    Thruster_Driver(0, 4); //lat
    //LED_COMS=1;
    while (1) {
        CLRWDT();
        //sprintf(surf_buff, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                //depth, heading, pitch, roll, ex_temp, int_temp, int_press, main_current, v_current);
        sprintf(surf_buff, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d ",
                input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8], check, check_recv, int_press);
        //regularly scheduled stuff
        if (time > 10) {
            //every 10 time ticks
            time = 1; //reset time
            //read environment
            depth = Tomcat_Depth();

            int_press = Tomcat_Press_Int();
            if (int_press > WARN_INT_PRESS)
                Tomcat_TX_warn(PRESS);
            if (int_press > ALARM_INT_PRESS)
                Tomcat_TX_error(PRESS);

            int_temp = Tomcat_Temp();
            if (int_temp > WARN_INT_TEMP)
                Tomcat_TX_warn(TEMP);
            if (int_temp > ALARM_INT_TEMP)
                Tomcat_TX_error(TEMP);

            ex_temp = Tomcat_Temp_Ex();

        }
        if (((time % 5) == 0) && tmr0_flag) {
            //every 5 time ticks
            //send to surface
            Tomcat_TX_data(surf_buff, strlen(surf_buff) - 1);
        }
        CLRWDT();
        if (((time % 2) == 0) && tmr0_flag) {
            //every 2 time ticks
            //read currents
            main_current = analogRead(MAIN_CURRENT) * MAIN_FACTOR;
            v_current = analogRead(V_CURRENT) * V_FACTOR;

            if (v_current > WARN_V_CURRENT)
                Tomcat_TX_warn(CURRENT);
            if (v_current > ALARM_V_CURRENT)
                Tomcat_TX_error(CURRENT);
        }
        if (tmr0_flag) {
            //every 1 time tick
            tmr0_flag = 0;
            //read IMU
            //heading = Tomcat_Heading();
            // pitch = Tomcat_Pitch();
            //  roll = Tomcat_Roll();
        }

        CLRWDT();
        if (comms_time > COMMS_TIMEOUT || comms_timed_out) {
            comms_timed_out = 1;
            LED_COMS = 0; //change to 0
            Tomcat_TX_error(COMMS);

        }
        if (rx1_flag == 1) {
            //rx from surface
            //packet all are ints
            //$port,stbd,vert,lat,grip,wrist,pan,tilt,lights,check sum\n\r

            //LED_COMS = 1;

            while(Busy1USART());
            puts1USART(rx_temp_buff);
            
            token = strtok(&rx_temp_buff[1], ",");//delimit);
            input[0] = atoi(token);
            check = input[0];
            for (in_count = 1; in_count < SUR_PACK_LEN; in_count++) {
                token = strtok(NULL, ",");//delimit);
                input[in_count] = atoi(token);
                check = check + input[in_count];
            }
            check_recv = atoi(strtok(NULL, ","));//delimit));
            //check checksum
            if (check == check_recv) {
                comms_time = 0;
                comms_timed_out = 0;
                LED_COMS = 1;

                Thruster_Driver(input[0], 1); //port
                Thruster_Driver(input[1], 2); //stbd
                Thruster_Driver(input[2], 3); //vert
                Thruster_Driver(input[3], 4); //lat
                Tomcat_Claw(input[4], input[5]);
                Tomcat_Camera(input[6], input[7]);

                if (input[8]) {
                    LIGHTS = 1; //may need to disable if lights draw too much
                } else {
                    LIGHTS = 0;
                }
            }
            rx1_count = 0;
            rx1_flag = 0;
        }
        if (rx2_flag) {
            //rx from pan tilt
            rx2_flag = 0;

        }
        CLRWDT();
        if (leak1_flag || LEAK_1) {
            leak1_flag = 0;
            Tomcat_TX_error(LEAK);
        }
    }
}

