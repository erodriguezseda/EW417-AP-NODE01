/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
// Program to test the CAN bus using SPI on the L432KC Nucleo board
// to an MCP2551 CAN transceiver bus IC using https://os.mbed.com/users/tecnosys/code/mcp2515/

// J. Bradshaw 20210512
#include "mbed.h"
#include "platform/mbed_thread.h"
#include "CAN3.h"
#include "BNO055.h"
#include "ServoOut.h"

#define THIS_CAN_ID     0x05   //Address of this CAN device
#define DEST_CAN_ID     0   //Address of destination

Serial pc(USBTX, USBRX);    //pc serial (tx, rx) uses USB PA_9 and PA_10 on Nucleo D1 and D0 pins
BNO055 bno(D4, D5);
SPI spi(D11, D12, D13);   // mosi, miso, sclk
CAN3 can3(spi, D10, D2);         // spi bus, CS for MCP2515 controller
ServoOut servoOut1(PA_0);   //A0);     // PA_0 is the servo output pulse
AnalogIn ain3(A3);

unsigned char can_txBufLen = 0;
unsigned char can_tx_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
CANMessage canTx_msg;

unsigned char can_rx_bufLen = 8;
unsigned char can_rx_buf[8];
unsigned short can_rxId = 0;
CANMessage canRx_msg;

Timer t;

void bno_init(void){    
    if(bno.check()){
        pc.printf("BNO055 connected\r\n");
        bno.setmode(OPERATION_MODE_CONFIG);
        bno.SetExternalCrystal(1);
        //bno.set_orientation(1);
        bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
        bno.set_angle_units(RADIANS);
    }
    else{
        pc.printf("BNO055 NOT connected\r\n Program Trap.");
        while(1);
    }    
}
    
int main() {        
    thread_sleep_for(500);
    
    t.start();        
    pc.baud(115200);
    bno_init();
    //can3.reset();            // reset the can bus interface
    can3.frequency(500000);    // set up for 500K baudrate
    
    servoOut1.pulse_us = 1500;
            
    pc.printf("CAN MCP2515 test: %s\r\n", __FILE__);
    while(1) {
        
        servoOut1.pulse_us = 1000 + (1000.0*ain3);
        
        bno.get_angles();
        
        pc.printf("%.2f %.2f %.2f\r\n",bno.euler.roll, bno.euler.pitch, bno.euler.yaw);
        //sprintf(can_rx_buf, "b%.2f\r\n", bno.euler.yaw);    //format output message string
        can_tx_buf[0] = bno.euler.rawroll & 0x00ff;
        can_tx_buf[1] = (bno.euler.rawroll >> 8) & 0x00ff;
        can_tx_buf[2] = bno.euler.rawpitch & 0x00ff;
        can_tx_buf[3] = (bno.euler.rawpitch >> 8) & 0x00ff;
        can_tx_buf[4] = bno.euler.rawyaw & 0x00ff;
        can_tx_buf[5] = (bno.euler.rawyaw >> 8) & 0x00ff;
        can_tx_buf[6] = 0;
        can_tx_buf[7] = 0;

        // CAN write message
        for(int i=0;i<8;i++){
            canTx_msg.data[i] = can_tx_buf[i];
        }        
        canTx_msg.id = THIS_CAN_ID; //(rand() % 0xff);    // Randomize transmit ID or  THIS_CAN_ID;
        
        can3.write(&canTx_msg);
        pc.printf("%.2f CAN TX id=%02X data: ", t.read(), canTx_msg.id);
        for(int i=0;i<8;i++){
            pc.printf(" %2X", can_tx_buf[i]);
        }
        pc.printf("\r\n");
        //set up a random delay time of up to .79 seconds
        //delayT = ((rand() % 7) * 31.0)  + ((rand() % 9) * .1) + runTime.sec_total;                        
        
        // CAN receive message       
        if(can3.read(&canRx_msg) == CAN_OK){ //if message is available, read into msg
            pc.printf("CAN RX id=0x%02X data: ", canRx_msg.id);
            for (int i = 0; i < canRx_msg.len; i++) {
                pc.printf(" %2X", canRx_msg.data[i]);
            }
            pc.printf("\r\n");
        }
        
        //if(canTxErrors > 50){            
//            can.reset();
//        }
//        if(canRxErrors > 50){            
//            can.reset();
//        }
        thread_sleep_for(50);     
    }//while(1)
}//main
