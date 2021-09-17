/* Template for EW485B Lab 03 - Node 1  */

//---------------------------------------------------------------------------------------------------//
//------START OF PART I OF CODE: YOU DO NOT HAVE TO MODIFY THE CODE BETWEEN THESE LINES--------------//
//---------------------------------------------------------------------------------------------------//

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "BNO055.h"
#include "CAN3.h"
#include "ServoOut.h"

int myID = 1;

Serial pc(USBTX, USBRX);    //pc serial (tx, rx) uses USB PA_9 and PA_10 on Nucleo D1 and D0 pins
BNO055 bno(D4, D5);
SPI spi(D11, D12, D13);   // mosi, miso, sclk
CAN3 can3(spi, D10, D2);         // spi bus, CS for MCP2515 controller
ServoOut servoOut1(A0);   //A0);     // PA_0 is the servo output pulse

CANMessage canTx_msg;
CANMessage canRx_msg;

Timer t;

void bno_init(void)
{
    if(bno.check()) {
        pc.printf("BNO055 connected\r\n");
        bno.setmode(OPERATION_MODE_CONFIG);
        bno.SetExternalCrystal(1);
        //bno.set_orientation(1);
        bno.setmode(OPERATION_MODE_NDOF);  
        bno.set_angle_units(DEGREES);
    } else {
        pc.printf("BNO055 NOT connected\r\n Program Trap.");
        while(1);
    }
}

float unwrap(float previous_angle, float new_angle)
{
    float d = new_angle - previous_angle;
    if (d > 180) {
        d = d - 360;
    } else {
        if (d < -180) {
            d = d + 360;
        }
    }
    return previous_angle + d;
}

int main()
{
    servoOut1.pulseMin = 500;
    servoOut1.pulseMax = 2500;
    thread_sleep_for(500);
    int initServo[5];
    float currAngles[5] = {-500,-500,-500,-500,-500};
    float yaw, oldYaw;
    float currTime;
    float duration;

    pc.baud(115200);
    pc.printf("Starting Program... \n\r");
    bno_init();
    //can3.reset();            // reset the can bus interface
    can3.frequency(500000);    // set up for 500K baudrate
    char msg_send[8];
    char msg_read_char[8];
    //servoOut1.pulse_us = 1500;
    while(1) {
        pc.printf("Node %d is ready!\n\r", myID);
        bno.get_angles();
        pc.printf("Last Yaw Position: %.2f\n\r", bno.euler.yaw);
        pc.printf("------Set up graph, -----\n\r");
        int adjSet = 1;
        int adjMatrix[5][5];
        while(adjSet) {
            pc.printf("Enter the ij element of Adjacency matrix (only '0' or '1') \n\r");
            pc.printf("\n");
            for(int i = 0; i < 5; i++) {
                for(int j=0; j < 5; j++) {
                    if ((i == 4) && (j==4)) {
                        pc.printf("Last Entry...\n\r");
                    }
                    pc.printf("Enter value for A[%d][%d]:", i+1, j+1);
                    pc.scanf("%d", &adjMatrix[i][j]);
                    pc.printf("%d \n\r", adjMatrix[i][j]);
                }
            }
            pc.printf("You entered:\n\r");
            for(int i = 0; i < 5; i++) {
                for(int j = 0; j < 5; j++) {
                    pc.printf("%d ", adjMatrix[i][j]);
                }
                pc.printf("\n\r");
            }
            pc.printf("Is the matrix correct? [Y,N]");
            char c = pc.getc();
            pc.printf("%c \n\r", c);
            if ((c == 'y') || (c == 'Y')) {
                adjSet = 0;
            } else {
                pc.printf("Try again.\n\r");
            }
        }
        pc.printf("Enter the initial position for each link (or servo motor).\n\r");
        pc.printf("\n");
        int tempServoPos = 0;
        for(int i = 0; i < 5; i++) {
            pc.printf("Position for Node %d (an integer between 0 and 20): ", (i+1));
            pc.scanf("%d", &tempServoPos);
            initServo[i] = 500 + tempServoPos*100;
            pc.printf("%d => PCM %d\n\r", tempServoPos, initServo[i]);
        }
        pc.printf("Enter the duration of experiment in seconds: ");
        pc.scanf("%f", &duration);
        pc.printf("%.2f \n\r", duration);
        //pc.printf("Manually, position Nodes at desired location. Press any key when you are done. \n\r");
        //pc.getc();
        pc.printf("Initializing experiment... \n\r");

        //Start Transmission to other agents
        sprintf(msg_send, "%d\r\n", 2021);
        for(int i=0; i<8; i++) {
            canTx_msg.data[i] = msg_send[i];
        }
        canTx_msg.id = myID;
        can3.write(&canTx_msg); //Send command
        thread_sleep_for(500);

        for(int i = 0; i < 5; i++) {
            sprintf(msg_send, "%d%d%d%d%d\r\n", adjMatrix[i][0],adjMatrix[i][1],adjMatrix[i][2],adjMatrix[i][3],adjMatrix[i][4]);
            for(int i=0; i<8; i++) {
                canTx_msg.data[i] = msg_send[i];
            }
            canTx_msg.id = myID;
            can3.write(&canTx_msg); //Send ith row
            pc.printf("Message Sent (row %d): %s \n\r", i+1, canTx_msg.data);
            thread_sleep_for(500);
        }

        sprintf(msg_send, "%.1f\r\n", duration);
        for(int i=0; i<8; i++) {
            canTx_msg.data[i] = msg_send[i];
        }
        canTx_msg.id = myID;
        can3.write(&canTx_msg); //Send command
        thread_sleep_for(500);

        for(int i = 0; i < 5; i++) {
            sprintf(msg_send, "%d\r\n", initServo[i]);
            for(int i=0; i<8; i++) {
                canTx_msg.data[i] = msg_send[i];
            }
            canTx_msg.id = myID;
            can3.write(&canTx_msg); //Send ith row
            //pc.printf("Message Sent (row %d): %s \n\r", i+1, canTx_msg.data);
            thread_sleep_for(250);
        }

        int controlSignal = initServo[myID-1];
        servoOut1 = controlSignal;
        thread_sleep_for(3000);
        bno.get_angles();
        yaw = bno.euler.yaw;

        //Get initial angles for all
        t.start();
        while (t.read() < 0.5) {
            can3.write(&canTx_msg);
            if(can3.read(&canRx_msg) == CAN_OK) { //if message is available, read into msg
                //pc.printf("CAN RX id=%d data: %s", canRx_msg.id, canRx_msg.data);
                for (int i = 0; i < 8; i++) {
                    msg_read_char[i] = (char)canRx_msg.data[i];
                }
                sscanf(msg_read_char, "%f", &currAngles[canRx_msg.id-1]);
                thread_sleep_for(15-2*myID);  //allows them to use the CAN at different times
                //some wait 13, 11, 9, 7, 5 seconds
            }
        }
        t.reset();

//---------------------------------------------------------------------------------------------------//
//------END OF PART I OF CODE: YOU DO NOT HAVE TO MODIFY THE CODE ABOVE THESE LINES------------------//
//---------------------------------------------------------------------------------------------------//
//-------------START OF PART II OF CODE: WRITE YOUR CODE BELOW THESE LINES---------------------------//
//---------------------------------------------------------------------------------------------------//
        int count = 0;
        while(t.read() < duration) {
            count++;
            if (count == 1) {
                pc.printf("No.\t Time\t Node 1\t Node 2\t Node 3\t Node 4\t Node 5\t PCM\n\r");
            }
            /* Write your code below */
        }
//---------------------------------------------------------------------------------------------------//
//------END OF PART II OF CODE: YOU DO NOT HAVE TO MODIFY THE CODE BELOW THESE LINES-----------------//
//---------------------------------------------------------------------------------------------------//
        t.stop();
        t.reset();
        servoOut1 = 0;
    }
}//main
