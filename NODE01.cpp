/*
Sends and Reads position of servos in degrees and prints them all.
 */

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
//DigitalOut led(LED3);

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
        bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
        bno.set_angle_units(DEGREES);
    } else {
        pc.printf("BNO055 NOT connected\r\n Program Trap.");
        while(1);
    }
}

float unwrap(float previous_angle, float new_angle)
{
    float d = new_angle - previous_angle;
    //d = d > 180 ? d - 2 * 180 : (d < -180 ? d + 2 * 180 : d);
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
    servoOut1 = 500;
    thread_sleep_for(1000);
    servoOut1 = 2500;
    thread_sleep_for(1000);
    servoOut1 = 1500;
    thread_sleep_for(1000);
    int initServo[5];
    float currAngles[5] = {-500,-500,-500,-500,-500};
    float yaw, oldYaw;
    float currTime;
    float duration;
    int estimate;
    float estimateYaw;

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
        pc.printf("Last BNO Yaw Position: %.2f\n\r", bno.euler.yaw);
        pc.printf("------Set up graph, -----\n\r");
        int adjSet = 1;
        int adjMatrix[5][5];
        pc.printf("Press 'r' to read the BNO Sensor, press any other key to continue to main program.");
        char r = pc.getc();
        if ((r == 'r') || (r == 'R')) {
            while (!pc.readable()){
            bno.get_angles();
            pc.printf("Angular Position: %5.2f. Press any key to continue to main program.\n\r", bno.euler.yaw);
            thread_sleep_for(400);
            }
        }
        r = pc.getc();
        pc.printf("\n\r \t ***** Agreement Protocol ***** \n\r\n\r");
        pc.printf("Enter 'a', 'b', 'c', 'd', or 'e' for desired communication graph. \n\r");
        pc.printf("\t a) Undirected Path\n\r");
        pc.printf("\t b) Undirected Cycle\n\r");
        pc.printf("\t c) Complete graph\n\r");
        pc.printf("\t d) Digraph 1\n\r");
        pc.printf("\t e) Other - Type Adjacency matrix\n\r");
        int completeGraph[5][5] = {{0,1,1,1,1},{1,0,1,1,1},{1,1,0,1,1},{1,1,1,0,1},{1,1,1,1,0}};
        int pathGraph[5][5] = {{0,1,0,0,0},{1,0,1,0,0},{0,1,0,1,0},{0,0,1,0,1},{0,0,0,1,0}};
        int cycleGraph[5][5] = {{0,1,0,0,1},{1,0,1,0,0},{0,1,0,1,0},{0,0,1,0,1},{1,0,0,1,0}};
        int diGraph[5][5] = {{0,0,0,0,1},{1,0,0,0,0},{0,1,0,0,0},{0,0,1,0,0},{0,0,0,1,0}};
        int graph[5][5];
        char c = pc.getc();
        if ((c == 'c') || (c == 'C')) {
            adjSet = 0;
            pc.printf("Complete graph\n\r");
        } else if ((c == 'a') || (c == 'A')) {
            adjSet = 0;
            pc.printf("Path graph\n\r");
        } else if ((c == 'b') || (c == 'B')) {
            adjSet = 0;
            pc.printf("Cycle graph\n\r");
        } else if ((c == 'd') || (c == 'D')) {
            adjSet = 0;
            pc.printf("Digraph 1\n\r");
        }
        if (adjSet != 1) {
            pc.printf("Adjacency Matrix is:\n\r");
            for(int i = 0; i < 5; i++) {
                for(int j = 0; j < 5; j++) {
                    if ((c == 'c') || (c == 'C')) {
                        adjMatrix[i][j] = completeGraph[i][j];
                    } else if ((c == 'a') || (c == 'A')) {
                        adjMatrix[i][j] = pathGraph[i][j];
                    } else if ((c == 'b') || (c == 'B')){
                        adjMatrix[i][j] = cycleGraph[i][j];
                    } else {
                        adjMatrix[i][j] = diGraph[i][j];
                    }

                    pc.printf("%d ", adjMatrix[i][j]);
                }
                pc.printf("\n\r");
            }
        }
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
            c = pc.getc();
            pc.printf("%c \n\r", c);
            if ((c == 'y') || (c == 'Y')) {
                adjSet = 0;
            } else {
                pc.printf("Try again.\n\r");
            }
        }
        pc.printf("Press '0' to use the BNO Sensor for heading information or '1' to use an Estimate from the Servo Control.\n\r");
        c = pc.getc();
        if (c == '1'){
            pc.printf("Using estimate...\n\r");
            estimate = 1;
        } else {
            pc.printf("Using BNO Sensor...\n\r");
            estimate = 0;
        }


        pc.printf("\n\rPress 1 for default experimental settings, press 2 for user-input settings\n\r");
        pc.printf("  1) Evenly distributed initial servo positions, duration of experiment of 10 seconds\n\r");
        pc.printf("  2) Manually enter initial servo positions and duration of experiment\n\r");
        c = pc.getc();
        
        if (c=='1'){
            initServo[0] = 500;
            initServo[1] = 1000;
            initServo[2] = 1500;
            initServo[3] = 2000;
            initServo[4] = 2500;
            pc.printf("Positions 0, 5, 10, 15, 20\n\r");
            for(int i = 0; i < 5; i++) {
                pc.printf("PCM %d\n\r", initServo[i]);
            }

            duration = 10;
            pc.printf("Duration of %.2f\n\r", duration);
            thread_sleep_for(500);
        }
        else {
            pc.printf("\t 2) Manually enter servo positions and duration of experiment\n\r");
            pc.printf("Enter the initial position of servo motors in PCM\n\r");
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
        }

        pc.printf("Initializing experiment... \n\r");

        //Start Transmission to other agents
        if (estimate == 1){
            sprintf(msg_send, "%d\r\n", 2023);  //send 2023 if estimate
        } else {
            sprintf(msg_send, "%d\r\n", 2021);  //send 2021 if BNO data
        }
        pc.printf("%s",msg_send);

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
        //t.stop();
        t.reset();
        //t.start();
        int count = 0;
        //led = 1;
        while(t.read() < 2.0*duration) {
            //led = !led;
            count++;
            oldYaw = yaw;
            bno.get_angles();
            yaw = unwrap(oldYaw,bno.euler.yaw);
            estimateYaw = 180.0/2000*controlSignal - 45;

            //pc.printf("MyID: %d  Raw Yaw Value: %.2f Unwrapped Yaw %.2f \r\n", myID, bno.euler.yaw, yaw);
            if (estimate == 0) {
                currAngles[myID-1] = yaw;
                sprintf(msg_send, "%.1f\r\n", yaw);
            } else {
                currAngles[myID-1] = estimateYaw;
                sprintf(msg_send, "%.1f\r\n", estimateYaw);
            }
            
            for(int i=0; i<8; i++) {
                canTx_msg.data[i] = msg_send[i];
            }
            canTx_msg.id = myID;

            
            currTime = t.read();
            int writeTimer = currTime + 0.003*(6-myID);
            while (t.read() < currTime + 0.09) {
                if (t.read() >= writeTimer) {
                    can3.write(&canTx_msg);
                    writeTimer = writeTimer + 0.003*(6-myID);
                }
                if(can3.read(&canRx_msg) == CAN_OK) { //if message is available, read into msg
                    //pc.printf("CAN RX id=%d data: %s", canRx_msg.id, canRx_msg.data);
                    for (int i = 0; i < 8; i++) {
                        msg_read_char[i] = (char)canRx_msg.data[i];
                    }
                    sscanf(msg_read_char, "%f", &currAngles[canRx_msg.id-1]);
                    //thread_sleep_for(15-2*myID);  //allows them to use the CAN at different times
                    //some wait 13, 11, 9, 7, 5 seconds
                }
            }
            for (int i = 0; i < 5; i++) {
                if (adjMatrix[myID-1][i] > 0) {
                    controlSignal = controlSignal + 0.5*adjMatrix[myID-1][i]*(int)(currAngles[i]-currAngles[myID-1]);
                }
            }
            if (controlSignal > 2500) {
                controlSignal = 2500;
            } else if (controlSignal < 500) {
                controlSignal = 500;
            }
            servoOut1 = controlSignal;
            if (count == 1) {
                pc.printf("No.\t Time\t Node 1\t Node 2\t Node 3\t Node 4\t Node 5\t PCM\n\r");
            }
            pc.printf("%d \t %.2f \t", count, t.read()/2);
            for(int i = 0; i < 5; i++) {
                pc.printf("%.1f\t", currAngles[i]);
            }
            pc.printf("%d\n\r", controlSignal);
        }//while(1)
        t.stop();
        t.reset();
        servoOut1 = 0;
        //led = 0;
    }
}//main
