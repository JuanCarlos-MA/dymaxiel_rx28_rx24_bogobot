#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"     // Dynamixel SDK library
#include "usefull_functions.cpp"             // Extra functions: getch()  &  kbhit()

// 1.- Global variables
dynamixel::PortHandler *portHandler;         // Port handler
dynamixel::PacketHandler *packetHandler;     // Packet handler
int dxl_comm_result = COMM_TX_FAIL;          // Communication result
int dxl_comm_present = COMM_TX_FAIL;          // Communication result
int dxl_comm_goal = COMM_TX_FAIL;          // Communication result
uint8_t dxl_error = 0;                       // Dynamixel error
uint16_t dxl_model_number;                   // Dynamixel model number
uint16_t dxl_present_position;               // Present position
uint16_t dxl_goal_position[1];				// Goal position
int position[18];							// Array for each servomotor position
int connect1 = 0;							// Ping accumulator ID: 0 to ID: 8
int connect2 = 0;							// Ping accumulator ID: 10 to ID: 18

// User-function that utilize global variables
void ping(void) {
    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PING Y HAB. TORQUE A LOS MOTORES >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) { //Servomotor ID: 0 to ID: 8	
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error); // Sending Instruction Packet (Ping)
	dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error); // Torque ON
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("%s\n", packetHandler -> getTxRxResult(dxl_comm_result));
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		printf("Servomotor ID: %i Modelo: %d esta conectado \n", dxl_id, dxl_model_number);
		++connect1;
    }

    for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) { //Servomotor ID: 10 to ID: 18
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error); // Sending Instruction Packet (Ping)
	dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error); // Torque ON
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("%s\n", packetHandler -> getTxRxResult(dxl_comm_result));
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		printf("Servomotor ID: %i Modelo: %d esta conectado \n", dxl_id, dxl_model_number);
		++connect2;
    }

    if(connect1 == 5 || connect2 == 9){
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN ACTUAL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) { //Servomotor ID: 0 to ID: 8
		dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error);
		printf("ID: %i PresPos:%03d\n", dxl_id, dxl_present_position);
		position[dxl_id] = dxl_present_position;
	}
	for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) { //Servomotor ID: 10 to ID: 18
		dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error);
		printf("ID: %i PresPos:%03d\n", dxl_id, dxl_present_position);
		position[dxl_id] = dxl_present_position;
	}
    }
}

void wrt(void){
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN A LLEGAR >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) { //Servomotor ID: 0 to ID: 8
		// Sending Instruction Packet (Write position)
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, dxl_id, 30, position[dxl_id], &dxl_error);
		printf("Goal Position: %03d\n", position[dxl_id]);
	}
	for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) { //Servomotor ID: 10 to ID: 18
		// Sending Instruction Packet (Write position)
		dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, dxl_id, 30, position[dxl_id], &dxl_error);
		printf("Goal Position: %03d\n", position[dxl_id]);
	}
}

void torque_off(void) {    // Disable Dynamixel Torque
    // Sending Instruction Packet
    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DESHABILITAR TORQUE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {
	    dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error); // Torque OFF
	    // Analizing received Status Packet
	    if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    else if (dxl_error != 0)
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	    else
		printf("Torque successfully disabled \n");
    }
    for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) {
	    dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error); // Torque OFF
	    // Analizing received Status Packet
	    if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    else if (dxl_error != 0)
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	    else
		printf("Torque successfully disabled \n");
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ping");
    ros::NodeHandle n;

    // -------------------------- INICIA CODIGO DE DINAMIXELSDK ------------------------------------  
    portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); // Linux: "/dev/ttyUSB0" 
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);      // Protocol version
    
    portHandler->openPort();                 // 2.- Open port
    portHandler->setBaudRate(1000000);       // 3.- Set baudrate (Ex. 9600, 57600, 1000000)

    ping();
    sleep(1);
    wrt();
    sleep(5);
    torque_off();
    portHandler->closePort();                // 5.- Close port
  
    // -------------------------- TERMINA CODIGO DE DINAMIXELSDK -----------------------------------
    ros::spinOnce();
    return 0;
}
