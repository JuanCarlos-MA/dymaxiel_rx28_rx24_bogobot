#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"     // Dynamixel SDK library
#include "usefull_functions.cpp"             // Extra functions: getch()  &  kbhit()
#include <iostream>
#include <stdlib.h>
#include <ctype.h>
#include <fstream>

// 1.- Global variables
dynamixel::PortHandler *portHandler;         // Port handler
dynamixel::PacketHandler *packetHandler;     // Packet handler
int dxl_comm_result = COMM_TX_FAIL;          // Communication result
uint8_t dxl_error = 0;                       // Dynamixel error
uint16_t dxl_model_number;                   // Dynamixel model number


int main(int argc, char **argv) {
    ros::init(argc, argv, "ping");
    ros::NodeHandle n;

    // -------------------------- INICIA CODIGO DE DINAMIXELSDK ------------------------------------  
    portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); // Linux: "/dev/ttyUSB0" 
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);      // Protocol version
    
    portHandler->openPort();                 // 2.- Open port
    portHandler->setBaudRate(1000000);       // 3.- Set baudrate (Ex. 9600, 57600, 1000000)

    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PING >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {  //Servomotor ID: 0 to ID: 8
	// Sending Instruction Packet (Ping)
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error);
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("%s\n", packetHandler -> getTxRxResult(dxl_comm_result));
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		printf("Servomotor ID: %i Modelo: %d esta conectado \n", dxl_id, dxl_model_number);
    }

   for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) { //Servomotor ID: 10 to ID: 18
	// Sending Instruction Packet (Ping)
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error);
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("%s\n", packetHandler -> getTxRxResult(dxl_comm_result));
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		printf("Servomotor ID: %i Modelo: %d esta conectado \n", dxl_id, dxl_model_number);
    }

    portHandler->closePort();                // 5.- Close port

    // -------------------------- TERMINA CODIGO DE DINAMIXELSDK -----------------------------------
    ros::spinOnce();
    return 0;



}
