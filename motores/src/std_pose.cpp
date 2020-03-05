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
int dxl_comm_present = COMM_TX_FAIL;          // Communication result
int dxl_comm_goal = COMM_TX_FAIL;          // Communication result
uint8_t dxl_error = 0;                       // Dynamixel error
uint16_t dxl_model_number;                   // Dynamixel model number
uint16_t dxl_present_position;               // Present position
uint16_t dxl_present_position_wrt;               // Present position
int dxl_goal_position;
int position[18];							// Array for each servomotor position
int datosCrea[19] = {512, 651, 474, 446, 635, 426, 282, 302, 522, 0, 524, 377, 527, 595, 574, 567, 567, 567, 470}; // Servos position data if the file doesn't exist
int connect1 = 0;							// Ping accumulator ID: 0 to ID: 8
int connect2 = 0;							// Ping accumulator ID: 10 to ID: 18
int std_position[18];						// Array for each servomotor position from file
int pres_position[18];						// Array for each servomotor present position
int a0[18];									// Variable for interpolation
int a1[18];									// Variable for interpolation
int idSelec;								// Varialbe for the ID selection by input
int noDatos = 0;							// Variable if the file doesn't exist
float t0[18];								// Variable for interpolation
float tf[18];								// Variable for interpolation
float q1[18];								// Variable for interpolation

using namespace std;
string idChan;								// Varialbe for the goal position by input	
string comma;								// Variable for data file
string id;									// Varialbe for the ID selection by input
bool isNumber(string);
// User-function that utilize global variables
void torque_on(void) {
    ifstream datosServo("catkin_ws/src/motores/src/servosPos.txt"); // Read the data file (Servos position)
    if(datosServo.fail()){
	ofstream fallaDatos ("catkin_ws/src/motores/src/servosPos.txt"); // Read the data file (Servos position) if doesn't exist before
	noDatos = 1;
	if (fallaDatos.is_open()){
	for(int data = 0; data <= 8; ++data){							//Servomotor ID: 0 to ID: 8	
		fallaDatos << datosCrea[data] << ",";						// Get the data by ','
	}
	for(int data = 10; data <= 18; ++data){							//Servomotor ID: 10 to ID: 18	
		fallaDatos << datosCrea[data] << ",";					   // Get the data by ','
	}
    }
    }
    for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {				//Servomotor ID: 0 to ID: 8	
	if(noDatos != 1){ 
		getline(datosServo,comma,',');							// Get the data by ','
		std_position[dxl_id] = atoi(comma.c_str());				
	}
	else if(noDatos == 1){
		ifstream fallaDatos ("catkin_ws/src/motores/src/servosPos.txt");
		getline(fallaDatos,comma,',');						  // Get the data by ','
		std_position[dxl_id] = atoi(comma.c_str());
	}
	// Sending Instruction Packet
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error); // Ping to servomotors
	dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error);		// Toque ON
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("%s\n", packetHandler -> getTxRxResult(dxl_comm_result));
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		++connect1;
    }

   for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) {				//Servomotor ID: 10 to ID: 18	
	if(noDatos != 1){ 
		getline(datosServo,comma,',');							// Get the data by ','
		std_position[dxl_id] = atoi(comma.c_str());
	}
	else if(noDatos == 1){
		ifstream fallaDatos ("catkin_ws/src/motores/src/servosPos.txt");
		getline(fallaDatos,comma,',');						 // Get the data by ','
		std_position[dxl_id] = atoi(comma.c_str());
	}
	// Sending Instruction Packet
	dxl_comm_result = packetHandler -> ping(portHandler, dxl_id, &dxl_model_number, &dxl_error);  // Ping to servomotors
	dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error);       // Toque ON
	// Analizing received Status Packet
	if (dxl_comm_result != COMM_SUCCESS) // If comm is not succesful
		printf("%s\n", packetHandler -> getTxRxResult(dxl_comm_result));
	else if (dxl_error != 0)             // If dynamixel got an error
		printf("%s\n", packetHandler -> getRxPacketError(dxl_error));
	else                                 // If everything is ok
		++connect2;
    }

}

void presentP(void){
	if(connect1 == 9 && connect2 == 9){ // If the all servos are connected
		printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN ACTUAL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
		for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {
			dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error); // Read present position ID: 0 to ID: 8
			usleep(10000);
			pres_position[dxl_id] = dxl_present_position;
		    printf("ID: %i PresPos:%03d\n", dxl_id, dxl_present_position); // Print present position
		}
		for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) {
			dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error); // Read present position ID: 10 to ID: 18
			usleep(10000);
			pres_position[dxl_id] = dxl_present_position;
			printf("ID: %i PresPos:%03d\n", dxl_id, dxl_present_position); // Print present position
		}
	
    	}	

}

void posGuar(void){
	for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) { //Beginning the linear interpolation ID: 0 to ID: 8
		t0[dxl_id] = 0;
		tf[dxl_id] = 7;
		a0[dxl_id] = ((pres_position[dxl_id]*tf[dxl_id])-(std_position[dxl_id]*0))/(tf[dxl_id]-t0[dxl_id]);
		a1[dxl_id] = (std_position[dxl_id]-pres_position[dxl_id])/(tf[dxl_id]-t0[dxl_id]);
		for(t0[dxl_id]; t0[dxl_id]  <= tf[dxl_id]; t0[dxl_id] = t0[dxl_id] + 0.5){
			q1[dxl_id] = a0[dxl_id] + a1[dxl_id]*t0[dxl_id];
			dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, dxl_id, 30, q1[dxl_id], &dxl_error); // Write each servo position interpolated 
		}
		
	}
	for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) { //Beginning the linear interpolation ID: 10 to ID: 18
		t0[dxl_id] = 0;
		tf[dxl_id] = 7;
		a0[dxl_id] = ((pres_position[dxl_id]*tf[dxl_id])-(std_position[dxl_id]*0))/(tf[dxl_id]-t0[dxl_id]);
		a1[dxl_id] = (std_position[dxl_id]-pres_position[dxl_id])/(tf[dxl_id]-t0[dxl_id]);
		for(t0[dxl_id]; t0[dxl_id]  <= tf[dxl_id]; t0[dxl_id] = t0[dxl_id] + 0.5){
			q1[dxl_id] = a0[dxl_id] + a1[dxl_id]*t0[dxl_id];
			dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, dxl_id, 30, q1[dxl_id], &dxl_error); // Write each servo position interpolated 
		}
	}
}

void wrt(void){
	printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN A LLEGAR >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\n");
	std::cout << "Escriba el ID del motor a mover o escriba '9' para salir: ";
	std::cin >> id;
	idSelec = atoi(id.c_str());	
	if(isNumber(id)){
		if((idSelec >= 0)||(idSelec <= 18)){ // Only select the ID motors or 9 to exit
			if(idSelec == 9){
				idSelec = 19;	
			}
			else{
				dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, idSelec, 36, &dxl_present_position, &dxl_error);
				printf("ID: %d Present Position:%03d\n\n",idSelec, dxl_present_position); 			// Show the current position before it is changed
				cout << "\nEscribe la posición deseada del motor o escrbe 'id' para cambiar de motor: ";
				cin >> idChan;
				dxl_goal_position = atoi(idChan.c_str());
				if(isNumber(idChan) || idChan == "id"){ 					// Validate the input (No words, only numbers or "id")
					do{
					while(idChan != "id"){
						dxl_comm_goal = packetHandler->write2ByteTxRx(portHandler, idSelec, 30, dxl_goal_position, &dxl_error); // Write the positon to the servo selected
						usleep(10000);
						dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, idSelec, 36, &dxl_present_position_wrt, &dxl_error); // Read the positon to the servo selected
						usleep(10000);			
						printf("ID: %d Goal Position: %03d PresPos:%03d\n\n",idSelec, dxl_goal_position, dxl_present_position_wrt); // Show the ID, Goal position selected
						cout << "\nEscribe la posición deseada del motor o escrbe 'id' para cambiar de motor: ";
						cin >> idChan;
						dxl_goal_position = atoi(idChan.c_str());
					}
					}while(isNumber(idChan) || idChan != "id");
				}
				// Validations
				else{
					printf("\nSe debe ingresar un valor tipo Integer, intete nuevamente\n");
				}
		
			}
		}
		else{
			printf("\nValor inválido, intente nuevamente\n");
		}
	}
	else{
		printf("\nSe debe ingresar un valor tipo Integer, intete nuevamente\n");
		}
}

void finalP(void){ // Method to show the final postion before to save
	printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POSICIÓN FINAL >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {
		dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error);
		printf("ID: %i PresPos:%03d\n", dxl_id, dxl_present_position);
		position[dxl_id] = dxl_present_position;
	}
	for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) {
		dxl_comm_present = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_present_position, &dxl_error);
		printf("ID: %i PresPos:%03d\n", dxl_id, dxl_present_position);
		position[dxl_id] = dxl_present_position;
	}
	
    		

}

bool isNumber(string s){ // Method to validate if the user inputs are number
	for(int i = 0; i < s.length(); i++)
		if(isdigit(s[i]) == false)
			return false;
	return true;
}

void torque_off(void) {    // Disable Dynamixel Torque
    // Sending Instruction Packet
    printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DESHABILITAR TORQUE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    for (int dxl_id = 0; dxl_id <= 8; ++dxl_id) {
	    dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error); 		// Torque OFF
	    // Analizing received Status Packet
	    if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    else if (dxl_error != 0)
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	    else
		printf("Torque del ID: %d deshabilitado \n",dxl_id);
    }
    for (int dxl_id = 10; dxl_id <= 18; ++dxl_id) {
	    dxl_comm_goal = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 0, &dxl_error);
	    // Analizing received Status Packet
	    if (dxl_comm_result != COMM_SUCCESS)
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    else if (dxl_error != 0)
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	    else
		printf("Torque del ID: %d deshabilitado \n",dxl_id);
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "std_pose");
    ros::NodeHandle n;

    // -------------------------- INICIA CODIGO DE DINAMIXELSDK ------------------------------------  
    portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); // Linux: "/dev/ttyUSB0" 
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);      // Protocol version
    
    portHandler->openPort();                 // 2.- Open port
    portHandler->setBaudRate(1000000);       // 3.- Set baudrate (Ex. 9600, 57600, 1000000)

    torque_on();
    presentP();
    posGuar();
    while(idSelec != 19){
	wrt();                          
	sleep(1);
    }
    usleep(10000);
    finalP();
	// Save the finals positions in a file
    ofstream guardaServos ("catkin_ws/src/motores/src/servosPos.txt");
    if (guardaServos.is_open()){
	for(int data = 0; data <= 8; ++data){
		guardaServos << position[data] << ",";
	}
	for(int data = 10; data <= 18; ++data){
		guardaServos << position[data] << ",";
	}
	guardaServos.close();
    }
   else cout << "Unable to open file";
    // De-energization of the servo motors
    printf("\nATENCIÓN: ASEGURA EL ROBOT, SE DESENERGIZAR EN 3 SEGUNDOS\n");
    sleep(3);
    torque_off();
    portHandler->closePort();                // 5.- Close port
  
    // -------------------------- TERMINA CODIGO DE DINAMIXELSDK -----------------------------------
    ros::spinOnce();
    return 0;
}
