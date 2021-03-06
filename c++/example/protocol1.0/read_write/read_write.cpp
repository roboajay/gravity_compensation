/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
// Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_CURRENT_CONSUMING		68
#define AADR_MX_GOAL_TORQUE				71
#define ADDR_MX_PRESENT_LOAD 			40




// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          2                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      900               // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

#include <cmath> 

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint16_t dxl_goal_position = 0;         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position
  uint16_t dxl_current_consuming = 0;
  //uint16_t dxl_goal_torque = 2045;
  uint16_t dxl_present_load = 0;
  
  
  int delta_postion = 0;
  volatile double change_in_current_previous = 0;
  
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }


 while(1)
  { 
	  
   // Read consuming current
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CURRENT_CONSUMING, &dxl_current_consuming, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
        printf("error in reading consuming current %s\n", packetHandler->getRxPacketError(dxl_error));
   }
   
   // Read present position
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
      printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
      printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
   }
	
	
   // Read present load
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
      printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
      printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
   }
	
   double change_in_current_present = (dxl_current_consuming - 2048);

	//algorithm for defining delta position 
	
	/*
	// Write goal torque
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, AADR_MX_GOAL_TORQUE, dxl_goal_torque, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
	printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}	
	else if (dxl_error != 0)
	{
	printf("error in writing goal torue %s\n", packetHandler->getRxPacketError(dxl_error));
	}	
	
	*/
	
	
	
	
	
	
	if(change_in_current_present > 2 && change_in_current_previous >= 0)
		{
			delta_postion = -1.0*change_in_current_present; 
			//delta_postion = -10.0;		//towards 285
			dxl_goal_position = dxl_present_position  + delta_postion;
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
			printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
			printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
			}
			
			if(dxl_present_position > dxl_goal_position)
			{
				// Write goal position
				//dxl_goal_position = dxl_present_position  + delta_postion;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}	
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
			
			else if (dxl_present_position <= dxl_goal_position)
			{
				// Write goal position
				dxl_goal_position = dxl_present_position;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
		}
	else if(change_in_current_present < -2 && change_in_current_previous <= 0)
		{
			delta_postion = -1.0*change_in_current_present; 
			//delta_postion = 10.0;  //towards 3810
			dxl_goal_position = dxl_present_position  + delta_postion;
			
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
			printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
			printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
			}
			
			if(dxl_present_position < dxl_goal_position)
			{
				// Write goal position
				//dxl_goal_position = dxl_present_position  + delta_postion;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}	
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
			
			else if (dxl_present_position >= dxl_goal_position)
			{
				// Write goal position
				dxl_goal_position = dxl_present_position;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
		}
	else
		{
			delta_postion = 0;
		}
	//delta_postion = position_coeff*(change_in_current);
	//delta_postion = 0;
	
	change_in_current_previous =  change_in_current_present;
	

	
	printf("\n Current Consuming :%03d, change_in_current_present :%lf, deltaposition:%d, PresPos:%03d, GoalPos:%03d PresLoad:%03d \n", dxl_current_consuming, change_in_current_present, delta_postion, dxl_present_position, dxl_goal_position, dxl_present_load);
	
	

	
}
	
	
	
	
	
	
	
/*
	else if(dxl_goal_position <= 285)
	{
		// Write goal position
		dxl_goal_position = 285;
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, 285, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
		printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
		}
		dxl_current_consuming_past = dxl_current_consuming;
	}
	else if(dxl_goal_position>=3810)
	{
		// Write goal position
		dxl_goal_position = 3810;
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, 3810, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0)
		{
		printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
		}
	
	}
*/
	
	
	
 

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}

