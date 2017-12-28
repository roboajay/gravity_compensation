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

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_CURRENT_CONSUMING		2
#define LEN_MX_MOVING                   1

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      285               // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3810               // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
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
  
  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  
  uint16_t dxl1_goal_position = 0;         // Goal position
  uint16_t dxl2_goal_position = 0;

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl1_present_position = 0;             // Present position
  uint16_t dxl2_present_position = 0;  
              
  uint16_t dxl1_current_consuming = 0;
  uint16_t dxl2_current_consuming = 0;
  
  int dxl1_delta_postion = 0;
  int dxl2_delta_postion = 0;
  
  volatile double dxl1_change_in_current_previous = 0;
  volatile double dxl2_change_in_current_previous = 0;
  bool dxl_getdata_result = false; 
  
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

  // Enable Dynamixel Torque for ID:1
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
  
  // Enable Dynamixel Torque for ID:2
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
	  
   /*   // Bulkread present position and current consuming for ID 1&2
      dxl_comm_result = groupBulkRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

      dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID);
        return 0;
      }

      dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL2_ID);
        return 0;
      }
      
      dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_MX_CURRENT_CONSUMING, LEN_MX_CURRENT_CONSUMING);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL1_ID);
        return 0;
      }

      dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_MX_CURRENT_CONSUMING, LEN_MX_CURRENT_CONSUMING);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL2_ID);
        return 0;
      }


      // Get present position value
      dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
	  dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
      // Get current consuming value
      dxl1_present_position = groupBulkRead.getData(DXL1_ID, ADDR_MX_CURRENT_CONSUMING, LEN_MX_CURRENT_CONSUMING);
      dxl2_present_position = groupBulkRead.getData(DXL2_ID, ADDR_MX_CURRENT_CONSUMING, LEN_MX_CURRENT_CONSUMING);  */
  

      //printf("[ID:%03d] Present Position : %d \t [ID:%03d] Is Moving : %d\n", DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_moving);
	
   // Read consuming current for ID:1
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_CURRENT_CONSUMING, &dxl1_current_consuming, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
        printf("error in reading consuming current %s\n", packetHandler->getRxPacketError(dxl_error));
   }
   
   // Read present position for ID:1
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
      printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
      printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
   }
	
	
   // Read consuming current for ID:2
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_CURRENT_CONSUMING, &dxl2_current_consuming, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
        printf("error in reading consuming current %s\n", packetHandler->getRxPacketError(dxl_error));
   }
   
   // Read present position for ID:2
   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS)
   {
      printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
   }
   else if (dxl_error != 0)
   {
      printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
   }
	
	double dxl1_change_in_current_present = (dxl1_current_consuming - 2048);
	double dxl2_change_in_current_present = (dxl2_current_consuming - 2048);

	
	
	if(dxl1_change_in_current_present > 5 && dxl1_change_in_current_previous >= 0)
		{
			dxl1_delta_postion = -3.0 * dxl1_change_in_current_present; 
			//delta_postion = -10.0;		//towards 285
			dxl1_goal_position = dxl1_present_position  + dxl1_delta_postion;
			
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
			printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
			printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
			}
			
			if(dxl1_present_position > dxl1_goal_position)
			{
				// Write goal position
				//dxl_goal_position = dxl_present_position  + delta_postion;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}	
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
			
			else if (dxl1_present_position <= dxl1_goal_position)
			{
				// Write goal position
				dxl1_goal_position = dxl1_present_position;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position, &dxl_error);
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
	else if(dxl1_change_in_current_present < -5 && dxl1_change_in_current_previous <= 0)
		{
			dxl1_delta_postion = -3.0 * dxl1_change_in_current_present; 
			//delta_postion = 10.0;  //towards 3810
			dxl1_goal_position = dxl1_present_position  + dxl1_delta_postion;
			
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl1_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
			printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
			printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
			}
			
			if(dxl1_present_position < dxl1_goal_position)
			{
				// Write goal position
				//dxl_goal_position = dxl_present_position  + delta_postion;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}	
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
			
			else if (dxl1_present_position >= dxl1_goal_position)
			{
				// Write goal position
				dxl1_goal_position = dxl1_present_position;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position, &dxl_error);
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
			dxl1_delta_postion = 0;
		}
	//delta_postion = position_coeff*(change_in_current);
	//delta_postion = 0;
	
	
	//Now for 2
	
	if(dxl2_change_in_current_present > 2 && dxl2_change_in_current_previous >= 0)
		{
			dxl2_delta_postion = -1.0 * dxl2_change_in_current_present; 
			//delta_postion = -10.0;		//towards 285
			dxl2_goal_position = dxl2_present_position  + dxl2_delta_postion;
			
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
			printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
			printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
			}
			
			if(dxl2_present_position > dxl2_goal_position)
			{
				// Write goal position
				//dxl_goal_position = dxl_present_position  + delta_postion;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}	
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
			
			else if (dxl2_present_position <= dxl2_goal_position)
			{
				// Write goal position
				dxl2_goal_position = dxl2_present_position;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_goal_position, &dxl_error);
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
	else if(dxl2_change_in_current_present < -2 && dxl2_change_in_current_previous <= 0)
		{
			dxl2_delta_postion = -1.0 * dxl2_change_in_current_present; 
			//delta_postion = 10.0;  //towards 3810
			dxl2_goal_position = dxl2_present_position  + dxl2_delta_postion;
			
			// Read present position
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
			printf("error in reading  %s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
			printf("error in reading present position %s\n", packetHandler->getRxPacketError(dxl_error));
			}
			
			if(dxl2_present_position < dxl2_goal_position)
			{
				// Write goal position
				//dxl_goal_position = dxl_present_position  + delta_postion;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_goal_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}	
				else if (dxl_error != 0)
				{
				printf("error in writing goal position %s\n", packetHandler->getRxPacketError(dxl_error));
				}
			}
			
			else if (dxl2_present_position >= dxl2_goal_position)
			{
				// Write goal position
				dxl1_goal_position = dxl1_present_position;
				dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_goal_position, &dxl_error);
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
			dxl2_delta_postion = 0;
		}
	
	dxl1_change_in_current_previous =  dxl1_change_in_current_present;
	dxl2_change_in_current_previous =  dxl2_change_in_current_present;
	

	
	printf("\n Current Consuming1 :%03d, change_in_current_present1 :%lf, deltaposition1:%d, PresPos1:%03d, GoalPos1:%03d \n", dxl1_current_consuming, dxl1_change_in_current_present, dxl1_delta_postion, dxl1_present_position, dxl1_goal_position);
	printf("\n Current Consuming2 :%03d, change_in_current_present2 :%lf, deltaposition2:%d, PresPos2:%03d, GoalPos2:%03d \n", dxl2_current_consuming, dxl2_change_in_current_present, dxl2_delta_postion, dxl2_present_position, dxl2_goal_position);
	

	
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
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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

