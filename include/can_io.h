
#ifndef CAN_IO_H_
#define CAN_IO_H_

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <iostream>
#include <cstdlib>
#include <string>

#include "senso__wheel.h"

class SocketCAN
{
	public:
    	struct SocketCAN_Control_Module
		{
			int control_word;
			int mode_of_operation;
			int auxiliary_functions;
			int end_stop_position;
			int position_offset;
			int torque_limitation;
			int peak_torque_limitation;
		};

		SocketCAN(const char *pCanT, const char *pCanR);
		~SocketCAN();

		// CAN Dev Set up
		void CANDev_SetUp();
		// Socket for Sending
		void Socket_Send();
		// Socket for Receiving
		void Socket_Receiv();
		bool CAN_Write(struct can_frame &frame_send);
		bool CAN_Read(struct can_frame &frame_recv, canid_t can_id);

    	void Set_Control_Module(senso__wheel_control_module_control_t &can_control_module, SocketCAN_Control_Module &ctl_module);

	private:
		int s_send, s_recv;
		struct sockaddr_can addr_send, addr_recv;
		struct ifreq ifr_send, ifr_recv;
		std::string canT, canR;
};

#endif