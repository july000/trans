#include "can_io.h"

SocketCAN::SocketCAN(const char *pCanT, const char *pCanR)
{
	canT = pCanT;
	canR = pCanR;
	CANDev_SetUp();
	Socket_Send();
	Socket_Receiv();
}

SocketCAN::~SocketCAN()
{
	// Close the sockets
	close(s_send);
    	close(s_recv);
}

// CAN Dev Set up
void SocketCAN::CANDev_SetUp()
{
	int result = 0;
	std::string set_down_canT = "sudo ip link set down " + canT;
	std::string set_down_canR = "sudo ip link set down " + canR;
	std::string set_up_canT = "sudo ip link set " + canT + " up type can bitrate 500000";
	std::string set_up_canR = "sudo ip link set " + canR + " up type can bitrate 500000";

	result = system(set_down_canT.c_str());
	if (result == 0) {
		std::cout << "set down " << canT << " done!" << std::endl;
	} else {
		std::cout << "set down " << canT << " failed!" << std::endl;
	}

	result = system(set_up_canT.c_str());
	if (result == 0) {
		std::cout << "set up " << canT << " done!" << std::endl;
	} else {
		std::cout << "set up " << canT << " failed!" << std::endl;
	}

	result = system(set_down_canR.c_str());
	if (result == 0) {
		std::cout << "set down " << canR << " done!" << std::endl;
	} else {
		std::cout << "set down " << canR << " failed!" << std::endl;
	}

	result = system(set_up_canR.c_str());
	if (result == 0) {
		std::cout << "set up " << canR << " done!" << std::endl;
	} else {
		std::cout << "set up " << canR << " failed!" << std::endl;
	}
}

// Socket for Sending
void SocketCAN::Socket_Send()
{
	// Create a socket for sending
	s_send = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s_send == -1) {
		perror("socket for sending");
		return;
	}

	// Specify the CAN interface name for sending
	strcpy(ifr_send.ifr_name, canT.c_str());
	ioctl(s_send, SIOCGIFINDEX, &ifr_send);

	// Bind the socket for sending to the CAN interface
	addr_send.can_family = AF_CAN;
	addr_send.can_ifindex = ifr_send.ifr_ifindex;
	if (bind(s_send, (struct sockaddr *)&addr_send, sizeof(addr_send)) == -1)
	{
		perror("bind for sending");
		close(s_send);
		return;
	}
}

// Socket for Receiving
void SocketCAN::Socket_Receiv()
{
	// Create a socket for receiving
	s_recv = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s_recv == -1)
	{
		perror("socket for receiving");
		close(s_send);
		return;
	}

	// Specify the CAN interface name for receiving
	strcpy(ifr_recv.ifr_name, canR.c_str());
	ioctl(s_recv, SIOCGIFINDEX, &ifr_recv);

	// Bind the socket for receiving to the CAN interface
	addr_recv.can_family = AF_CAN;
	addr_recv.can_ifindex = ifr_recv.ifr_ifindex;
	if (bind(s_recv, (struct sockaddr *)&addr_recv, sizeof(addr_recv)) == -1) {
		perror("bind for receiving");
		close(s_send);
		close(s_recv);
		return;
	}
}

bool SocketCAN::CAN_Write(struct can_frame &frame_send)
{
	// Send the CAN frame
	if (write(s_send, &frame_send, sizeof(frame_send)) == -1) {
		perror("write");
		return false;
	}

	printf("CAN frame sent successfully from %s!\n", canT.c_str());
	return true;
}

bool SocketCAN::CAN_Read(struct can_frame &frame_recv, canid_t can_id)
{
	// Receive the CAN frame
	ssize_t nbytes = read(s_recv, &frame_recv, sizeof(frame_recv));
	if (nbytes == -1)
	{
		perror("read");
		return false;
	}
	else if (nbytes == sizeof(frame_recv))
	{
		if (frame_recv.can_id == can_id)
		{
			printf("Received CAN frame on %s: ID=0x%x, DLC=%d, Data=", canR.c_str(), frame_recv.can_id, frame_recv.can_dlc);
			for (int i = 0; i < frame_recv.can_dlc; ++i)
			{
				printf("%02x ", frame_recv.data[i]);
			}

			printf("\n");
		}
	}
	else
	{
		printf("Received incomplete CAN frame on %s!\n", canR.c_str());
		return false;
	}
	return true;
}

void SocketCAN::Set_Control_Module(senso__wheel_control_module_control_t &can_control_module, SocketCAN_Control_Module &ctl_module)
{
    can_control_module.control_word = senso__wheel_control_module_control_control_word_encode(ctl_module.control_word);
    can_control_module.mode_of_operation = senso__wheel_control_module_control_mode_of_operation_encode(ctl_module.mode_of_operation);
    can_control_module.auxiliary_functions = senso__wheel_control_module_control_auxiliary_functions_encode(ctl_module.auxiliary_functions);
    can_control_module.end_stop_position = senso__wheel_control_module_control_end_stop_position_encode(ctl_module.end_stop_position);
    can_control_module.position_offset = senso__wheel_control_module_control_position_offset_encode(ctl_module.position_offset); 
    can_control_module.torque_limitation = senso__wheel_control_module_control_torque_limitation_encode(ctl_module.torque_limitation);
    can_control_module.peak_torque_limitation = senso__wheel_control_module_control_peak_torque_limitation_encode(ctl_module.peak_torque_limitation);
}

// int main(int argc, char* argv[])
// {
// 	if (argc != 3)
// 	{
// 		std::cout << "usage: " << argv[0] << " <can_dev_send> <can_dev_recv>" << std::endl;
// 		std::cout << "check can_dev: ip li" << std::endl;
// 		return 0;
// 	}
// 
// 	SocketCAN Can_IO(argv[1], argv[2]);
// 
// 
// 	// Prepare a CAN frame for sending
// 	struct can_frame frame_send;
// 	frame_send.can_id = 0x123;
// 	frame_send.can_dlc = 8;
// 	memcpy(frame_send.data, "\x11\x22\x33\x44\x55\x66\x77\x88", 8);
// 
// 	Can_IO.CAN_Write(frame_send);
// 	Can_IO.CAN_Write(frame_send);
// 
// 	struct can_frame frame_recv;
// 	Can_IO.CAN_Read(frame_recv);
// 
// 	return 0;
// }

