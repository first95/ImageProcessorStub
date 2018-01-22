/*
	messaging.h
	
	Declarations for PCG messaging
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_MSGING_H__
#define __PCG_MSGING_H__

#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include <string.h>
#include <iomanip>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <message_formats.h>
#include "logger.h"
#include "json.hpp"
using json = nlohmann::json;

#define MSGING_RECV_BUFFER_SIZE_B 8192

class Messaging {
private:
	int sockOutboundData = -1; // Unix socket number
	int sockInbound = -1; // Unix socket number
	struct sockaddr_in flightPlannerAddress;
	bool readyToSend = false; // indicates an outbound socket was successfully opened
	bool listening = false; // indicates an incoming socket was successfully bound
	
	// State variables regarding the receive buffer
	char recvBuffer[MSGING_RECV_BUFFER_SIZE_B];
	char * incomingMsgStart = recvBuffer;
	size_t incomingMsgBytesReceived = 0;

	// Networking config variables - must not be changed after calling init functions
	string   flightPlannerIpAddr;
	uint16_t localListenPort;
	uint16_t remoteDestPort;
	uint16_t dataSourcePort; // when we transmit data, it comes from this port.
	uint16_t cmdRespSourcePort; // when we transmit command responses, it comes from this port.

	Logger * logger;
	
	void initListen();
	void initSend();
public:

	static void test();
	void init(json options, Logger * lgr);
	void setBlockingListen(bool block);
	void sendMessage(const Msg * msg);
	Msg * checkForMessage(); // Returns null if no message is waiting.  If not null, caller is responsible for delete[]ing the returned object.
};


#endif // __PCG_MSGING_H__

