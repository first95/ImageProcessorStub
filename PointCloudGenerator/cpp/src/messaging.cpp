/*
	messaging.h
	
	Declarations for PCG messaging
	
	2016-12-23  JDW  Created.
*/

#include <messaging.h>
using namespace std;
 
void Messaging::test() {
	Messaging module;
	module.initSend();
	module.initListen();
	
	cout << "Testing receive.  Waiting for packet." << endl;
	Msg* msg_in = NULL;
	while(1) {
		while (msg_in == NULL) {
			msg_in = module.checkForMessage();
			usleep(1000);
		}
		cout << "Got a message of type 0x" << hex << setw(4) << msg_in->getMsgId() <<
			", size 0x" <<  msg_in->getLenB() << " bytes." << endl;
			
		switch(msg_in->getMsgId()) {
			case POINT_CLOUD_METADATA:
			{
				cout << "Received point cloud metadata." << dec << endl;
				PointCloudMetadataMessage * parsed_msg = (PointCloudMetadataMessage *)msg_in;
				cout << "PointCloudSeqNum         = " << parsed_msg->getPointCloudSeqNum         () << endl;
				cout << "NumPktsThisPointCloud    = " << parsed_msg->getNumPktsThisPointCloud    () << endl;
				cout << "NumPointsThisPointCloud  = " << parsed_msg->getNumPointsThisPointCloud  () << endl;
				cout << "OpticalDataCaptureTimeS  = " << parsed_msg->getOpticalDataCaptureTimeS  () << endl;
				cout << "OpticalDataCaptureTimeMs = " << parsed_msg->getOpticalDataCaptureTimeMs () << endl;
				cout << "TimeSpentProcessingMs    = " << parsed_msg->getTimeSpentProcessingMs    () << endl;
			}
			break;
			case POINT_CLOUD_DATA:
			{
				cout << "Received point cloud data." << dec << endl;
				PointCloudDataMessage * parsed_msg = (PointCloudDataMessage *)msg_in;
				cout << "PointCloudSeqNum     = " << parsed_msg->getPointCloudSeqNum     () << endl;
				cout << "PktNumThisPointCloud = " << parsed_msg->getPktNumThisPointCloud () << endl;
				cout << "NumPointsThisMsg     = " << parsed_msg->getNumPointsThisMsg     () << endl;
				
				CloudPoint * cloud = parsed_msg->getPointCloud();
				cout << "cloud[4] PointX() = " << cloud[4].getPointX() << endl;
				cout << "cloud[4] PointY() = " << cloud[4].getPointY() << endl;
				cout << "cloud[4] PointZ() = " << cloud[4].getPointZ() << endl;
			}
			break;
			case EN_POINT_CLOUD_GEN:
				cout << "Commanded to enable point cloud generation." << endl;
			break;
			case DIS_POINT_CLOUD_GEN:
				cout << "Commanded to disable point cloud generation." << endl;
			break;
			case SHUTDOWN_PCG:
				cout << "Commanded to shutdown." << endl;
			break;
			default:
				cout << "Unrecognized MID 0x" << hex << setw(4) << msg_in->getMsgId() << endl;
			break;
		}
		
		delete[] msg_in;
		msg_in = NULL;
	}
}

void Messaging::init(json options, Logger * lgr) { 
	logger = lgr;
	
	// Load configuration options
	string cur_key = "";
	try {
		cur_key = "flightPlannerIpAddr"; flightPlannerIpAddr = options[cur_key];
		cur_key = "localListenPort";     localListenPort     = options[cur_key];
		cur_key = "remoteDestPort";      remoteDestPort      = options[cur_key];
		cur_key = "dataSourcePort";      dataSourcePort      = options[cur_key];
		cur_key = "cmdRespSourcePort";   cmdRespSourcePort   = options[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in acquisition section: "
			 << e.what() << endl;
		throw(e);
		return;
	}

	initSend(); 
	initListen(); 
}

void Messaging::setBlockingListen(bool block) {
	int ret_val = -1;
	// If setting any flags other than O_NONBLOCK, change that 0, you can do a F_GETFL
	ret_val = fcntl(sockInbound, F_SETFL, block? 0 : O_NONBLOCK);
	if(ret_val < 0) {
		throw runtime_error("fcntl(): Failed to set socket blocking behavior.");
	}
}

void Messaging::initListen() {
	int ret_val = -1;
	sockInbound = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(sockInbound >= 0) {
		struct sockaddr_in local_sock;
		memset(&local_sock, 0, sizeof(local_sock));
		local_sock.sin_family = AF_INET;
		local_sock.sin_addr.s_addr = htonl(INADDR_ANY);
		local_sock.sin_port = htons(localListenPort);
		ret_val = bind(sockInbound, (struct sockaddr *) &local_sock, sizeof(local_sock));
		if(ret_val >= 0) {
			listening = true;
		} else {
			throw runtime_error("bind(): Failed to bind incoming data socket.");
		}
	} else {
		throw runtime_error("socket(): Failed to open incoming data socket.");
	}
}

void Messaging::initSend() {
	int ret_val = -1;
	sockOutboundData = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(sockOutboundData >= 0) {
		struct sockaddr_in src_sock;
		memset(&src_sock, 0, sizeof(src_sock));
		src_sock.sin_family = AF_INET;
		src_sock.sin_addr.s_addr = htonl(INADDR_ANY);
		src_sock.sin_port = htons(dataSourcePort);
		ret_val = bind(sockOutboundData, (struct sockaddr *) &src_sock, sizeof(src_sock));
		if(ret_val >= 0) {
			int broadcastEnable=1;
			ret_val = setsockopt(sockOutboundData, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
			if(ret_val == 0) {
				memset((void*)(&flightPlannerAddress), 0, sizeof(flightPlannerAddress));
				flightPlannerAddress.sin_family = AF_INET;
				flightPlannerAddress.sin_port = htons(remoteDestPort);
				ret_val = inet_aton(flightPlannerIpAddr.c_str(), &(flightPlannerAddress.sin_addr));
				if(ret_val == 1) {
					readyToSend = true;
				} else {
					throw runtime_error("inet_aton(): Failed to translate destination IP address.");
				}
			} else {
				throw runtime_error("setsockopt(): Failed to gain broadcast privileges.");
			}
		} else {
			throw runtime_error("bind(): Failed to bind outbound data socket.");
		}
	} else {
		throw runtime_error("socket(): Failed to open outbound data socket.");
	}
}

void Messaging::sendMessage(const Msg * msg) {
	if(readyToSend) {
		int ret_val = sendto(sockOutboundData, (void*)msg, msg->getLenB(), 0 , (sockaddr*)(&flightPlannerAddress), sizeof(flightPlannerAddress));
		if(ret_val != msg->getLenB()) {
			throw runtime_error("sendto(): Failed to send entire message.");
		}
	}
}

Msg * Messaging::checkForMessage() {
	if(!listening) { return NULL; }
	
	// Copy any new data into the buffer
	Msg * ret_msg = NULL;
	struct sockaddr_in sender_address;
	unsigned int sender_addr_len = sizeof(sender_address);
	int bytes_rxd = recvfrom(sockInbound, incomingMsgStart, MSGING_RECV_BUFFER_SIZE_B - incomingMsgBytesReceived, 0,
		(struct sockaddr*)&sender_address, &sender_addr_len);
	if(bytes_rxd == -1) {
		// This is probably due to a lack of message waiting for us
		if(errno == EAGAIN || errno == EWOULDBLOCK) {
			// No new data
			bytes_rxd = 0;
		} else {
			throw runtime_error("recvfrom(): Error checking for new packet.");
		}
	} else {
		// We have new data
	}
	
	// Return the next message, if any, in our buffer
	incomingMsgBytesReceived += bytes_rxd;
	if(incomingMsgBytesReceived >= sizeof(Msg)) {
		// We have enough to start trying to make sense of the message
		size_t msg_len = ((Msg*)incomingMsgStart)->getLenB() ;
		if(msg_len >= incomingMsgBytesReceived) {
			// We have this entire message
			ret_msg = (Msg*)new char[msg_len];
			memcpy((void*)ret_msg, incomingMsgStart, msg_len);
			incomingMsgStart += msg_len;
			incomingMsgBytesReceived -= msg_len;
			
			// If the buffer was left empty by that, jump back to the beginning
			if(incomingMsgBytesReceived == 0) {
				incomingMsgStart = recvBuffer;
			} else {
				// The buffer has at least the start of another message
			}
		} else {
			// We only have a chunk of this message
		}
	} else {
		// Have received either nothing or just part of the header
	}
	
	// Check if we've filled the buffer
	if((incomingMsgStart - recvBuffer) + incomingMsgBytesReceived >= MSGING_RECV_BUFFER_SIZE_B) {
		// Buffer is full.  Discard contents.
		logger->logWarning("Filled message receive buffer; discarding.");
		incomingMsgStart = recvBuffer;
		incomingMsgBytesReceived = 0;
	}
	
	return ret_msg;
}