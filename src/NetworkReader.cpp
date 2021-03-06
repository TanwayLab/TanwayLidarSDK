/*
* Software License Agreement (BSD License)
*
*  Copyright (c) Tanway science and technology co., LTD.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided  that the following conditions are met:
*
*   1.Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*   2.Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*   3.Neither the name of the copyright holder(s) nor the names of its  contributors
*     may be used to endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#include "NetworkReader.h"
#include "CommonDefine.h"


NetworkReader::NetworkReader(std::string lidarIP, std::string localIP, int localPort, PackageCache& packageCache, std::mutex* mutex)
	: m_lidarIP(lidarIP), m_localIP(localIP), m_localPort(localPort), m_packageCache(packageCache), m_mutex(mutex)
{
	run_read.store(false);
	run_exit_pcloud.store(false);
	run_exit_gps.store(false);
}

NetworkReader::~NetworkReader()
{
	run_read.store(false);

	while (!run_exit_pcloud)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	while (!run_exit_gps)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void NetworkReader::Start()
{
	run_read.store(true);
	run_exit_pcloud.store(false);
	run_exit_gps.store(false);
	std::thread(std::bind(&NetworkReader::ThreadProcessPointCloud, this)).detach();
	std::thread(std::bind(&NetworkReader::ThreadProcessGPS, this)).detach();
}

void NetworkReader::ThreadProcessPointCloud()
{
	run_exit_pcloud.store(false);
	
#ifdef _WIN32
	WSADATA wsd; 
	int nResult = WSAStartup(MAKEWORD(2, 2), &wsd);
	if (nResult != NO_ERROR)
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_INIT_SOCKET, std::string("Init socket error!"));
		run_exit_pcloud.store(true);
		return;
	}
#endif

	

	SocketT recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 

	if (recvSocket == INVALID_SOCKET)
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_CREATE_SOCKET_POINT, std::string("Create point cloud socket error!"));
		run_exit_pcloud.store(true);
		return;
	}

	SOCKADDR_IN sRecvAddr, sSendAddr; 
	socklen_t nSenderAddrSize = sizeof(sSendAddr);
	sRecvAddr.sin_family = AF_INET; 
	sRecvAddr.sin_port = htons(m_localPort);
#ifdef __linux__
	sRecvAddr.sin_addr.s_addr = inet_addr(m_localIP.data());
#elif _WIN32
	inet_pton(AF_INET, m_localIP.data(), &(sRecvAddr.sin_addr.s_addr));
#endif
	

	int nRet = bind(recvSocket, (SOCKADDR *)&sRecvAddr, sizeof(sRecvAddr));
	if (nRet == SOCKET_ERROR) 
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_BIND_POINT, std::string("Bind port for point cloud socket error! "));
		run_exit_pcloud.store(true);
		return;
	}

	//time out
#ifdef __linux__
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if (SOCKET_ERROR == setsockopt(recvSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(timeval)))
#elif _WIN32
	int timeout = 1000;
	if (SOCKET_ERROR == setsockopt(recvSocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int)))
#endif
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_SETOPT_TIMEOUT_GPS, std::string("Failed to set gps socket timeout!"));
		run_exit_pcloud.store(true);
		return;
	}
	int error_count = 0;
	while (run_read)
	{
		TWUDPPackage::Ptr udp_data(new TWUDPPackage);
		udp_data->m_length = recvfrom(recvSocket, udp_data->m_szData, UDP_MAX_LENGTH, 0, (SOCKADDR *)&sSendAddr, &nSenderAddrSize);

		if (udp_data->m_length < 0)
		{
#ifdef __linux__
			if (errno == EWOULDBLOCK)
#elif _WIN32
			if (WSAGetLastError() == WSAETIMEDOUT)
#endif
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_TIMEOUT_POINT, std::string("Receive point data time out!"));
				continue;
			}
			else
			{
				USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_SOCKET_RECV_POINT, std::string("The point cloud socket received failed and will exit!"));
				error_count++;
				if (error_count >= 10)
				{
					break;
				}
			}
		}

		//std::string  strLidarIP = inet_ntoa(sSendAddr.sin_addr);
		char IPdotdec[20];
		inet_ntop(AF_INET, (void *)&(sSendAddr.sin_addr), IPdotdec, 16);
		std::string  strLidarIP = IPdotdec;

		if (strLidarIP != m_lidarIP) continue;
		
		m_packageCache.PushBackPackage(udp_data);

	}
	run_exit_pcloud.store(true);

	USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_EXIT_POINT, std::string("The point cloud data receiver thread has exited!"));
}

void NetworkReader::ThreadProcessGPS()
{
	run_exit_gps.store(false);

#ifdef _WIN32
	WSADATA wsd; 
	int nResult = WSAStartup(MAKEWORD(2, 2), &wsd);
	if (nResult != NO_ERROR)
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_INIT_SOCKET, std::string("Init socket error!"));
		run_exit_gps.store(true);
		return;
	}
#endif

	SocketT recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); //

	if (recvSocket == INVALID_SOCKET)
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_CREATE_SOCKET_GPS, std::string("Create gps socket error!"));
		run_exit_gps.store(true);
		return;
	}

	SOCKADDR_IN sRecvAddr, sSendAddr; 
	socklen_t nSenderAddrSize = sizeof(sSendAddr);
	sRecvAddr.sin_family = AF_INET;
	sRecvAddr.sin_port = htons(10110);
#ifdef __linux__
	sRecvAddr.sin_addr.s_addr = inet_addr(m_localIP.data());
#elif _WIN32
	inet_pton(AF_INET, m_localIP.data(), &(sRecvAddr.sin_addr.s_addr));
#endif

	int nRet = bind(recvSocket, (SOCKADDR *)&sRecvAddr, sizeof(sRecvAddr));

	if (nRet == SOCKET_ERROR)
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_BIND_GPS, std::string("Bind port for gps socket error!"));
		run_exit_gps.store(true);
		return;
	}

	//time out
#ifdef __linux__
	struct timeval tv;
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	if (SOCKET_ERROR == setsockopt(recvSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(timeval)))
#elif _WIN32
	int timeout = 2000;
	if (SOCKET_ERROR == setsockopt(recvSocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int)))
#endif
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_SETOPT_TIMEOUT_GPS, std::string("Failed to set gps socket timeout!"));
		run_exit_gps.store(true);
		return;
	}
	int error_count = 0;
	while (run_read)
	{
		TWUDPPackage::Ptr udp_data(new TWUDPPackage);
		udp_data->m_length = recvfrom(recvSocket, udp_data->m_szData, UDP_MAX_LENGTH, 0, (SOCKADDR *)&sSendAddr, &nSenderAddrSize);

		if (udp_data->m_length < 0)
		{
#ifdef __linux__
			if (errno == EWOULDBLOCK)
#elif _WIN32
			if (WSAGetLastError() == WSAETIMEDOUT)
#endif
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_TIMEOUT_GPS, std::string("Receive gps data time out!"));
				continue;
			}
			else
			{
				USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_SOCKET_RECV_GPS, std::string("The gps socket received failed and will exit!"));
				error_count++;
				if (error_count >= 10)
				{
					break;
				}
			}
		}

		char IPdotdec[20];
		inet_ntop(AF_INET, (void *)&(sSendAddr.sin_addr), IPdotdec, 16);
		std::string  strLidarIP = IPdotdec;

		if (strLidarIP != m_lidarIP) continue;

		m_packageCache.PushBackPackage(udp_data);

	}
	run_exit_gps.store(true);

	USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_EXIT_GPS, std::string("The gps data receiver thread has exited!"));
}

void NetworkReader::RegExceptionCallback(const std::function<void(const TWException&)>& callback)
{
	m_funcException = callback;
}
