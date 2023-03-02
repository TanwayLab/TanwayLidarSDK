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
#pragma once
#include "CommomHeader.h"

#ifdef __linux__
#define SocketT socklen_t
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKADDR_IN sockaddr_in
#define SOCKADDR sockaddr
#define sprintfT sprintf
#elif _WIN32
#define SocketT SOCKET
#define sprintfT sprintf_s
#endif

#define UDP_MAX_LENGTH 1600

enum TWLidarType
{
	LT_TensorLite,		//0
	LT_TensorPro,		//1
	LT_TensorPro_echo2,	//2
	LT_Scope,			//3
	LT_TSP0332,			//4
	LT_Scope192,		//5
	LT_Duetto,			//6
	LT_ScopeMiniA2_192,	//7
	LT_Total
};

struct TWUDPPackage
{
	typedef std::shared_ptr<TWUDPPackage> Ptr;

	TWUDPPackage() :m_length(0) {
#ifdef __linux__
		timeval start;
		gettimeofday(&start, NULL);
		t_sec = start.tv_sec;
		t_usec = start.tv_usec;
#elif _WIN32
		time_t clock;
		struct tm tm;
		SYSTEMTIME wtm;
		GetLocalTime(&wtm);
		tm.tm_year = wtm.wYear - 1900;
		tm.tm_mon = wtm.wMonth - 1;
		tm.tm_mday = wtm.wDay;
		tm.tm_hour = wtm.wHour;
		tm.tm_min = wtm.wMinute;
		tm.tm_sec = wtm.wSecond;
		tm.tm_isdst = -1;
		clock = mktime(&tm);
		t_sec = (long)clock;
		t_usec = wtm.wMilliseconds * 1000;
#endif
	}
	char m_szData[UDP_MAX_LENGTH];
	int m_length;
	//time
	unsigned int t_sec; 
	unsigned int t_usec; 
};

struct TWIMUData
{
	TWIMUData()
	{
		memset(angular_velocity, 0, sizeof(float)*3);
		memset(linear_acceleration, 0, sizeof(float)*3);
	}
	uint64_t stamp = 0;
	std::string frame_id = "TanwayIMU";
	
	bool calibrate = false;
	float  temperature = 0;
	float angular_velocity[3];
	float linear_acceleration[3];
	float gyro_noise = 0;
	float gyro_bias = 0;
	float accel_noise = 0;
	float accel_bias = 0;
};

template <typename PointT>
#ifdef _MSC_VER
struct __declspec(align(16)) TWPointCloud
#elif __GNUC__
struct __attribute__((aligned(16))) TWPointCloud
#endif
{
	typedef std::vector<PointT> PointData;
	typedef std::shared_ptr<TWPointCloud<PointT>> Ptr;

	void PushBack(PointT point);
	int Size();
	void Reserve(unsigned int count);

	uint32_t height = 0;
	uint32_t width = 0;
	uint64_t stamp = 0;
	std::string frame_id = "TanwayTP";

	PointData m_pointData;
};

template <typename PointT>
void TWPointCloud<PointT>::Reserve(unsigned int count)
{
	m_pointData.reserve(count);
}

template <typename PointT>
int TWPointCloud<PointT>::Size()
{
	return m_pointData.size();
}

template <typename PointT>
void TWPointCloud<PointT>::PushBack(PointT point)
{
	m_pointData.push_back(point);
}

