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
	LT_TensorLite,
	LT_TensorPro,
	LT_TensorPro_echo2,
	LT_Scope
};

struct TWUDPPackage
{
	typedef std::shared_ptr<TWUDPPackage> Ptr;

	TWUDPPackage() :m_length(0) {}
	char m_szData[UDP_MAX_LENGTH];
	int m_length;
};

template <typename PointT>
#ifdef _MSC_VER
struct __declspec(align(16)) TWPointCloud
#elif __GNUC__
struct __attribute__((aligned(16))) TWPointCloud
#endif
{
	typedef std::vector<PointT> TWPointData;
	typedef std::shared_ptr<TWPointCloud<PointT>> Ptr;

	void PushBack(PointT point);
	int Size();

	uint32_t height = 0;
	uint32_t width = 0;
	std::string frame_id = "TanwayTP";

	TWPointData m_pointData;
};

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

