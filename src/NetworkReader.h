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
#include "PackageCache.h"
#include "TWException.h"

class NetworkReader
{
public:
	NetworkReader(TWLidarType lidarType, std::string lidarIP, std::string localIP,int localPointCloudPort, int localDIFPort, PackageCache& packageCache, std::mutex* mutex);
	~NetworkReader();

	void Start();

	void ThreadProcessPointCloud();
	void ThreadProcessGPS();
	void ThreadProcessDIF();
	void RegExceptionCallback(const std::function<void(const TWException&)>& callback);

private:
	TWLidarType m_lidarType;
	std::string m_lidarIP;
	std::string m_localIP;
	int m_localPointCloudPort;
	int m_localDIFPort;
	std::atomic<bool>  run_read;
	std::atomic<bool>  run_exit_pcloud;
	std::atomic<bool>  run_exit_gps;
	std::atomic<bool>  run_exit_dif;
	std::mutex* m_mutex;

	PackageCache& m_packageCache;

	std::function<void(const TWException&)> m_funcException=NULL;
};

