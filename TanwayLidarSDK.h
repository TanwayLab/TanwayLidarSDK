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
#include "./src/CommomHeader.h"
#include "./src/PackageCache.h"
#include "./src/NetworkReader.h"
#include "./src/PcapReader.h"
#include "./src/DecodePackage.h"
#include "./src/TWException.h"

/*
*Interface classes for the SDK
*/
template <typename PointT> class TanwayLidarSDK
{
public:
	/*
	*lidarIP: ip address of connected lidar.
	*localIP: ip address of local
	*localPointloudPort: the network port which the lidar pointcloud data is send to
	*localDIFPort: the network port which the lidar dif data is send to
	*lidarType: lidar type
	*decodePackagePtr: new decode processing
	*/
	TanwayLidarSDK(std::string lidarIP, std::string localIP, int localPointloudPort, int localDIFPort, TWLidarType lidarType, std::shared_ptr<DecodePackage<PointT>> decodePackagePtr = NULL);
	/*
	*pcapPath: the pcap file path
	*lidarType: lidar type
	*lidarIPForFilter: the IP address of the lidar used to filter data
	*localPointCloudPortForFilter: the network port which the lidar pointcloud data is send to
	*localDIFPortForFilter: the network port which the lidar dif data is send to
	*repeat: Loops through the PCAP files
	*decodePackagePtr: new decode processing
	*/
	TanwayLidarSDK(std::string pcapPath, std::string lidarIPForFilter, int localPointCloudPortForFilter, int localDIFPortForFilter, TWLidarType lidarType, bool repeat, std::shared_ptr<DecodePackage<PointT>> decodePackagePtr = NULL);

	~TanwayLidarSDK();

	/*
	*Re-read a new pcap file.
	*To reread a new pcap file, the previous pcap file must have been loaded for playback before this function can be used
	*/
	void RereadPcap(std::string pcapPath);
	/*
	*Pause reading the PCAP file
	*/
	void PausePcap(bool pause);
	/*
	*Set corrected value
	*/
	void SetCorrectedAngleToTSP0332(float angle1, float angle2);
	void SetCorrectedAngleToScope192(float angle1, float angle2, float angle3);
	void SetCorrectionAngleToScopeMiniA2_192(float angle1, float angle2, float angle3);
	void SetCorrectionAngleToDuetto(float angle1, float angle2, float angle3);
	void SetCorrectionMovementToDuetto(float lx, float ly, float lz, float rx, float ry, float rz);
	void SetMoveAngleToDuetto(float leftAngle, float rightAngle);
	/*
	*Register the point cloud callback function.
	*/
	inline void RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback);
	/*
	*Register the gps string callback function.
	*/
	inline void RegGPSCallback(const std::function<void(const std::string&)>& callback);
	/*
	*Register the IMU data callback function.
	*/
	inline void RegIMUDataCallback(const std::function<void(const TWIMUData&)>& callback);
	/*
	*Register the exception info callback function.
	*/
	inline void RegExceptionCallback(const std::function<void(const TWException&)>& callback);

	/*
	* run sdk
	*/
	void Start();

private:
	std::shared_ptr<PackageCache> m_packageCache;
	std::shared_ptr<NetworkReader> m_networkReaderPtr;
	std::shared_ptr<PcapReader> m_pcapReaderPtr;
	std::shared_ptr<DecodePackage<PointT>> m_decodePackagePtr;
	std::mutex m_mutexE;
};


template <typename PointT>
TanwayLidarSDK<PointT>::TanwayLidarSDK(std::string lidarIP, std::string localIP, int localPointloudPort, int localDIFPort, TWLidarType lidarType, std::shared_ptr<DecodePackage<PointT>> decodePackagePtr)
{
	m_packageCache = std::make_shared<PackageCache>();
	m_networkReaderPtr = std::make_shared<NetworkReader>(lidarType, lidarIP, localIP, localPointloudPort, localDIFPort, *m_packageCache, &m_mutexE);
	if (!decodePackagePtr)
	{
		m_decodePackagePtr = std::make_shared<DecodePackage<PointT>>(m_packageCache, lidarType, &m_mutexE);
	}
	else
	{
		m_decodePackagePtr = decodePackagePtr;
		m_decodePackagePtr->SetLidarType(lidarType);
		m_decodePackagePtr->SetPackageCache(m_packageCache);
		m_decodePackagePtr->SetMutex(&m_mutexE);
	}
}

template <typename PointT>
TanwayLidarSDK<PointT>::TanwayLidarSDK(std::string pcapPath, std::string lidarIPForFilter, int localPointCloudPortForFilter, int localDIFPortForFilter, TWLidarType lidarType, bool repeat, std::shared_ptr<DecodePackage<PointT>> decodePackagePtr)
{
	m_packageCache = std::make_shared<PackageCache>();
	m_pcapReaderPtr = std::make_shared<PcapReader>(pcapPath, lidarIPForFilter, localPointCloudPortForFilter, localDIFPortForFilter, *m_packageCache, repeat, &m_mutexE);
	if (!decodePackagePtr)
	{
		m_decodePackagePtr = std::make_shared<DecodePackage<PointT>>(m_packageCache, lidarType, &m_mutexE);
	}
	else
	{
		m_decodePackagePtr = decodePackagePtr;
		m_decodePackagePtr->SetLidarType(lidarType);
		m_decodePackagePtr->SetPackageCache(m_packageCache);
		m_decodePackagePtr->SetMutex(&m_mutexE);
	}
}

template <typename PointT>
void TanwayLidarSDK<PointT>::RereadPcap(std::string pcapPath)
{
	if (m_pcapReaderPtr)
		m_pcapReaderPtr->RereadPcap(pcapPath);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::PausePcap(bool pause)
{
	if (m_pcapReaderPtr)
		m_pcapReaderPtr->PausePcap(pause);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::SetCorrectionAngleToDuetto(float angle1, float angle2, float angle3)
{
	m_decodePackagePtr->SetCorrectionAngleToDuetto(angle1, angle2, angle3);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::SetCorrectionMovementToDuetto(float lx, float ly, float lz, float rx, float ry, float rz)
{
	m_decodePackagePtr->SetCorrectionMovementToDuetto(lx, ly, lz, rx, ry, rz);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::SetMoveAngleToDuetto(float leftAngle, float rightAngle)
{
	m_decodePackagePtr->SetMoveAngleToDuetto(leftAngle, rightAngle);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::SetCorrectedAngleToScope192(float angle1, float angle2, float angle3)
{
	m_decodePackagePtr->SetCorrectionAngleToScope192(angle1, angle2, angle3);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::SetCorrectionAngleToScopeMiniA2_192(float angle1, float angle2, float angle3)
{
	m_decodePackagePtr->SetCorrectionAngleToScopeMiniA2_192(angle1, angle2, angle3);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::SetCorrectedAngleToTSP0332(float angle1, float angle2)
{
	m_decodePackagePtr->SetCorrectionAngleToTSP0332(angle1, angle2);
}

template <typename PointT>
TanwayLidarSDK<PointT>::~TanwayLidarSDK()
{

}

template <typename PointT>
void TanwayLidarSDK<PointT>::RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback)
{
	m_decodePackagePtr->RegPointCloudCallback(callback);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::RegGPSCallback(const std::function<void(const std::string&)>& callback)
{
	m_decodePackagePtr->RegGPSCallback(callback);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::RegIMUDataCallback(const std::function<void(const TWIMUData&)>& callback)
{
	m_decodePackagePtr->RegIMUDataCallback(callback);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::RegExceptionCallback(const std::function<void(const TWException&)>& callback)
{
	if (m_networkReaderPtr)
		m_networkReaderPtr->RegExceptionCallback(callback);
	if (m_pcapReaderPtr)
		m_pcapReaderPtr->RegExceptionCallback(callback);
	m_decodePackagePtr->RegExceptionCallback(callback);
}

template <typename PointT>
void TanwayLidarSDK<PointT>::Start()
{
	if (m_networkReaderPtr)
		m_networkReaderPtr->Start();
	if (m_pcapReaderPtr)
		m_pcapReaderPtr->Start();

	m_decodePackagePtr->Start();
}
