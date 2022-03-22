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


#define MemberCheck(member) \
template<typename T>\
struct has_member_##member{\
    template <typename _T>static auto check(_T)->typename std::decay<decltype(_T::member)>::type;\
    static void check(...);\
    using type=decltype(check(std::declval<T>()));\
    enum{value=!std::is_void<type>::value};\
};

MemberCheck(x)
MemberCheck(y)
MemberCheck(z)
MemberCheck(intensity)
MemberCheck(distance)
MemberCheck(channel)
MemberCheck(angle)
MemberCheck(echo)
MemberCheck(sepIndex)
MemberCheck(faceIndex)
MemberCheck(color)
MemberCheck(t_sec)
MemberCheck(t_usec)

#define PointT_HsaMember(C, member) has_member_##member<C>::value


template <typename PointT>
class DecodePackage
{
public:
	DecodePackage(std::shared_ptr<PackageCache> packageCachePtr, TWLidarType lidarType, std::mutex* mutex);
	DecodePackage(){};
	virtual ~DecodePackage();

	void Start();
	void RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback);
	void RegGPSCallback(const std::function<void(const std::string&)>& callback);
	void RegExceptionCallback(const std::function<void(const TWException&)>& callback);

	void SetPackageCache(std::shared_ptr<PackageCache> packageCachePtr){ m_packageCachePtr = packageCachePtr; }
	void SetLidarType(TWLidarType lidarType){m_lidarType = lidarType;}
	void SetCorrectionAngleToTSP0332(float angle1, float angle2);
	void SetCorrectionAngleToScope192(float angle1, float angle2, float angle3);
	void SetMutex(std::mutex* mutex){ m_mutex = mutex; }

private:
	void BeginDecodePackageData();
	

	void DecodeTensorLite(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro_echo2(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeScope(char* udpData);
	void DecodeTensorPro0332(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeScope192(char* udpData);

	void DecodeGPSData(char* udpData);	//decode gps date


protected:
	virtual void UseDecodePointPro(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec);
	virtual void UseDecodePointTSP03_32(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec);
	virtual void UseDecodePointScope(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth);
	virtual void UseDecodePointScope_192(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth);
	virtual void ProcessPointCloud(){};

protected:
	int FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low);
	int TwoHextoInt(unsigned char high, unsigned char low);

public:
	double m_startAngle = 30.0;
	double m_endAngle = 150.0;
	
protected:
	double m_firstSeparateAngle = -1;
	double m_calRA = (float)(3.14159265f / 180.0f);
	double m_calPulse = 0.004577 / 0.15;
	double m_calSimple = 500 * 2.997924 / 10.f / 16384.f / 2;

	//TSP03-32、Scope162 temporary variable
	double x_cal_1 = 0;
	double x_cal_2 = 0;
	double y_cal_1 = 0;
	double y_cal_2 = 0;
	double z_cal_1 = 0;
	double z_cal_2 = 0;

	//Tensor
	double m_verticalChannelAngle16[16] =
	{
		-5.274283f, -4.574258f,	-3.872861f, -3.1703f, -2.466783f, -1.762521f, -1.057726f, -0.352611f,
		0.352611f, 1.057726f, 1.762521f, 2.466783f, 3.1703f, 3.872861f, 4.574258f, 5.274283f
	};
	double m_verticalChannelAngle16_cos_vA_RA[16] = { 0.0 };
	double m_verticalChannelAngle16_sin_vA_RA[16] = { 0.0 };
	double m_skewing_sin_tsp[2] = { 0.0 };
	double m_skewing_cos_tsp[2] = { 0.0 };

	//Scope
	double m_verticalChannelAngle64[64] =
	{
		-14.64f, -14.17f, -13.69f, -13.22f, -12.75f, -12.28f, -11.81f, -11.34f, -10.87f, -10.40f, -9.93f, -9.47f, -9.00f, -8.54f, -8.07f, -7.61f, -7.14f, -6.68f, -6.22f, -5.76f, -5.29f, -4.83f, -4.37f, -3.91f, -3.45f, -2.99f, -2.53f, -2.07f, -1.61f, -1.15f, -0.69f, -0.23f,
		0.23f, 0.69f, 1.15f, 1.61f, 2.07f, 2.53f, 2.99f, 3.45f, 3.91f, 4.37f, 4.83f, 5.29f, 5.76f, 6.22f, 6.68f, 7.14f, 7.61f, 8.07f, 8.54f, 9.00f, 9.47f, 9.93f, 10.40f, 10.87f, 11.34f, 11.81f, 12.28f, 12.75f, 13.22f, 13.69f, 14.17f, 14.64f
	};
	double m_verticalChannelAngle64_cos_vA_RA[64] = { 0.0 };
	double m_verticalChannelAngle64_sin_vA_RA[64] = { 0.0 };
	double m_skewing_sin_scope[3] = { 0.0 };
	double m_skewing_cos_scope[3] = { 0.0 };

private:
	std::shared_ptr<PackageCache> m_packageCachePtr;
	TWLidarType m_lidarType;
	std::atomic<bool>  run_decode;
	std::atomic<bool>  run_exit;
	std::mutex* m_mutex;

	std::function<void(typename TWPointCloud<PointT>::Ptr)> m_funcPointCloud = NULL;
	std::function<void(const std::string&)> m_funcGPS = NULL;
	std::function<void(const TWException&)> m_funcException = NULL;

public:
	typename TWPointCloud<PointT>::Ptr m_pointCloutPtr;
};

template <typename PointT>
void DecodePackage<PointT>::RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback)
{
	m_funcPointCloud = callback;
}

template <typename PointT>
void DecodePackage<PointT>::RegGPSCallback(const std::function<void(const std::string&)>& callback)
{
	m_funcGPS = callback;
}

template <typename PointT>
void DecodePackage<PointT>::RegExceptionCallback(const std::function<void(const TWException&)>& callback)
{
	m_funcException = callback;
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToScope192(float angle1, float angle2, float angle3)
{
	m_skewing_sin_scope[0] = sin(angle1 * m_calRA);
	m_skewing_sin_scope[1] = sin(angle2 * m_calRA);
	m_skewing_sin_scope[2] = sin(angle3 * m_calRA);

	m_skewing_cos_scope[0] = cos(angle1 * m_calRA);
	m_skewing_cos_scope[1] = cos(angle2 * m_calRA);
	m_skewing_cos_scope[2] = cos(angle3 * m_calRA);
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToTSP0332(float angle1, float angle2)
{
	m_skewing_sin_tsp[0] = sin(angle1 * m_calRA);
	m_skewing_sin_tsp[1] = sin(angle2 * m_calRA);  //-6.0

	m_skewing_cos_tsp[0] = cos(angle1 * m_calRA);
	m_skewing_cos_tsp[1] = cos(angle2 * m_calRA);
}

template <typename PointT>
int DecodePackage<PointT>::TwoHextoInt(unsigned char high, unsigned char low)
{
	int addr = low & 0xFF;
	addr |= ((high << 8) & 0XFF00);
	return addr;
}

template <typename PointT>
int DecodePackage<PointT>::FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low)
{
	int addr = low & 0xFF;
	addr |= ((middle << 8) & 0xFF00);
	addr |= ((highmiddle << 16) & 0xFF0000);
	addr |= ((high << 24) & 0xFF000000);
	return addr;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, x)>::type setX(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, x)>::type setX(PointT& point, const float& value)
{
	point.x = value;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, y)>::type setY(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, y)>::type setY(PointT& point, const float& value)
{
	point.y = value;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, z)>::type setZ(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, z)>::type setZ(PointT& point, const float& value)
{
	point.z = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, intensity)>::type setIntensity(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, intensity)>::type setIntensity(PointT& point, const float& value)
{
	point.intensity = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, channel)>::type setChannel(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, channel)>::type setChannel(PointT& point, const int& value)
{
	point.channel = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, angle)>::type setAngle(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, angle)>::type setAngle(PointT& point, const float& value)
{
	point.angle = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, echo)>::type setEcho(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, echo)>::type setEcho(PointT& point, const int& value)
{
	point.echo = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, color)>::type setColor(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, color)>::type setColor(PointT& point, const float& value)
{
	point.color = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, t_sec)>::type setT_sec(PointT& point, const unsigned int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, t_sec)>::type setT_sec(PointT& point, const unsigned int& value)
{
	point.t_sec = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HsaMember(PointT, t_usec)>::type setT_usec(PointT& point, const unsigned int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HsaMember(PointT, t_usec)>::type setT_usec(PointT& point, const unsigned int& value)
{
	point.t_usec = value;
}

template <typename PointT>
void DecodePackage<PointT>::Start()
{
	run_decode.store(true);
	run_exit.store(false);
	m_pointCloutPtr = std::make_shared<TWPointCloud<PointT>>();
	std::thread(std::bind(&DecodePackage::BeginDecodePackageData, this)).detach();
}

template <typename PointT>
DecodePackage<PointT>::DecodePackage(std::shared_ptr<PackageCache> packageCachePtr, TWLidarType lidarType, std::mutex* mutex): 
	m_packageCachePtr(packageCachePtr), m_lidarType(lidarType), m_mutex(mutex)
{
	run_decode.store(false);
	run_exit.store(false);

	//Scope-192
	double ScopeB_Elevation_A = 0.1;
	double ScopeC_Elevation_A = 0.2;
	m_skewing_sin_scope[0] = sin(0.0 * m_calRA);
	m_skewing_sin_scope[1] = sin(ScopeB_Elevation_A * m_calRA);
	m_skewing_sin_scope[2] = sin(ScopeC_Elevation_A * m_calRA);

	m_skewing_cos_scope[0] = cos(0.0 * m_calRA);
	m_skewing_cos_scope[1] = cos(ScopeB_Elevation_A * m_calRA);
	m_skewing_cos_scope[2] = cos(ScopeC_Elevation_A * m_calRA);

	for (int i = 0; i < 64; i++)
	{
		double vA = m_verticalChannelAngle64[i];
		m_verticalChannelAngle64_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle64_sin_vA_RA[i] = sin(vA * m_calRA);
	}

	//TSP03-32
	m_skewing_sin_tsp[0] = sin(0.0 * m_calRA);
	m_skewing_sin_tsp[1] = sin(-6.0 * m_calRA);  //-6.0

	m_skewing_cos_tsp[0] = cos(0.0 * m_calRA);
	m_skewing_cos_tsp[1] = cos(-6.0 * m_calRA);

	for (int i = 0; i < 16; i++)
	{
		//计算
		double vA = m_verticalChannelAngle16[i];
		m_verticalChannelAngle16_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle16_sin_vA_RA[i] = sin(vA * m_calRA);
	}

}

template <typename PointT>
DecodePackage<PointT>::~DecodePackage()
{
	//std::this_thread::sleep_for(std::chrono::seconds(1));
	run_decode.store(false);

	while (!run_exit)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

template <typename PointT>
void DecodePackage<PointT>::BeginDecodePackageData()
{
	run_exit.store(false);
	while (run_decode)
	{
		if (m_packageCachePtr->Size() <= 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		TWUDPPackage::Ptr packagePtr = m_packageCachePtr->PopFrontPackage();

		if (packagePtr->m_length == 120)
		{
			DecodeGPSData(packagePtr->m_szData);
			continue;
		}

		switch (m_lidarType)
		{
		case LT_TensorLite:
			if (packagePtr->m_length == 1440)
				DecodeTensorLite(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_TensorPro:
			if (packagePtr->m_length == 1440)
				DecodeTensorPro(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_TensorPro_echo2:
			if (packagePtr->m_length == 1440)
				DecodeTensorPro_echo2(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_TSP0332:
			if (packagePtr->m_length == 1440)
				DecodeTensorPro0332(packagePtr->m_szData, packagePtr->t_sec, packagePtr->t_usec);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_Scope:
			if (packagePtr->m_length == 1120)
				DecodeScope(packagePtr->m_szData);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_Scope192:
			if (packagePtr->m_length == 1120)
				DecodeScope192(packagePtr->m_szData);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		default:
		{
			USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_INVALID_DEVICE, "Invalid device type!");
		}
			break;
		}

	}
	run_exit.store(true);
	USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_EXIT_DECODE, std::string("The decode package thread has exited!"));
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointPro(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec)
{
	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double cos_hA = cos(horAngle * m_calRA);
	double sin_hA = sin(horAngle * m_calRA);

	double vA = m_verticalChannelAngle16[channel-1];
	double cos_vA_RA = cos(vA * m_calRA);
	double x = L * cos_vA_RA * cos_hA;
	double y = L * cos_vA_RA * sin_hA;
	double z = L * sin(vA * m_calRA);

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	setT_sec(basic_point, t_sec);
	setT_usec(basic_point, t_usec);

	m_pointCloutPtr->PushBack(basic_point);

}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointTSP03_32(int echo, double horAngle, int channel, float hexL, float hexPulseWidth, int offset, char* data, unsigned int t_sec, unsigned int t_usec)
{
	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double x = L * (m_verticalChannelAngle16_cos_vA_RA[channel - 1] * x_cal_1 + m_verticalChannelAngle16_sin_vA_RA[channel - 1] * x_cal_2);
	double y = L * (m_verticalChannelAngle16_cos_vA_RA[channel - 1] * y_cal_1 + m_verticalChannelAngle16_sin_vA_RA[channel - 1] * y_cal_2);
	double z = -L * (m_verticalChannelAngle16_cos_vA_RA[channel - 1] * z_cal_1 + m_verticalChannelAngle16_sin_vA_RA[channel - 1] * z_cal_2);

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	setT_sec(basic_point, t_sec);
	setT_usec(basic_point, t_usec);

	m_pointCloutPtr->PushBack(basic_point);
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointScope(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth)
{
	if (sepIndex == 0)
		m_firstSeparateAngle = horAngle;
	else
	{
		if (m_firstSeparateAngle >= 0)
			horAngle = m_firstSeparateAngle;
	}

	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double cos_hA = cos(horAngle * m_calRA);
	double sin_hA = sin(horAngle * m_calRA);

	double vA = m_verticalChannelAngle64[channel - 1];
	double cos_vA_RA = cos(vA * m_calRA);
	double x = L * cos_vA_RA * cos_hA;
	double y = L * cos_vA_RA * sin_hA;
	double z = L * sin(vA * m_calRA);


	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);

	m_pointCloutPtr->PushBack(basic_point);
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodePointScope_192(int echo, int sepIndex, int faceIndex, double horAngle, int channel, float hexL, float hexPulseWidth)
{
	if (sepIndex == 0)
		m_firstSeparateAngle = horAngle;
	else
	{
		if (m_firstSeparateAngle >= 0)
			horAngle = m_firstSeparateAngle;
	}

	//distance
	double L = hexL * m_calSimple;
	if (L <= 0 || L > 300) return;

	double intensity = hexPulseWidth * m_calPulse;

	double x = L * (m_verticalChannelAngle64_cos_vA_RA[channel - 1] * x_cal_1 + m_verticalChannelAngle64_sin_vA_RA[channel - 1] * x_cal_2);
	double y = L * (m_verticalChannelAngle64_cos_vA_RA[channel - 1] * y_cal_1 + m_verticalChannelAngle64_sin_vA_RA[channel - 1] * y_cal_2);
	double z = -L * (m_verticalChannelAngle64_cos_vA_RA[channel - 1] * z_cal_1 + m_verticalChannelAngle64_sin_vA_RA[channel - 1] * z_cal_2);

	PointT basic_point;
	setX(basic_point, static_cast<float>(x));
	setY(basic_point, static_cast<float>(y));
	setZ(basic_point, static_cast<float>(z));
	setIntensity(basic_point, static_cast<float>(intensity));
	setChannel(basic_point, channel);
	setAngle(basic_point, static_cast<float>(horAngle));
	setEcho(basic_point, echo);
	//setT_sec(basic_point, t_sec);
	//setT_usec(basic_point, t_usec);

	m_pointCloutPtr->PushBack(basic_point);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeGPSData(char* udpData)
{
	//gps string
	std::string gps_value = "";
	//status valid(0x41);  invalid(0x56)
	std::string gps_status = "GPS STATUS: (Unknown)";
	bool bUnknown = false;
	if (udpData[3] == 0x41)
		gps_status = "GPS STATUS: (Valid)";
	else if (udpData[3] == 0x56)
		gps_status = "GPS STATUS: (Invalid)";
	else
	{
		gps_status = "GPS STATUS: (Unknown)";
		bUnknown = true;
	}

	//GPS time info
	//                   0    1    2    3    4    5    6   7    8    9    10   11   12   13   14   15   16   17   18    19
	char sz_gps[20] = { '2', '0', ' ', ' ', '-', ' ', ' ', '-', ' ', ' ', ' ', ' ', ' ', ':', ' ', ' ', ':', ' ', ' ', '\0' };

	sz_gps[2] = bUnknown ? '#' : udpData[4]; //year
	sz_gps[3] = bUnknown ? '#' : udpData[5];
	sz_gps[5] = bUnknown ? '#' : udpData[6]; //month
	sz_gps[6] = bUnknown ? '#' : udpData[7];
	sz_gps[8] = bUnknown ? '#' : udpData[8]; //day
	sz_gps[9] = bUnknown ? '#' : udpData[9];

	sz_gps[11] = bUnknown ? '#' : udpData[10]; //hour
	sz_gps[12] = bUnknown ? '#' : udpData[11];
	sz_gps[14] = bUnknown ? '#' : udpData[12]; //minute
	sz_gps[15] = bUnknown ? '#' : udpData[13];
	sz_gps[17] = bUnknown ? '#' : udpData[14]; //second
	sz_gps[18] = bUnknown ? '#' : udpData[15];

	//GPS time
	//unsigned int time_us = FourHexToInt(udpData[16], udpData[17], udpData[18], udpData[19]);

	if (bUnknown)
		gps_value = gps_status;
	else
		gps_value = gps_status + std::string("  ") + std::string(sz_gps);

	std::lock_guard<std::mutex> lock(*m_mutex);
	if (m_funcGPS) m_funcGPS(gps_value);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorLite(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	DecodeTensorPro(udpData, t_sec, t_usec);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		//horizontal angle
		int hextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = hextoAngle * 0.00001;

		if (horizontalAngle < m_startAngle && m_pointCloutPtr->Size() != 0)
		{
			m_pointCloutPtr->height = 1;
			m_pointCloutPtr->width = m_pointCloutPtr->Size();
			m_pointCloutPtr->stamp = (uint64_t)t_sec * 1000 * 1000 + t_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloutPtr);

			//create
			m_pointCloutPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloutPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;

		int seq = 0;
		while (seq < 16)
		{
			//distance
			float hexL = TwoHextoInt(udpData[offset + seq * 4], udpData[offset + seq * 4 + 1])*1.0f;
			//
			float hexPlusWidth = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3])*1.0f;

			//channel 1-16
			int channel = seq + 1;

			//time of point
			unsigned int cur_sec = t_sec;
			unsigned int cur_usec = t_usec;
			if (t_usec < blocks_num * 29.4) //The time interval between the two columns is 29.4 subtle
			{
				cur_sec = cur_usec - 1;
				cur_usec = (unsigned int)(t_usec + 1000000 - blocks_num * 29.4);
			}
			else
			{
				cur_usec = (unsigned int)(t_usec - blocks_num * 29.4);
			}

			//using
			UseDecodePointPro(1, horizontalAngle, channel, hexL, hexPlusWidth, offset, udpData, cur_sec, cur_usec);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro_echo2(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		//horizontal angle
		int hextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = hextoAngle * 0.00001;

		if (horizontalAngle < m_startAngle && m_pointCloutPtr->Size() != 0)
		{
			m_pointCloutPtr->height = 1;
			m_pointCloutPtr->width = m_pointCloutPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloutPtr);

			//create
			m_pointCloutPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloutPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;

		int seq = 0;
		while (seq < 16)
		{
			//distance
			float hexL = TwoHextoInt(udpData[offset + seq * 4], udpData[offset + seq * 4 + 1])*1.0f;
			//
			float hexPlusWidth = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3])*1.0f;

			//channel: 1-16
			int channel = seq + 1;

			//using
			UseDecodePointPro(blocks_num%2 + 1, horizontalAngle, channel, hexL, hexPlusWidth, offset, udpData, t_sec, t_usec);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro0332(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		//horizontal angle
		int hextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = hextoAngle * 0.00001;
		//face index
		unsigned char  hexMirror = udpData[offset + 68];
		hexMirror = hexMirror >> 7;
		unsigned short faceIndex = hexMirror;

		if (horizontalAngle < m_startAngle && 0 == faceIndex && m_pointCloutPtr->Size() != 0)
		{
			m_pointCloutPtr->height = 1;
			m_pointCloutPtr->width = m_pointCloutPtr->Size();
			m_pointCloutPtr->stamp = (uint64_t)t_sec * 1000 * 1000 + t_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloutPtr);

			//create
			m_pointCloutPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloutPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle < m_startAngle || horizontalAngle > m_endAngle) continue;

		//
		double hA = 0.5 * horizontalAngle * m_calRA;
		double hA_sin = sin(hA);
		double hA_cos = cos(hA);

		x_cal_1 = 2.0 * m_skewing_cos_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos*hA_cos - 1;
		x_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos;

		y_cal_1 = 2.0 * m_skewing_cos_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_sin * hA_cos;
		y_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_sin;

		z_cal_1 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos;
		z_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_sin_tsp[faceIndex] - 1;


		int seq = 0;
		while (seq < 16)
		{
			//distance
			float hexL = TwoHextoInt(udpData[offset + seq * 4], udpData[offset + seq * 4 + 1])*1.0f;
			//
			float hexPlusWidth = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3])*1.0f;

			//channel 1-16
			int channel = seq + 1;

			//time of point
			unsigned int cur_sec = t_sec;
			unsigned int cur_usec = t_usec;
			if (t_usec < blocks_num * 29.4) //The time interval between the two columns is 29.4 subtle
			{
				cur_sec = cur_usec - 1;
				cur_usec = (unsigned int)(t_usec + 1000000 - blocks_num * 29.4);
			}
			else
			{
				cur_usec = (unsigned int)(t_usec - blocks_num * 29.4);
			}

			//using
			UseDecodePointTSP03_32(1, horizontalAngle, channel, hexL, hexPlusWidth, offset, udpData, cur_sec, cur_usec);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScope(char* udpData)
{
	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset = blocks_num * 140;

		//horizontal angle index: 128-131
		int hextoAngle = FourHexToInt(udpData[offset + 128], udpData[offset + 129], udpData[offset + 130], udpData[offset + 131]);
		double horizontalAngle = hextoAngle * 0.00001;

		if (horizontalAngle < m_startAngle && m_pointCloutPtr->Size() != 0)
		{
			m_pointCloutPtr->height = 1;
			m_pointCloutPtr->width = m_pointCloutPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloutPtr);

			//create
			m_pointCloutPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloutPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;


		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;
		//face id
		unsigned char  hexFaceIndex = udpData[offset + 136];
		hexFaceIndex = hexFaceIndex << 2;
		unsigned short faceIndex = hexFaceIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			//echo 1
			float hexL1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]));
			float hexPulseWidth1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]));

			//echo 2
			float hexL2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]));
			float hexPulseWidth2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]));

			//channel
			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			//using
			UseDecodePointScope(1, sepIndex, faceIndex, horizontalAngle, channel, hexL1, hexPulseWidth1);
			UseDecodePointScope(2, sepIndex, faceIndex, horizontalAngle, channel, hexL2, hexPulseWidth2);

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScope192(char* udpData)
{
	double horizontalAngle = 0;
	//face id
	unsigned short faceIndex = 0;

	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset = blocks_num * 140;
		if (0 == blocks_num || 4 == blocks_num)
		{
			//horizontal angle index: 128-131
			int HextoAngle = FourHexToInt(udpData[offset + 128], udpData[offset + 129], udpData[offset + 130], udpData[offset + 131]);
			horizontalAngle = HextoAngle  * 0.00001;

			unsigned char  hexMirror = udpData[offset + 136];
			hexMirror = hexMirror << 2;
			faceIndex = hexMirror >> 6;

			double hA = 0.5 * (horizontalAngle + 10.0) * m_calRA;
			double hA_sin = sin(hA);
			double hA_cos = cos(hA);

			x_cal_1 = 2.0 * m_skewing_cos_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_cos*hA_cos - 1;
			x_cal_2 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_cos;

			y_cal_1 = 2.0 * m_skewing_cos_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_sin * hA_cos;
			y_cal_2 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_sin;

			z_cal_1 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_cos_scope[faceIndex] * hA_cos;
			z_cal_2 = 2.0 * m_skewing_sin_scope[faceIndex] * m_skewing_sin_scope[faceIndex] - 1;
		}

		if (horizontalAngle < m_startAngle && 0 == faceIndex && m_pointCloutPtr->Size() != 0)
		{
			m_pointCloutPtr->height = 1;
			m_pointCloutPtr->width = m_pointCloutPtr->Size();

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloutPtr);

			//create
			m_pointCloutPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloutPtr->Reserve(360);
			continue;
		}
		if (horizontalAngle <m_startAngle || horizontalAngle > m_endAngle) continue;


		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			//echo 1
			float hexL1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]));
			float hexPulseWidth1 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]));

			//echo 2
			float hexL2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]));
			float hexPulseWidth2 = static_cast<float>(TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]));

			//channel
			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			//using
			UseDecodePointScope_192(1, sepIndex, faceIndex, horizontalAngle, channel, hexL1, hexPulseWidth1);
			UseDecodePointScope_192(2, sepIndex, faceIndex, horizontalAngle, channel, hexL2, hexPulseWidth2);

			seq++;
		}
	}
}
