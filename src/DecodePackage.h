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
MemberCheck(block)
MemberCheck(t_sec)
MemberCheck(t_usec)

#define PointT_HasMember(C, member) has_member_##member<C>::value


template <typename PointT>
class DecodePackage
{
protected:
	typedef struct _ori_point_data
	{
		double x;
		double y;
		double z;

		double distance = 0;
		int  channel = 0;
		double angle = 0;
		double pulse = 0;
		int echo = 1;
		int mirror = 0;
		int left_right = 0;
		int block = 0;
		unsigned int t_sec = 0;
		unsigned int t_usec = 0;
	}TWPointData;

public:
	DecodePackage(std::shared_ptr<PackageCache> packageCachePtr, TWLidarType lidarType, std::mutex* mutex);
	DecodePackage();
	virtual ~DecodePackage();

	void Start();
	void RegPointCloudCallback(const std::function<void(typename TWPointCloud<PointT>::Ptr)>& callback);
	void RegGPSCallback(const std::function<void(const std::string&)>& callback);
	void RegIMUDataCallback(const std::function<void(const TWIMUData&)>& callback);
	void RegExceptionCallback(const std::function<void(const TWException&)>& callback);

	void SetPackageCache(std::shared_ptr<PackageCache> packageCachePtr){ m_packageCachePtr = packageCachePtr; }
	void SetLidarType(TWLidarType lidarType){m_lidarType = lidarType;}
	void SetCorrectionAngleToTSP0332(float angle1, float angle2);
	void SetCorrectionAngleToScope192(float angle1, float angle2, float angle3);
	void SetCorrectionAngleToScopeMiniA2_192(float angle1, float angle2, float angle3);
	void SetCorrectKBValueToDuetto(double k, double b);
	void SetMutex(std::mutex* mutex){ m_mutex = mutex; }

private:
	void InitBasicVariables();
	void BeginDecodePackageData();
	

	void DecodeTensorLite(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro_echo2(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeTensorPro0332(char* udpData, unsigned int t_sec, unsigned int t_usec);
	void DecodeScope(char* udpData);
	void DecodeScope192(char* udpData);
	void DecodeScopeMiniA2_192(char* udpData);
	void DecodeDuetto(char* udpData);

	void DecodeGPSData(char* udpData);	//decode gps date

	void DecodeDIFData(char* udpData);
	void DecodeIMUData(char* udpData);


protected:
	virtual void UseDecodeTensorPro(char* udpData, std::vector<TWPointData>& pointCloud);
	virtual void UseDecodeTensorPro_echo2(char* udpData, std::vector<TWPointData>& pointCloud);
	virtual void UseDecodeTensorPro0332(char* udpData, std::vector<TWPointData>& pointCloud);
	virtual void UseDecodeScope(char* udpData, std::vector<TWPointData>& pointCloud);
	virtual void UseDecodeScope192(char* udpData, std::vector<TWPointData>& pointCloud);
	virtual void UseDecodeScopeMiniA2_192(char* udpData, std::vector<TWPointData>& pointCloud);
	virtual void UseDecodeDuetto(char* udpData, std::vector<TWPointData>& pointCloud);

	virtual void ProcessPointCloud(){};

protected:
	int FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low);
	float FourHexToFloat(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low);
	int TwoHextoInt(unsigned char high, unsigned char low);
	int GetDuettoBlockNumber(double angle, int mirror, int lORr);
	bool IsEqualityFloat3(const double value1, const double value2);
	bool IsEqualityFloat5(const double value1, const double value2);

public:
	double m_startAngle = 30.0;
	double m_endAngle = 150.0;
	
protected:
	double m_firstSeparateAngle = -1;
	double m_calRA = (double)(3.14159265f / 180.0f);
	double m_calPulse = 0.004577 / 0.15;
	double m_calSimple = 500 * 2.997924 / 10.f / 16384.f / 2;

	int m_blockNumberForDuetto[6] = {1/*RA*/, 2001/*RB*/, 4001/*RC*/, 3001/*LA*/, 5001/*LB*/, 1001/*LC*/ };

	//Tensor
	double m_verticalChannelsAngle_Tensor16[16] =
	{
		-5.274283f, -4.574258f,	-3.872861f, -3.1703f, -2.466783f, -1.762521f, -1.057726f, -0.352611f,
		0.352611f, 1.057726f, 1.762521f, 2.466783f, 3.1703f, 3.872861f, 4.574258f, 5.274283f
	};

	//TSP03 32
	double m_verticalChannelAngle16_cos_vA_RA[16] = { 0.0 };
	double m_verticalChannelAngle16_sin_vA_RA[16] = { 0.0 };
	double m_skewing_tsp_Angle[3] = {0.0, -6.0};
	double m_skewing_sin_tsp[2] = { 0.0 };
	double m_skewing_cos_tsp[2] = { 0.0 };

	//Scope
	double m_verticalChannelAngle_Scope64[64] =
	{
		-14.64f, -14.17f, -13.69f, -13.22f, -12.75f, -12.28f, -11.81f, -11.34f, -10.87f, -10.40f, -9.93f, -9.47f, -9.00f, -8.54f, -8.07f, -7.61f, -7.14f, -6.68f, -6.22f, -5.76f, -5.29f, -4.83f, -4.37f, -3.91f, -3.45f, -2.99f, -2.53f, -2.07f, -1.61f, -1.15f, -0.69f, -0.23f,
		0.23f, 0.69f, 1.15f, 1.61f, 2.07f, 2.53f, 2.99f, 3.45f, 3.91f, 4.37f, 4.83f, 5.29f, 5.76f, 6.22f, 6.68f, 7.14f, 7.61f, 8.07f, 8.54f, 9.00f, 9.47f, 9.93f, 10.40f, 10.87f, 11.34f, 11.81f, 12.28f, 12.75f, 13.22f, 13.69f, 14.17f, 14.64f
	};
	double m_verticalChannelAngle_Scope64_cos_vA_RA[64] = { 0.0 };
	double m_verticalChannelAngle_Scope64_sin_vA_RA[64] = { 0.0 };
	double m_skewing_scope_Angle[3] = {-0.0, -0.12, -0.24};
	double m_skewing_scope_Angle_Correct[3] = {-0.0, -0.12, -0.24};
	double m_skewing_sin_scope[3] = { 0.0 };
	double m_skewing_cos_scope[3] = { 0.0 };
	double m_rotate_scope_sin = sin(-10.0 * m_calRA);
	double m_rotate_scope_cos = cos(-10.0 * m_calRA);

	//ScopeMiniA2-192
	float m_verticalChannelAngle_Scope64_A2[64] =
	{
		-12.368f, -11.986f, -11.603f, -11.219f, -10.834f, -10.448f, -10.061f, -9.674f, -9.285f, -8.896f, -8.505f, -8.115f, -7.723f, -7.331f, -6.938f, -6.545f, -6.151f, -5.756f, -5.361f, -4.966f, -4.570f, -4.174f, -3.777f, -3.381f, -2.983f, -2.586f, -2.189f, -1.791f, -1.393f, -0.995f, -0.597f, -0.199f,
		0.199f, 0.597f, 0.995f, 1.393f, 1.791f, 2.189f, 2.586f, 2.983f, 3.381f, 3.777f, 4.174f, 4.570f, 4.966f, 5.361f, 5.756f, 6.151f, 6.545f, 6.938f, 7.331f, 7.723f, 8.115f, 8.505f, 8.896f, 9.285f, 9.674f, 10.061f, 10.448f, 10.834f, 11.219f, 11.603f, 11.986f, 12.368f
	};
	double m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[64] = { 0.f };
	double m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[64] = { 0.f };
	double m_skewing_scopeMiniA2_Angle[3] = {0.0, 0.1, 0.2};
	double m_skewing_scopeMiniA2_Angle_Correct[3] = {0.0, 0.1, 0.2};
	double m_skewing_sin_scopeMiniA2_192[3] = { 0.0 };
	double m_skewing_cos_scopeMiniA2_192[3] = { 0.0 };


	//Duetto
	float m_verticalChannelsAngle_Duetto16L[16] =
	{
		-3.75782f, -3.25777f, -2.75729f, -2.25646f, -1.75533f, -1.25397f, -0.75245f, -0.25083f,
		0.250827f, 0.752447f, 1.253969f, 1.755328f, 2.256457f, 2.757293f, 3.25777f, 3.757823f
	};

	float m_verticalChannelsAngle_Duetto16R[16] =
	{
		-3.51152f, -3.00987f, -2.50823f, -2.00658f, -1.50494f, -1.00329f, -0.50165f, 0.0f,
		0.501645f, 1.003224f, 1.504673f, 2.005925f, 2.506916f, 3.00758f, 3.507853f, 4.00767f
	};
	double m_verticalChannelAngle_Duetto16L_cos_vA_RA[16] = { 0 };
	double m_verticalChannelAngle_Duetto16L_sin_vA_RA[16] = { 0 };
	double m_verticalChannelAngle_Duetto16R_cos_vA_RA[16] = { 0 };
	double m_verticalChannelAngle_Duetto16R_sin_vA_RA[16] = { 0 };
	double m_leftMoveAngle = 30.0;
	double m_rightMoveAngle = 30.0;
	double m_rotate_duetto_sinL;
	double m_rotate_duetto_cosL;
	double m_rotate_duetto_sinR;
	double m_rotate_duetto_cosR;
	double m_skewing_duetto_Angle[3] = {-4.5, 0.0, 4.5};
	double m_skewing_sin_duetto[3];
	double m_skewing_cos_duetto[3];
	double duettoPivotVector[3] = {0, 0, 1};
	double m_correction_movement_L[3] = {0.017, 0, 0};
	double m_correction_movement_R[3] = {-0.017, 0, 0};
	double m_kValue = 1.0;
	double m_bValue = 0.0;


private:
	std::shared_ptr<PackageCache> m_packageCachePtr;
	TWLidarType m_lidarType;
	std::atomic<bool>  run_decode;
	std::atomic<bool>  run_exit;
	std::mutex* m_mutex;

	std::function<void(typename TWPointCloud<PointT>::Ptr)> m_funcPointCloud = NULL;
	std::function<void(const std::string&)> m_funcGPS = NULL;
	std::function<void(const TWIMUData&)> m_funcIMU = NULL;

	std::function<void(const TWException&)> m_funcException = NULL;

public:
	typename TWPointCloud<PointT>::Ptr m_pointCloudPtr;
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
void DecodePackage<PointT>::RegIMUDataCallback(const std::function<void(const TWIMUData&)>& callback)
{
	m_funcIMU = callback;
}

template <typename PointT>
void DecodePackage<PointT>::RegExceptionCallback(const std::function<void(const TWException&)>& callback)
{
	m_funcException = callback;
}


template <typename PointT>
void DecodePackage<PointT>::SetCorrectKBValueToDuetto(double k, double b)
{
	m_kValue = k;
	m_bValue = b;
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToScope192(float angle1, float angle2, float angle3)
{
	m_skewing_scope_Angle[0] = angle1;
	m_skewing_scope_Angle[1] = angle2;
	m_skewing_scope_Angle[2] = angle3;

	m_skewing_scope_Angle_Correct[0] = angle1;
	m_skewing_scope_Angle_Correct[1] = angle2;
	m_skewing_scope_Angle_Correct[2] = angle3;

	m_skewing_sin_scope[0] = sin(m_skewing_scope_Angle[0] * m_calRA);
	m_skewing_sin_scope[1] = sin(m_skewing_scope_Angle[1] * m_calRA);
	m_skewing_sin_scope[2] = sin(m_skewing_scope_Angle[2] * m_calRA);

	m_skewing_cos_scope[0] = cos(m_skewing_scope_Angle[0] * m_calRA);
	m_skewing_cos_scope[1] = cos(m_skewing_scope_Angle[1] * m_calRA);
	m_skewing_cos_scope[2] = cos(m_skewing_scope_Angle[2] * m_calRA);
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToScopeMiniA2_192(float angle1, float angle2, float angle3)
{
	m_skewing_scopeMiniA2_Angle[0] = angle1;
	m_skewing_scopeMiniA2_Angle[1] = angle2;
	m_skewing_scopeMiniA2_Angle[2] = angle3;
	m_skewing_scopeMiniA2_Angle_Correct[0] = angle1;
	m_skewing_scopeMiniA2_Angle_Correct[1] = angle2;
	m_skewing_scopeMiniA2_Angle_Correct[2] = angle3;

	m_skewing_sin_scopeMiniA2_192[0] = sin(angle1 * m_calRA);
	m_skewing_sin_scopeMiniA2_192[1] = sin(angle2 * m_calRA);
	m_skewing_sin_scopeMiniA2_192[2] = sin(angle3 * m_calRA);

	m_skewing_cos_scopeMiniA2_192[0] = cos(angle1 * m_calRA);
	m_skewing_cos_scopeMiniA2_192[1] = cos(angle2 * m_calRA);
	m_skewing_cos_scopeMiniA2_192[2] = cos(angle3 * m_calRA);
}

template <typename PointT>
void DecodePackage<PointT>::SetCorrectionAngleToTSP0332(float angle1, float angle2)
{
	m_skewing_tsp_Angle[0] = angle1;
	m_skewing_tsp_Angle[1] = angle2;

	m_skewing_sin_tsp[0] = sin(m_skewing_tsp_Angle[0] * m_calRA);
	m_skewing_sin_tsp[1] = sin(m_skewing_tsp_Angle[1] * m_calRA);  //-6.0

	m_skewing_cos_tsp[0] = cos(m_skewing_tsp_Angle[0] * m_calRA);
	m_skewing_cos_tsp[1] = cos(m_skewing_tsp_Angle[1] * m_calRA);
}

template <typename PointT>
int DecodePackage<PointT>::GetDuettoBlockNumber(double angle, int mirror, int lORr)
{
	if (angle < m_startAngle && 1 == mirror) 
	{
		m_blockNumberForDuetto[0] = 1; 		//RA 1-1000
		m_blockNumberForDuetto[1] = 2001; 	//RB
		m_blockNumberForDuetto[2] = 4001; 	//RC
		m_blockNumberForDuetto[3] = 3001; 	//LA
		m_blockNumberForDuetto[4] = 5001; 	//LB
		m_blockNumberForDuetto[5] = 1001; 	//LC
		return 0;
	}
	else
	{
		return m_blockNumberForDuetto[mirror + lORr*3]++;
	}
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
float DecodePackage<PointT>::FourHexToFloat(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low)
{
	unsigned char sz_float[4] = {low, middle, highmiddle, high};
	float addr = 0;
	memcpy(&addr, sz_float, 4);
	return addr;
}

template <typename PointT>
bool DecodePackage<PointT>::IsEqualityFloat3(const double value1, const double value2)
{
	return fabs(value1 - value2) < 0.001;
}

template <typename PointT>
bool DecodePackage<PointT>::IsEqualityFloat5(const double value1, const double value2)
{
	return fabs(value1 - value2) < 0.00001;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, x)>::type setX(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, x)>::type setX(PointT& point, const float& value)
{
	point.x = value;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, y)>::type setY(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, y)>::type setY(PointT& point, const float& value)
{
	point.y = value;
}


template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, z)>::type setZ(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, z)>::type setZ(PointT& point, const float& value)
{
	point.z = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, intensity)>::type setIntensity(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, intensity)>::type setIntensity(PointT& point, const float& value)
{
	point.intensity = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, channel)>::type setChannel(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, channel)>::type setChannel(PointT& point, const int& value)
{
	point.channel = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, angle)>::type setAngle(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, angle)>::type setAngle(PointT& point, const float& value)
{
	point.angle = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, echo)>::type setEcho(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, echo)>::type setEcho(PointT& point, const int& value)
{
	point.echo = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, color)>::type setColor(PointT& point, const float& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, color)>::type setColor(PointT& point, const float& value)
{
	point.color = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, block)>::type setBlock(PointT& point, const int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, block)>::type setBlock(PointT& point, const int& value)
{
	point.block = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, t_sec)>::type setT_sec(PointT& point, const unsigned int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, t_sec)>::type setT_sec(PointT& point, const unsigned int& value)
{
	point.t_sec = value;
}

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, t_usec)>::type setT_usec(PointT& point, const unsigned int& value)
{
}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, t_usec)>::type setT_usec(PointT& point, const unsigned int& value)
{
	point.t_usec = value;
}

template <typename PointT>
void DecodePackage<PointT>::Start()
{
	run_decode.store(true);
	run_exit.store(false);
	m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
	std::thread(std::bind(&DecodePackage::BeginDecodePackageData, this)).detach();
}

template <typename PointT>
DecodePackage<PointT>::DecodePackage(std::shared_ptr<PackageCache> packageCachePtr, TWLidarType lidarType, std::mutex* mutex): 
	m_packageCachePtr(packageCachePtr), m_lidarType(lidarType), m_mutex(mutex)
{
	InitBasicVariables();
}

template <typename PointT>
DecodePackage<PointT>::DecodePackage()
{
	InitBasicVariables();
}

template <typename PointT>
void DecodePackage<PointT>::InitBasicVariables()
{
	run_decode.store(false);
	run_exit.store(false);

	//Scope-192
	m_skewing_sin_scope[0] = sin(m_skewing_scope_Angle_Correct[0] * m_calRA);
	m_skewing_sin_scope[1] = sin(m_skewing_scope_Angle_Correct[1] * m_calRA);
	m_skewing_sin_scope[2] = sin(m_skewing_scope_Angle_Correct[2] * m_calRA);

	m_skewing_cos_scope[0] = cos(m_skewing_scope_Angle_Correct[0] * m_calRA);
	m_skewing_cos_scope[1] = cos(m_skewing_scope_Angle_Correct[1] * m_calRA);
	m_skewing_cos_scope[2] = cos(m_skewing_scope_Angle_Correct[2] * m_calRA);

	for (int i = 0; i < 64; i++)
	{
		double vA = m_verticalChannelAngle_Scope64[i];
		m_verticalChannelAngle_Scope64_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle_Scope64_sin_vA_RA[i] = sin(vA * m_calRA);
	}

	//Scope-Mini-A2-192
	m_skewing_sin_scopeMiniA2_192[0] = sin(m_skewing_scopeMiniA2_Angle_Correct[0] * m_calRA);
	m_skewing_sin_scopeMiniA2_192[1] = sin(m_skewing_scopeMiniA2_Angle_Correct[1] * m_calRA);
	m_skewing_sin_scopeMiniA2_192[2] = sin(m_skewing_scopeMiniA2_Angle_Correct[2] * m_calRA);

	m_skewing_cos_scopeMiniA2_192[0] = cos(m_skewing_scopeMiniA2_Angle_Correct[0] * m_calRA);
	m_skewing_cos_scopeMiniA2_192[1] = cos(m_skewing_scopeMiniA2_Angle_Correct[1] * m_calRA);
	m_skewing_cos_scopeMiniA2_192[2] = cos(m_skewing_scopeMiniA2_Angle_Correct[2] * m_calRA);

	for (int i = 0; i < 64; i++)
	{
		double vA = m_verticalChannelAngle_Scope64_A2[i];
		m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[i] = sin(vA * m_calRA);
	}

	//TSP03-32
	m_skewing_sin_tsp[0] = sin(m_skewing_tsp_Angle[0] * m_calRA);
	m_skewing_sin_tsp[1] = sin(m_skewing_tsp_Angle[1] * m_calRA);  //-6.0

	m_skewing_cos_tsp[0] = cos(m_skewing_tsp_Angle[0] * m_calRA);
	m_skewing_cos_tsp[1] = cos(m_skewing_tsp_Angle[1] * m_calRA);

	for (int i = 0; i < 16; i++)
	{
		double vA = m_verticalChannelsAngle_Tensor16[i];
		m_verticalChannelAngle16_cos_vA_RA[i] = cos(vA * m_calRA);
		m_verticalChannelAngle16_sin_vA_RA[i] = sin(vA * m_calRA);
	}

	//Duetto
	for (int i = 0; i < 16; i++)
	{
		double vA_L = m_verticalChannelsAngle_Duetto16L[i];
		m_verticalChannelAngle_Duetto16L_cos_vA_RA[i] = cos(vA_L * m_calRA);
		m_verticalChannelAngle_Duetto16L_sin_vA_RA[i] = sin(vA_L * m_calRA);

		double vA_R = m_verticalChannelsAngle_Duetto16R[i];
		m_verticalChannelAngle_Duetto16R_cos_vA_RA[i] = cos(vA_R * m_calRA);
		m_verticalChannelAngle_Duetto16R_sin_vA_RA[i] = sin(vA_R * m_calRA);
	}
	double DuettoA_Elevation = -4.5;
	double DuettoB_Elevation = 0.0;
	double DuettoC_Elevation = 4.5;
	m_skewing_sin_duetto[0] = sin(DuettoA_Elevation * m_calRA);
	m_skewing_sin_duetto[1] = sin(DuettoB_Elevation * m_calRA);
	m_skewing_sin_duetto[2] = sin(DuettoC_Elevation * m_calRA);
	m_skewing_cos_duetto[0] = cos(DuettoA_Elevation * m_calRA);
	m_skewing_cos_duetto[1] = cos(DuettoB_Elevation * m_calRA);
	m_skewing_cos_duetto[2] = cos(DuettoC_Elevation * m_calRA);
	m_rotate_duetto_sinL = sin(m_leftMoveAngle * m_calRA);  //
	m_rotate_duetto_cosL = cos(m_leftMoveAngle * m_calRA);  //
	m_rotate_duetto_sinR = sin(m_rightMoveAngle * m_calRA);  //
	m_rotate_duetto_cosR = cos(m_rightMoveAngle * m_calRA);  //

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
		case LT_Duetto:
			if (packagePtr->m_length == 1348)
				DecodeDuetto(packagePtr->m_szData);
			else if (packagePtr->m_length == 1024)
				DecodeDIFData(packagePtr->m_szData);
			else if (packagePtr->m_length == 128)
				DecodeIMUData(packagePtr->m_szData);
			else
			{
				USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_NOMATCH_DEVICE, "Lidar type and protocol data do not match!");
			}
			break;
		case LT_ScopeMiniA2_192:
			if (packagePtr->m_length == 1120)
				DecodeScopeMiniA2_192(packagePtr->m_szData);
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
void DecodePackage<PointT>::UseDecodeTensorPro(char* udpData, std::vector<TWPointData>& pointCloud)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		unsigned int HextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = HextoAngle * 0.00001;

		unsigned char hexBlockMicrosecond = udpData[offset + 68];
		hexBlockMicrosecond = hexBlockMicrosecond & 0x7F;
		unsigned int blockMicrosecond = FourHexToInt(hexBlockMicrosecond, udpData[offset + 69], udpData[offset + 70], udpData[offset + 71]);

		unsigned char  hexMirror = udpData[offset + 68];
		hexMirror = hexMirror >> 7;
		unsigned short mirror = hexMirror;

		int seq = 0;
		while (seq < 16)
		{
			unsigned short  hexL = TwoHextoInt(udpData[offset + seq * 4 + 0], udpData[offset + seq * 4 + 1]);
			unsigned short  hexPulse = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3]);

			double L = hexL * m_calSimple;
			double pulse = hexPulse * m_calPulse;

			double cos_hA = cos(horizontalAngle * m_calRA);
			double sin_hA = sin(horizontalAngle * m_calRA);
			double vA = m_verticalChannelsAngle_Tensor16[seq];
			double cos_vA_RA = cos(vA * m_calRA);


			TWPointData basic_point;
			basic_point.x = L * cos_vA_RA * cos_hA;
			basic_point.y = L * cos_vA_RA * sin_hA;
			basic_point.z = L * sin(vA * m_calRA);

			basic_point.distance = L;
			basic_point.channel = seq + 1; 
			basic_point.angle = horizontalAngle;
			basic_point.pulse = pulse;
			basic_point.mirror = mirror;
			basic_point.echo = 1;
			basic_point.t_sec = 0;
			basic_point.t_usec = blockMicrosecond;

			pointCloud.push_back(std::move(basic_point));

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodeTensorPro_echo2(char* udpData, std::vector<TWPointData>& pointCloud)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{
		int offset = blocks_num * 72;

		unsigned int HextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = HextoAngle * 0.00001;

		unsigned char hexBlockMicrosecond = udpData[offset + 68];
		hexBlockMicrosecond = hexBlockMicrosecond & 0x7F;
		unsigned int blockMicrosecond = FourHexToInt(hexBlockMicrosecond, udpData[offset + 69], udpData[offset + 70], udpData[offset + 71]);

		unsigned char  hexMirror = udpData[offset + 68];
		hexMirror = hexMirror >> 7;
		unsigned short mirror = hexMirror;

		int seq = 0;
		while (seq < 16)
		{
			unsigned short  hexL = TwoHextoInt(udpData[offset + seq * 4 + 0], udpData[offset + seq * 4 + 1]);
			unsigned short  hexPulse = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3]);

			double L = hexL * m_calSimple;
			double pulse = hexPulse * m_calPulse;

			double cos_hA = cos(horizontalAngle * m_calRA);
			double sin_hA = sin(horizontalAngle * m_calRA);
			double vA = m_verticalChannelsAngle_Tensor16[seq];
			double cos_vA_RA = cos(vA * m_calRA);


			TWPointData basic_point;
			basic_point.x = L * cos_vA_RA * cos_hA;
			basic_point.y = L * cos_vA_RA * sin_hA;
			basic_point.z = L * sin(vA * m_calRA);

			basic_point.distance = L;
			basic_point.channel = seq + 1; 
			basic_point.angle = horizontalAngle;
			basic_point.pulse = pulse;
			basic_point.mirror = mirror;
			basic_point.echo = blocks_num % 2 + 1;
			basic_point.t_sec = 0;
			basic_point.t_usec = blockMicrosecond;
			pointCloud.push_back(std::move(basic_point));

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodeTensorPro0332(char* udpData, std::vector<TWPointData>& pointCloud)
{
	for (int blocks_num = 0; blocks_num < 20; blocks_num++)
	{ 
		int offset = blocks_num * 72;

		unsigned int HextoAngle = FourHexToInt(udpData[offset + 64], udpData[offset + 65], udpData[offset + 66], udpData[offset + 67]);
		double horizontalAngle = HextoAngle * 0.00001;

		unsigned char hexBlockMicrosecond = udpData[offset + 68];
		hexBlockMicrosecond = hexBlockMicrosecond & 0x7F;
		unsigned int blockMicrosecond = FourHexToInt(hexBlockMicrosecond, udpData[offset + 69], udpData[offset + 70], udpData[offset + 71]);

		unsigned char  hexMirror = udpData[offset + 68];
		hexMirror = hexMirror >> 7;
		unsigned short mirror = hexMirror;

		double hA = 0.5 * horizontalAngle * m_calRA;
		double hA_sin = sin(hA);
		double hA_cos = cos(hA);

		unsigned short faceIndex = mirror;
		double x_cal_1 = 2.0 * m_skewing_cos_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos*hA_cos - 1;
		double x_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos;

		double y_cal_1 = m_skewing_cos_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * 2.0 * hA_sin * hA_cos;
		double y_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_sin;

		double z_cal_1 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_cos_tsp[faceIndex] * hA_cos;
		double z_cal_2 = 2.0 * m_skewing_sin_tsp[faceIndex] * m_skewing_sin_tsp[faceIndex] - 1;

		int seq = 0;
		while (seq < 16)
		{
			unsigned short  hexL = TwoHextoInt(udpData[offset + seq * 4 + 0], udpData[offset + seq * 4 + 1]);
			unsigned short  hexPulse = TwoHextoInt(udpData[offset + seq * 4 + 2], udpData[offset + seq * 4 + 3]);

			double L = hexL * m_calSimple;
			double pulse = hexPulse * m_calPulse;

			double cos_vA_RA = m_verticalChannelAngle16_cos_vA_RA[seq];
			double sin_vA_RA = m_verticalChannelAngle16_sin_vA_RA[seq];

			//echo1
			{
				TWPointData basic_point;
				basic_point.x = L * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
				basic_point.y = L * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
				basic_point.z = -L * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

				basic_point.distance = L;
				basic_point.channel = seq + 1; 
				basic_point.angle = horizontalAngle;
				basic_point.pulse = pulse;
				basic_point.mirror = mirror;
				basic_point.echo = 1;
				basic_point.t_sec = 0;
				basic_point.t_usec = blockMicrosecond;
				pointCloud.push_back(std::move(basic_point));
			}

			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodeScope(char* udpData, std::vector<TWPointData>& pointCloud)
{
	double firstSeparateAngle = -1.0;

	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset_block = blocks_num * 140;

		unsigned int HextoAngle = FourHexToInt(udpData[offset_block + 136 - 8], udpData[offset_block + 136 - 7], udpData[offset_block + 136 - 6], udpData[offset_block + 136 - 5]);
		double horizontalAngle = HextoAngle  * 0.00001;

		unsigned int blockSecond = FourHexToInt(udpData[offset_block + 132], udpData[offset_block + 133], udpData[offset_block + 134], udpData[offset_block + 135]);
		unsigned char hexBlockMicrosecond = udpData[offset_block + 137];
		hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
		unsigned int blockMicrosecond = FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset_block + 138], udpData[offset_block + 139]);

		unsigned char  hexMirror = udpData[offset_block + 136];
		hexMirror = hexMirror << 2;
		unsigned short mirror = hexMirror >> 6;

		unsigned char  hexSepIndex = udpData[offset_block + 136];
		unsigned short sepIndex = hexSepIndex >> 6;

		int calIndex = TwoHextoInt(udpData[offset_block + 138], udpData[offset_block + 139]);

		if (sepIndex == 0)
			firstSeparateAngle = horizontalAngle;
		else
		{
			if (firstSeparateAngle >= 0)
				horizontalAngle = firstSeparateAngle;
		}

		int seq = 0;
		while (seq < 16)
		{
			double hexToInt1 = TwoHextoInt(udpData[offset_block + seq * 8 + 0], udpData[offset_block + seq * 8 + 1]);
			double hexPulse1 = TwoHextoInt(udpData[offset_block + seq * 8 + 2], udpData[offset_block + seq * 8 + 3]);
			double L_1 = hexToInt1 * m_calSimple;
			double pulse_1 = hexPulse1 * m_calPulse;


			double hexToInt2 = TwoHextoInt(udpData[offset_block + seq * 8 + 4], udpData[offset_block + seq * 8 + 5]);
			double hexPulse2 = TwoHextoInt(udpData[offset_block + seq * 8 + 6], udpData[offset_block + seq * 8 + 7]);
			double L_2 = hexToInt2 * m_calSimple;
			double pulse_2 = hexPulse2 * m_calPulse; 

			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			double cos_hA = cos(horizontalAngle * m_calRA);
			double sin_hA = sin(horizontalAngle * m_calRA);
			double vA = m_verticalChannelAngle_Scope64[channel - 1];
			double cos_vA_RA = cos(vA * m_calRA);

			//echo1
			{
				double x = L_1 * cos_vA_RA * cos_hA;
				double y = L_1 * cos_vA_RA * sin_hA;
				double z = L_1 * sin(vA * m_calRA);

				TWPointData basic_point;
				basic_point.x = x;
				basic_point.y = y;
				basic_point.z = z;
				basic_point.distance = L_1;
				basic_point.channel = channel; 
				basic_point.angle = horizontalAngle;
				basic_point.pulse = pulse_1;
				basic_point.echo = 1;
				basic_point.mirror = mirror;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;
				pointCloud.push_back(std::move(basic_point));
			}

			//echo2
			/*
			{
				double x = L_2 * cos_vA_RA * cos_hA;
				double y = L_2 * cos_vA_RA * sin_hA;
				double z = L_2 * sin(vA * m_calRA);

				TWPointData basic_point;
				basic_point.x = x;
				basic_point.y = y;
				basic_point.z = z;
				basic_point.distance = L_2;
				basic_point.channel = channel; 
				basic_point.angle = horizontalAngle;
				basic_point.pulse = pulse_1;
				basic_point.echo = 1;
				basic_point.mirror = mirror;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;
				pointCloud.push_back(std::move(basic_point));
			}
			*/
			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodeScope192(char* udpData, std::vector<TWPointData>& pointCloud)
{
	double horizontalAngle = 0;
	//face id
	unsigned short mirror = 0;
	double x_cal_1 = 0.0;
	double x_cal_2 = 0.0;
	double y_cal_1 = 0.0;
	double y_cal_2 = 0.0;
	double z_cal_1 = 0.0;
	double z_cal_2 = 0.0;

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
			mirror = hexMirror >> 6;

			double hA = 0.5 * (horizontalAngle + 10.0) * m_calRA;
			double hA_sin = sin(hA);
			double hA_cos = cos(hA);

			double offsetAngle = 0;
			unsigned char  hexACount = udpData[offset + 136];
			hexACount = hexACount << 4;
			unsigned short uACount = hexACount >> 4;
			offsetAngle = uACount * 0.04 - 0.3;

			//calculate 
			if (mirror < 3 && fabs(m_skewing_scope_Angle_Correct[mirror] - (m_skewing_scope_Angle[mirror] + offsetAngle)) > 0.001)
			{
				m_skewing_scope_Angle_Correct[mirror] = m_skewing_scope_Angle[mirror] + offsetAngle;
				m_skewing_sin_scope[mirror] = sin(-1.0 * m_skewing_scope_Angle_Correct[mirror] * m_calRA);
				m_skewing_cos_scope[mirror] = cos(-1.0 * m_skewing_scope_Angle_Correct[mirror] * m_calRA);
			}

			x_cal_1 = 2.0 * m_skewing_cos_scope[mirror] * m_skewing_cos_scope[mirror] * hA_cos*hA_cos - 1;
			x_cal_2 = 2.0 * m_skewing_sin_scope[mirror] * m_skewing_cos_scope[mirror] * hA_cos;

			y_cal_1 = 2.0 * m_skewing_cos_scope[mirror] * m_skewing_cos_scope[mirror] * hA_sin * hA_cos;
			y_cal_2 = 2.0 * m_skewing_sin_scope[mirror] * m_skewing_cos_scope[mirror] * hA_sin;

			z_cal_1 = 2.0 * m_skewing_sin_scope[mirror] * m_skewing_cos_scope[mirror] * hA_cos;
			z_cal_2 = 2.0 * m_skewing_sin_scope[mirror] * m_skewing_sin_scope[mirror] - 1;
		}

		unsigned int blockSecond = FourHexToInt(udpData[offset + 132], udpData[offset + 133], udpData[offset + 134], udpData[offset + 135]);
		unsigned char hexBlockMicrosecond = udpData[offset + 137];
		hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
		unsigned int blockMicrosecond = FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset + 138], udpData[offset + 139]);

		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			double hexToInt1 = TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]);
			double hexPulse1 = TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]);
			double L_1 = hexToInt1 * m_calSimple;
			double pulse_1 = hexPulse1 * m_calPulse; 


			double hexToInt2 = TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]);
			double hexPulse2 = TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]);
			double L_2 = hexToInt2 * m_calSimple;
			double pulse_2 = hexPulse2 * m_calPulse;

			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			double cos_vA_RA = m_verticalChannelAngle_Scope64_cos_vA_RA[channel - 1];
			double sin_vA_RA = m_verticalChannelAngle_Scope64_sin_vA_RA[channel - 1];

			//echo1
			{
				DecodePackage::TWPointData basic_point;
				basic_point.angle = horizontalAngle;
				basic_point.mirror = mirror;
				basic_point.channel = channel;

				double x_tmp = L_1 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
				double y_tmp = L_1 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
				double z_tmp = -L_1 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);
				basic_point.x = x_tmp * m_rotate_scope_cos - y_tmp * m_rotate_scope_sin;
				basic_point.y = x_tmp * m_rotate_scope_sin + y_tmp * m_rotate_scope_cos;
				basic_point.z = z_tmp;
				
				basic_point.echo = 1;
				basic_point.distance = L_1;
				basic_point.pulse = pulse_1;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;

				pointCloud.push_back(std::move(basic_point));
			}

			//echo2
			/*
			{
				DecodePackage::TWPointData basic_point;
				basic_point.angle = horizontalAngle;
				basic_point.mirror = mirror;
				basic_point.channel = channel;

				double x_tmp = L_2 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
				double y_tmp = L_2 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
				double z_tmp = -L_2 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);
				basic_point.x = x_tmp * m_rotate_scope_cos - y_tmp * m_rotate_scope_sin;
				basic_point.y = x_tmp * m_rotate_scope_sin + y_tmp * m_rotate_scope_cos;
				basic_point.z = z_tmp;

				basic_point.echo = 2;
				basic_point.distance = L_2;
				basic_point.pulse = pulse_2;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;
				
				pointCloud.push_back(std::move(basic_point));
			}
			*/
			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodeScopeMiniA2_192(char* udpData, std::vector<TWPointData>& pointCloud)
{
	double horizontalAngle = 0;
	//face id
	unsigned short mirror = 0;
	double x_cal_1 = 0.0;
	double x_cal_2 = 0.0;
	double y_cal_1 = 0.0;
	double y_cal_2 = 0.0;
	double z_cal_1 = 0.0;
	double z_cal_2 = 0.0;
	
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
			mirror = hexMirror >> 6;

			//offset angle m_skewing_scopeMiniA2_angle 
			double offsetAngle = 0;
			unsigned char  hexACount = udpData[offset + 136];
			hexACount = hexACount << 4;
			unsigned short uACount = hexACount >> 4;
			offsetAngle = uACount * 0.04 - 0.3;

			//calculate 
			if (mirror < 3 && fabs(m_skewing_scopeMiniA2_Angle_Correct[mirror] - (m_skewing_scopeMiniA2_Angle[mirror] + offsetAngle)) > 0.001)
			{
				m_skewing_scopeMiniA2_Angle_Correct[mirror] = m_skewing_scopeMiniA2_Angle[mirror] + offsetAngle;
				m_skewing_sin_scopeMiniA2_192[mirror] = sin(m_skewing_scopeMiniA2_Angle_Correct[mirror] * m_calRA);
				m_skewing_cos_scopeMiniA2_192[mirror] = cos(m_skewing_scopeMiniA2_Angle_Correct[mirror] * m_calRA);
			}

			double hA = 0.5 * (horizontalAngle) * m_calRA;
			double hA_sin = sin(hA);
			double hA_cos = cos(hA);

			x_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[mirror] * m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos*hA_cos - 1;
			x_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] * m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos;

			y_cal_1 = 2.0 * m_skewing_cos_scopeMiniA2_192[mirror] * m_skewing_cos_scopeMiniA2_192[mirror] * hA_sin * hA_cos;
			y_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] * m_skewing_cos_scopeMiniA2_192[mirror] * hA_sin;

			z_cal_1 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] * m_skewing_cos_scopeMiniA2_192[mirror] * hA_cos;
			z_cal_2 = 2.0 * m_skewing_sin_scopeMiniA2_192[mirror] * m_skewing_sin_scopeMiniA2_192[mirror] - 1;
		}

		unsigned int blockSecond = FourHexToInt(udpData[offset + 132], udpData[offset + 133], udpData[offset + 134], udpData[offset + 135]);
		unsigned char hexBlockMicrosecond = udpData[offset + 137];
		hexBlockMicrosecond = hexBlockMicrosecond & 0x0F;
		unsigned int blockMicrosecond = FourHexToInt(0x00, hexBlockMicrosecond, udpData[offset + 138], udpData[offset + 139]);

		//separate index
		unsigned char  hexSepIndex = udpData[offset + 136];
		unsigned short sepIndex = hexSepIndex >> 6;

		int seq = 0;
		while (seq < 16)
		{
			double hexToInt1 = TwoHextoInt(udpData[offset + seq * 8 + 0], udpData[offset + seq * 8 + 1]);
			double hexPulse1 = TwoHextoInt(udpData[offset + seq * 8 + 2], udpData[offset + seq * 8 + 3]);
			double L_1 = hexToInt1 * m_calSimple;
			double pulse_1 = hexPulse1 * m_calPulse; 

			double hexToInt2 = TwoHextoInt(udpData[offset + seq * 8 + 4], udpData[offset + seq * 8 + 5]);
			double hexPulse2 = TwoHextoInt(udpData[offset + seq * 8 + 6], udpData[offset + seq * 8 + 7]);
			double L_2 = hexToInt2 * m_calSimple;
			double pulse_2 = hexPulse2 * m_calPulse;

			int channel = 65 - (16 * (blocks_num >= 4 ? blocks_num - 4 : blocks_num) + seq + 1);

			double cos_vA_RA = m_verticalChannelAngle_ScopeMiniA2_cos_vA_RA[channel - 1];
			double sin_vA_RA = m_verticalChannelAngle_ScopeMiniA2_sin_vA_RA[channel - 1];

			//echo1
			{
				DecodePackage::TWPointData basic_point;
				basic_point.angle = horizontalAngle;
				basic_point.mirror = mirror;
				basic_point.channel = channel;

				basic_point.x = L_1 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
				basic_point.y = L_1 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
				basic_point.z = -L_1 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);
				
				basic_point.echo = 1;
				basic_point.distance = L_1;
				basic_point.pulse = pulse_1;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;

				pointCloud.push_back(std::move(basic_point));
			}

			//echo2
			/*
			{
				DecodePackage::TWPointData basic_point;
				basic_point.angle = horizontalAngle;
				basic_point.mirror = mirror;
				basic_point.channel = channel;

				basic_point.x = L_2 * (cos_vA_RA * x_cal_1 + sin_vA_RA * x_cal_2);
				basic_point.y = L_2 * (cos_vA_RA * y_cal_1 + sin_vA_RA * y_cal_2);
				basic_point.z = -L_2 * (cos_vA_RA * z_cal_1 + sin_vA_RA * z_cal_2);

				basic_point.echo = 2;
				basic_point.distance = L_2;
				basic_point.pulse = pulse_2;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;
				
				pointCloud.push_back(std::move(basic_point));
			}
			*/
			seq++;
		}
	}
}

template <typename PointT>
void DecodePackage<PointT>::UseDecodeDuetto(char* udpData, std::vector<TWPointData>& pointCloud)
{
	//ptp
	unsigned int frameSecond = FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
	double frameMicrosecond = FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

	double sin_gamma1 = m_rotate_duetto_sinL;
	double cos_gamma1 = m_rotate_duetto_cosL;
	double sin_gamma2 = m_rotate_duetto_sinR;
	double cos_gamma2 = m_rotate_duetto_cosR;

	double nx = duettoPivotVector[0];
	double ny = duettoPivotVector[1];
	double nz = duettoPivotVector[2];

	for (int blocks_num = 0; blocks_num < 8; blocks_num++)
	{
		int offset_block = blocks_num * 164;

		int offsetMicrosecond = TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
		double totalMicrosecond = frameMicrosecond + offsetMicrosecond*0.1;
		unsigned int blockSecond = (totalMicrosecond >= 1000000)? (frameSecond+1) : frameSecond;
		unsigned int blockMicrosecond = (totalMicrosecond >= 1000000)? (unsigned int)(totalMicrosecond - 1000000) : (unsigned int)totalMicrosecond;

		//L/R
		unsigned char  hexLOrR = udpData[35 + offset_block];
		hexLOrR = hexLOrR << 7;
		unsigned short leftOrRight = hexLOrR >> 7; //0:右；1:左
		//mirror
		unsigned char  hexMirror = udpData[35 + offset_block];
		hexMirror = hexMirror << 5;
		unsigned short mirror = hexMirror >> 6;
		//block 
		double hexBlockAngle = TwoHextoInt(udpData[36 + offset_block + 0 * 10], udpData[37 + offset_block + 0 * 10]);
		double blockAngle = hexBlockAngle * 0.01;
		int block = GetDuettoBlockNumber(blockAngle, mirror, leftOrRight);
		//
		double cos_delta = m_skewing_cos_duetto[mirror];
		double sin_delta = m_skewing_sin_duetto[mirror];

		for (int seq = 0; seq < 16; seq++)
		{
			double hexHorAngle = TwoHextoInt(udpData[36 + offset_block + seq * 10], udpData[37 + offset_block + seq * 10]);
			double horAngle = hexHorAngle * 0.01;
			
			//x、y、z
			double x_t = 0, y_t = 0, z_t = 0;
			double x_move = 0, y_move = 0, z_move = 0;
			//
			if (1 == leftOrRight)
			{
				double mp_angle = (210.0 - horAngle) *0.5 + 240;
				double sin_theta = sin(mp_angle * m_calRA);
				double cos_theta = cos(mp_angle * m_calRA);

				x_move = m_correction_movement_L[0];
				y_move = m_correction_movement_L[1];
				z_move = m_correction_movement_L[2];

				double cos_beta = m_verticalChannelAngle_Duetto16L_cos_vA_RA[seq];
				double sin_beta = m_verticalChannelAngle_Duetto16L_sin_vA_RA[seq];

				double Tr11 = nx * nx * (1 - cos_theta) + cos_theta;
				double Tr12 = nx * ny * (1 - cos_theta) + nz * sin_theta;
				double Tr13 = nx * nz * (1 - cos_theta) - ny * sin_theta;
				double Tr21 = nx * ny * (1 - cos_theta) - nz * sin_theta;
				double Tr22 = ny * ny * (1 - cos_theta) + cos_theta;
				double Tr23 = ny * nz * (1 - cos_theta) + nx * sin_theta;
				double Tr31 = nx * nz * (1 - cos_theta) + ny * sin_theta;
				double Tr32 = ny * nz * (1 - cos_theta) - nx * sin_theta;
				double Tr33 = nz * nz * (1 - cos_theta) + cos_theta;

				double Nx = cos_delta * cos_gamma1 * Tr11 - cos_delta * sin_gamma1 * Tr12 + sin_delta * Tr13;
				double Ny = cos_delta * cos_gamma1 * Tr21 - cos_delta * sin_gamma1 * Tr22 + sin_delta * Tr23;
				double Nz = cos_delta * cos_gamma1 * Tr31 - cos_delta * sin_gamma1 * Tr32 + sin_delta * Tr33;

				x_t = cos_beta * (-cos_gamma1 + 2 * cos_gamma1 * Nx * Nx - 2 * sin_gamma1 * Nx * Ny) - sin_beta * (2 * Nx * Nz);
				y_t = cos_beta * (sin_gamma1 + 2 * cos_gamma1 * Nx * Ny - 2 * sin_gamma1 * Ny * Ny) - sin_beta * (2 * Ny * Nz);
				z_t = cos_beta * (2 * cos_gamma1 * Nx * Nz - 2 * sin_gamma1 * Ny * Nz) + sin_beta * (1 - 2 * Nz * Nz);
			}
			else
			{
				double mp_angle = (210.0 - horAngle) *0.5;
				double sin_theta = sin(mp_angle * m_calRA);
				double cos_theta = cos(mp_angle * m_calRA);

				x_move = m_correction_movement_R[0];
				y_move = m_correction_movement_R[1];
				z_move = m_correction_movement_R[2];

				double cos_beta = m_verticalChannelAngle_Duetto16R_cos_vA_RA[seq];
				double sin_beta = m_verticalChannelAngle_Duetto16R_sin_vA_RA[seq];

				double Tr11 = nx * nx * (1 - cos_theta) + cos_theta;
				double Tr12 = nx * ny * (1 - cos_theta) + nz * sin_theta;
				double Tr13 = nx * nz * (1 - cos_theta) - ny * sin_theta;
				double Tr21 = nx * ny * (1 - cos_theta) - nz * sin_theta;
				double Tr22 = ny * ny * (1 - cos_theta) + cos_theta;
				double Tr23 = ny * nz * (1 - cos_theta) + nx * sin_theta;
				double Tr31 = nx * nz * (1 - cos_theta) + ny * sin_theta;
				double Tr32 = ny * nz * (1 - cos_theta) - nx * sin_theta;
				double Tr33 = nz * nz * (1 - cos_theta) + cos_theta;

				double Nx = cos_delta * cos_gamma2 * Tr11 + cos_delta * sin_gamma2 * Tr12 - sin_delta * Tr13;
				double Ny = cos_delta * cos_gamma2 * Tr21 + cos_delta * sin_gamma2 * Tr22 - sin_delta * Tr23;
				double Nz = cos_delta * cos_gamma2 * Tr31 + cos_delta * sin_gamma2 * Tr32 - sin_delta * Tr33;

				x_t = cos_beta * (cos_gamma2 - 2 * cos_gamma2 * Nx * Nx - 2 * sin_gamma2 * Nx * Ny) - sin_beta * (2 * Nx * Nz);
				y_t = cos_beta * (sin_gamma2 - 2 * cos_gamma2 * Nx * Ny - 2 * sin_gamma2 * Ny * Ny) - sin_beta * (2 * Ny * Nz);
				z_t = -cos_beta * (2 * cos_gamma2 * Nx * Nz + 2 * sin_gamma2 * Ny * Nz) + sin_beta * (1 - 2 * Nz * Nz);
			}

			double hexL1 = TwoHextoInt(udpData[40 + offset_block + seq * 10], udpData[41 + offset_block + seq * 10]);
			double L_1 = hexL1 * 0.005;
			unsigned char hexChar1 = udpData[42 + offset_block + seq * 10];
			unsigned short hexPulse1 = hexChar1;
			double pulse_1 = hexPulse1 * 0.25;

			double hexL2 = TwoHextoInt(udpData[43 + offset_block + seq * 10], udpData[44 + offset_block + seq * 10]);
			double L_2 = hexL2 * 0.005; //
			unsigned char hexChar2 = udpData[45 + offset_block + seq * 10];
			unsigned short hexPulse2 = hexChar2;
			double pulse_2 = hexPulse2 * 0.25;


			DecodePackage::TWPointData basic_point;
			basic_point.angle = horAngle;
			basic_point.mirror = mirror;
			basic_point.left_right = leftOrRight;
			basic_point.channel = 48 * leftOrRight + (abs(mirror - 2) * 16 + (16 - seq));
			basic_point.block = block;

			//echo 1
			{
				basic_point.x = L_1 * x_t + x_move;
				basic_point.y = L_1 * y_t + y_move;
				basic_point.z = L_1 * z_t + z_move;

				basic_point.distance = L_1;
				int tmpIntensity = (int)((m_kValue * pulse_1 + m_bValue)* 256.0 * 0.06667 + 0.5);
				basic_point.pulse = tmpIntensity > 255 ? 255:tmpIntensity;
				basic_point.echo = 1;/*  */
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;

				pointCloud.push_back(std::move(basic_point));
			}

			//echo2
			/*
			{
				basic_point.x = L_2 * x_t + x_move;
				basic_point.y = L_2 * y_t + y_move;
				basic_point.z = L_2 * z_t + z_move;

				basic_point.distance = L_2;
				int tmpIntensity = (int)((m_kValue * pulse_2 + m_bValue)* 256.0 * 0.06667 + 0.5);
				basic_point.pulse = tmpIntensity > 255 ? 255:tmpIntensity;
				basic_point.echo = 2;
				basic_point.t_sec = blockSecond;
				basic_point.t_usec = blockMicrosecond;

				pointCloud.push_back(std::move(basic_point));
			}
			*/
		}
	}
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
void DecodePackage<PointT>::DecodeDIFData(char* udpData)
{
	//ABC
	int hex8_mirrorA = (unsigned char)udpData[508 + 4 * 14 + 1];
	int hex8_mirrorB = (unsigned char)udpData[508 + 4 * 14 + 2];
	int hex8_mirrorC = (unsigned char)udpData[508 + 4 * 14 + 3];

	double mirrorA = (hex8_mirrorA - 128) * 0.01 + (-4.5);
	double mirrorB = (hex8_mirrorB - 128) * 0.01 + (0);
	double mirrorC = (hex8_mirrorC - 128) * 0.01 + (4.5);

	//
	unsigned short hexMoveAngleL = TwoHextoInt(udpData[508 + 4 * 31 + 0], udpData[508 + 4 * 31 + 1]);
	unsigned short hexMoveAngleR = TwoHextoInt(udpData[508 + 4 * 31 + 2], udpData[508 + 4 * 31 + 3]);
	double moveAngleL = hexMoveAngleL * 0.01;
	double moveAngleR = hexMoveAngleR * 0.01;

	//NX NY NZ
	int hexPivotVectorX = TwoHextoInt(udpData[508 + 4 * 48 + 0], udpData[508 + 4 * 48 + 1]);
	int hexPivotVectorY = TwoHextoInt(udpData[508 + 4 * 48 + 2], udpData[508 + 4 * 48 + 3]);
	int hexPivotVectorZ = TwoHextoInt(udpData[508 + 4 * 49 + 2], udpData[508 + 4 * 49 + 3]);
	double pivotVectorX = (hexPivotVectorX - 32768) * 0.0001;
	double pivotVectorY = (hexPivotVectorY - 32768) * 0.0001;
	double pivotVectorZ = (hexPivotVectorZ - 32768) * 0.0001;

	if (!IsEqualityFloat3(mirrorA, m_skewing_duetto_Angle[0]))
	{
		m_skewing_duetto_Angle[0] = mirrorA;
		m_skewing_sin_duetto[0] = sin(m_skewing_duetto_Angle[0] * m_calRA);
		m_skewing_cos_duetto[0] = cos(m_skewing_duetto_Angle[0] * m_calRA);
		//std::cout << "Mirror: A, " << mirrorA << std::endl;
	}
	if (!IsEqualityFloat3(mirrorB, m_skewing_duetto_Angle[1]))
	{
		m_skewing_duetto_Angle[1] = mirrorB;
		m_skewing_sin_duetto[1] = sin(m_skewing_duetto_Angle[1] * m_calRA);
		m_skewing_cos_duetto[1] = cos(m_skewing_duetto_Angle[1] * m_calRA);
		//std::cout << "Mirror: B, " << mirrorB << std::endl;
	}
	if (!IsEqualityFloat3(mirrorC, m_skewing_duetto_Angle[2]))
	{
		m_skewing_duetto_Angle[2] = mirrorC;
		m_skewing_sin_duetto[2] = sin(m_skewing_duetto_Angle[2] * m_calRA);
		m_skewing_cos_duetto[2] = cos(m_skewing_duetto_Angle[2] * m_calRA);
		//std::cout << "Mirror: C, " << mirrorC << std::endl;
	}

	if (!IsEqualityFloat3(moveAngleL, m_leftMoveAngle))
	{						
		m_leftMoveAngle = moveAngleL;
		m_rotate_duetto_sinL = sin(m_leftMoveAngle * m_calRA); //
		m_rotate_duetto_cosL = cos(m_leftMoveAngle * m_calRA); //
		//std::cout << "MoveAngle: L, " << moveAngleL << std::endl;
	}
	if (!IsEqualityFloat3(moveAngleR, m_rightMoveAngle))
	{
		m_rightMoveAngle = moveAngleR;
		m_rotate_duetto_sinR = sin(m_rightMoveAngle * m_calRA); //
		m_rotate_duetto_cosR = cos(m_rightMoveAngle * m_calRA); //
		//std::cout << "MoveAngle: R, " << moveAngleR << std::endl;
	}

	if (!IsEqualityFloat5(pivotVectorX, duettoPivotVector[0]))
	{
		duettoPivotVector[0] = pivotVectorX;
		//std::cout << "PivotVector: X, " << pivotVectorX << std::endl;
	}
	if (!IsEqualityFloat5(pivotVectorY, duettoPivotVector[1]))
	{
		duettoPivotVector[1] = pivotVectorY;
		//std::cout << "PivotVector: Y, " << pivotVectorY << std::endl;
	}
	if (!IsEqualityFloat5(pivotVectorZ, duettoPivotVector[2]))
	{
		duettoPivotVector[2] = pivotVectorZ;
		//std::cout << "PivotVector: Z, " << pivotVectorZ << std::endl;
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeIMUData(char* udpData)
{
	TWIMUData imu_data;

	//time
	unsigned int t_sec = FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
	double t_usec = FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

	imu_data.stamp = (uint64_t)((t_sec) * 1000 * 1000 + t_usec);

	//status calibrate
	unsigned char  hex_status = udpData[35];
	hex_status = hex_status << 7;
	unsigned short leftOrRight = hex_status >> 7; 
	imu_data.calibrate = (0==leftOrRight) ? false:true;

	//temperature
	imu_data.temperature = (float)((TwoHextoInt(udpData[36], udpData[37]) - 32768) * (1.0/256));

	//angular velocity
	imu_data.angular_velocity[0] = (float)((TwoHextoInt(udpData[38], udpData[39]) - 32768) * 0.0005);
	imu_data.angular_velocity[1] = (float)((TwoHextoInt(udpData[40], udpData[41]) - 32768) * 0.0005);
	imu_data.angular_velocity[2] = (float)((TwoHextoInt(udpData[42], udpData[43]) - 32768) * 0.0005);

	//linear acceleration
	imu_data.linear_acceleration[0] = (float)((TwoHextoInt(udpData[44], udpData[45]) - 32768) * 0.0002);
	imu_data.linear_acceleration[1] = (float)((TwoHextoInt(udpData[46], udpData[47]) - 32768) * 0.0002);
	imu_data.linear_acceleration[2] = (float)((TwoHextoInt(udpData[48], udpData[49]) - 32768) * 0.0002);

	imu_data.gyro_noise = FourHexToFloat(udpData[50], udpData[51], udpData[52], udpData[53]);
	imu_data.gyro_bias = FourHexToFloat(udpData[54], udpData[55], udpData[56], udpData[57]);

	imu_data.accel_noise = FourHexToFloat(udpData[58], udpData[59], udpData[60], udpData[61]);
	imu_data.accel_bias = FourHexToFloat(udpData[62], udpData[63], udpData[64], udpData[65]);

	if (m_funcIMU) m_funcIMU(imu_data);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorLite(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	DecodeTensorPro(udpData, t_sec, t_usec);
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeTensorPro(udpData, pointData);

	int pointSize = pointData.size();
	for (int i = 0; i < pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];

		if (oriPoint.angle < m_startAngle && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle < m_startAngle || oriPoint.angle > m_endAngle) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);

		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro_echo2(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeTensorPro_echo2(udpData, pointData);

	int pointSize = pointData.size();
	for (int i = 0; i < pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];

		if (oriPoint.angle < m_startAngle && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle <m_startAngle || oriPoint.angle > m_endAngle) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);

		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeTensorPro0332(char* udpData, unsigned int t_sec, unsigned int t_usec)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeTensorPro0332(udpData, pointData);

	int pointSize = pointData.size();
	for (int i = 0; i < pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];

		if (oriPoint.angle < m_startAngle && 0 == oriPoint.mirror && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle <m_startAngle || oriPoint.angle > m_endAngle) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);

		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScope(char* udpData)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeScope(udpData, pointData);

	int pointSize = pointData.size();
	for (int i = 0; i < pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];

		if (oriPoint.angle < m_startAngle && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle <m_startAngle || oriPoint.angle > m_endAngle) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);

		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScope192(char* udpData)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeScope192(udpData, pointData);

	int pointSize = pointData.size();
	for (int i = 0; i < pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];

		if (oriPoint.angle < m_startAngle && 0 == oriPoint.mirror && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle <m_startAngle || oriPoint.angle > m_endAngle) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);

		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeDuetto(char* udpData)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeDuetto(udpData, pointData);

	int pointSize = pointData.size();
	for (int i=0; i<pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];
		
		if (oriPoint.angle < m_startAngle && 1 == oriPoint.mirror && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle <m_startAngle || oriPoint.angle > m_endAngle || oriPoint.distance <= 0) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setBlock(basic_point, oriPoint.block);
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);
		
		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}

template <typename PointT>
void DecodePackage<PointT>::DecodeScopeMiniA2_192(char* udpData)
{
	std::vector<DecodePackage::TWPointData> pointData;
	pointData.reserve(300);

	UseDecodeScopeMiniA2_192(udpData, pointData);

	int pointSize = pointData.size();
	for (int i = 0; i < pointSize; i++)
	{
		const DecodePackage::TWPointData& oriPoint = pointData[i];

		if (oriPoint.angle < m_startAngle && 0 == oriPoint.mirror && m_pointCloudPtr->Size() != 0)
		{
			m_pointCloudPtr->height = 1;
			m_pointCloudPtr->width = m_pointCloudPtr->Size();
			unsigned int last_sec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_sec;
			unsigned int last_usec = m_pointCloudPtr->m_pointData[m_pointCloudPtr->width - 1].t_usec;
			m_pointCloudPtr->stamp = (uint64_t)(last_sec) * 1000 * 1000 + last_usec;

			std::lock_guard<std::mutex> lock(*m_mutex);
			if (m_funcPointCloud) m_funcPointCloud(m_pointCloudPtr);

			//create
			m_pointCloudPtr = std::make_shared<TWPointCloud<PointT>>();
			m_pointCloudPtr->Reserve(10000);
			continue;
		}

		if (oriPoint.angle <m_startAngle || oriPoint.angle > m_endAngle) continue;

		if (oriPoint.distance <= 0) continue;

		PointT basic_point;
		setX(basic_point, static_cast<float>(oriPoint.x));
		setY(basic_point, static_cast<float>(oriPoint.y));
		setZ(basic_point, static_cast<float>(oriPoint.z));
		setIntensity(basic_point, static_cast<float>(oriPoint.pulse));
		setChannel(basic_point, oriPoint.channel);
		setAngle(basic_point, static_cast<float>(oriPoint.angle));
		setEcho(basic_point, oriPoint.echo);
		setColor(basic_point, static_cast<float>(oriPoint.distance));
		setT_sec(basic_point, oriPoint.t_sec);
		setT_usec(basic_point, oriPoint.t_usec);

		m_pointCloudPtr->PushBack(std::move(basic_point));
	}
}
