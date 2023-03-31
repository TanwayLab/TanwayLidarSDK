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

#include <draco/io/point_cloud_io.h>
#include <draco/compression/encode.h>
#include <draco/io/file_utils.h>
#include <draco/io/stdio_file_writer.h>
#include <draco/io/stdio_file_reader.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>


#include "../TanwayLidarSDK.h"

//point struct
struct PointXYZ
{
	float x;
	float y;
	float z;
	float intensity;
	//int channel;
	//unsigned int t_sec;         /* The value represents seconds since 1900-01-01 00:00:00 (the UNIX epoch).*/ 
	//unsigned int t_usec;        /* remaining microseconds */
};

bool EncodeDracoPointcloud(const TWPointCloud<PointXYZ>::Ptr pointCloud, std::string fileOutput)
{
	int pointCount = pointCloud->Size();
	std::unique_ptr<draco::PointCloud> dracoPointCloud(new draco::PointCloud());
	dracoPointCloud->set_num_points(pointCount);

	draco::GeometryAttribute::Type attType = draco::GeometryAttribute::POSITION;//设置压缩类型
	draco::PointAttribute att;//创建压缩属性
	att.Init(attType, 4, draco::DT_FLOAT32, false, sizeof(float) * 4);//初始化对象
	int att_id = dracoPointCloud->AddAttribute(att, true, pointCount);//添加到dracoPointCloud中
	draco::PointAttribute *attPtr = dracoPointCloud->attribute(att_id);//获取添加的对象

	for (draco::PointIndex i(0); i < pointCount; i++)
	{
		std::vector<float> pointData(4);
		pointData[0] = (pointCloud->m_pointData)[i.value()].x;
		pointData[1] = (pointCloud->m_pointData)[i.value()].y;
		pointData[2] = (pointCloud->m_pointData)[i.value()].z;
		pointData[3] = (pointCloud->m_pointData)[i.value()].intensity;

		attPtr->SetAttributeValue(attPtr->mapped_index(i), &pointData[0]);//顶点数据添加到压缩属性att中
	}

	//创建压缩对象
	draco::Encoder encoder;
	//设置压缩参数
	const int dracoCompressionSpeed = 7; //压缩速度
	const int dracoPositionBits = 14; //压缩质量
	encoder.SetSpeedOptions(dracoCompressionSpeed, dracoCompressionSpeed);
	encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, dracoPositionBits);
	encoder.SetEncodingMethod(draco::PointCloudEncodingMethod::POINT_CLOUD_KD_TREE_ENCODING);//压缩方式

																							 //开始压缩
	draco::EncoderBuffer dracoBuffer;//压缩后的数据存储到draco的缓冲区中
	const draco::Status statusEncoder = encoder.EncodePointCloudToBuffer(*dracoPointCloud, &dracoBuffer);
	//压缩结果
	if (!statusEncoder.ok())
	{
		std::cout << "Error: Encode pointcloud.\n";
		return false;
	}
	std::cout << "encode success!\n";
	std::cout << "encode buffer size: " << dracoBuffer.size() << std::endl;//压缩后的数据大小

	{
		//输出到文件中
		std::unique_ptr<draco::FileWriterInterface> writerPtr = draco::StdioFileWriter::Open(fileOutput);

		if (!writerPtr->Write(dracoBuffer.data(), dracoBuffer.size()))
		{
			std::cout << "Failed to write the output file.\n";
			return false;
		}
	}
	return true;
}

TWPointCloud<PointXYZ>::Ptr DecodeDracoPointcloud(std::string fileInput)
{
	TWPointCloud<PointXYZ>::Ptr pointCloud = std::make_shared<TWPointCloud<PointXYZ>>();
	//读取文件
	draco::DecoderBuffer decodeBuffer;
	std::unique_ptr<draco::FileReaderInterface> readerPtr = draco::StdioFileReader::Open("E:/record/32/1.trc");
	std::vector<char> buffer(readerPtr->GetFileSize());
	if (!readerPtr->ReadFileToBuffer(&buffer))
	{
		std::cout << "Failed to read the input file.\n";
		return nullptr;
	}
	decodeBuffer.Init(&buffer[0], buffer.size());


	draco::Decoder decoder;
	//设置压缩参数
	draco::StatusOr<std::unique_ptr<draco::PointCloud>> statusPointCloudPtr = decoder.DecodePointCloudFromBuffer(&decodeBuffer);
	if (!statusPointCloudPtr.ok())
	{
		std::cout << "Error: Decode pointcloud.\n";
		return nullptr;
	}
	std::unique_ptr<draco::PointCloud> pointCloudPtr = std::move(statusPointCloudPtr).value();
	const int attID = pointCloudPtr->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
	int pointCount = pointCloudPtr->num_points();
	draco::PointAttribute *attPtr = pointCloudPtr->attribute(attID);
	for (draco::PointIndex i(0); i < pointCount; i++)
	{
		int attCount = attPtr->num_components(); //应该是4
		float point[4] = { 0 };
		attPtr->GetMappedValue(i, point);

		PointXYZ pt;
		pt.x = point[0];
		pt.y = point[1];
		pt.z = point[2];
		pt.intensity = point[3];
		pointCloud->PushBack(pt);
	}
	return pointCloud;
}

void pointCloudCallback(TWPointCloud<PointXYZ>::Ptr pointCloud)
{
	/*
	*The point cloud struct uses a smart pointer. 
	*Please copy the point cloud data to another thread for use.
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << "width:" << pointCloud->width 
			  << " height:" << pointCloud->height 
			  << " point cloud size: " << pointCloud->Size() << std::endl;

	//重新定义脉宽值为map索引值
	for (int i = 0; i < pointCloud->Size(); i++)
	{
		pointCloud->m_pointData[i].intensity = i*0.01f;
	}

	
	
	//压缩处理
	bool enRet = EncodeDracoPointcloud(pointCloud, "E:/record/32/1.trc");

	//解压缩处理
	TWPointCloud<PointXYZ>::Ptr pointCloudPtr = DecodeDracoPointcloud("E:/record/32/1.trc");

	//测试验证：对原始点云数据写文件
	{
		//排序
		std::map<double, PointXYZ> mapPointCloud;
		for (int i = 0; i < pointCloud->Size(); i++)
		{
			mapPointCloud[pointCloud->m_pointData[i].intensity]
				= pointCloud->m_pointData[i];
		}
		//对排序后的点输出到文件
		std::ofstream fileWrite_encode;
		fileWrite_encode.open("./pointcloud_encode.txt", std::ios_base::out | std::ios_base::trunc);
		for (auto itor = mapPointCloud.begin(); itor != mapPointCloud.end(); itor++)
		{
			fileWrite_encode
				<< std::fixed << std::setw(10) << std::setprecision(5)
				<< itor->second.x << ","
				<< itor->second.y << ","
				<< itor->second.z << ","
				<< itor->second.intensity << std::endl;
		}
		fileWrite_encode.flush();
		fileWrite_encode.close();
	}

	//测试验证：对压缩并解压缩完成后的点云数据写文件
	{
		//对点排序
		std::map<double, PointXYZ> mapPointCloud;
		for (int i = 0; i < pointCloudPtr->Size(); i++)
		{
			mapPointCloud[pointCloudPtr->m_pointData[i].intensity]
				= pointCloudPtr->m_pointData[i];
		}

		//对排序后的点输出到文件
		std::ofstream fileWrite_decode;
		fileWrite_decode.open("./pointcloud_decode.txt", std::ios_base::out | std::ios_base::trunc);
		for (auto itor= mapPointCloud.begin(); itor != mapPointCloud.end(); itor++)
		{
			fileWrite_decode
				<< std::fixed << std::setw(10) << std::setprecision(5)
				<< itor->second.x << ","
				<< itor->second.y << ","
				<< itor->second.z << ","
				<< itor->second.intensity << std::endl;
		}

		fileWrite_decode.flush();
		fileWrite_decode.close();
	}
	

	system("pause");
	
}

void gpsCallback(std::string gps_value)
{
	/*
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << gps_value << std::endl;
}

void exceptionCallback(const TWException& exception)
{
	/* 
	*This callback function is called when the SDK sends a tip or raises an exception problem.
	*Use another thread to respond to the exception to avoid time-consuming operations.
	*/
	if (exception.GetErrorCode() > 0)
		std::cout << "[Error Code]: " << exception.GetErrorCode() << " -> " << exception.ToString() << std::endl;
	if (exception.GetTipsCode() > 0)
		std::cout << "[Tips Code]: " << exception.GetTipsCode() << " -> " << exception.ToString() << std::endl;	
}

int main()
{
	/*
	*Real-time connection using lidar. 
	*If you have PCL installed, you can also use the point type 'pcl::PointXYZI' like this 'TanwayLidarSDK<pcl::PointXYZI> lidar';
	*/
	//TanwayLidarSDK<PointXYZ> lidar("192.168.111.51", "192.168.111.204", 5600, LT_TSP0332);
	//lidar.RegPointCloudCallback(pointCloudCallback);
	//lidar.RegGPSCallback(gpsCallback);
	//lidar.RegExceptionCallback(exceptionCallback);
	//lidar.Start();

	/*
	*using pcap file to replay.
	*/
	TanwayLidarSDK<PointXYZ> lidar("E:/record/32/tsp32_distrub2.pcap", "192.168.111.51", 5600, LT_TSP0332, true);
	lidar.RegPointCloudCallback(pointCloudCallback);
	lidar.RegGPSCallback(gpsCallback);
	lidar.RegExceptionCallback(exceptionCallback);
	lidar.SetCorrectedAngleToScope192(0.0f, 0.1f, 0.2f);
	lidar.SetCorrectedAngleToTSP0332(0.0f, -6.0f);
	lidar.Start();


	////quit
	//while (true)
	//{
	//	std::this_thread::sleep_for(std::chrono::seconds(1));
	//}


	std::chrono::time_point<std::chrono::system_clock> beginPlayTime = std::chrono::system_clock::now();

	while (1)
	{
		//std::this_thread::sleep_until(beginPlayTime);
		std::this_thread::sleep_for(std::chrono::microseconds(0));

		//std::chrono::time_point<std::chrono::system_clock> endPlayTime = std::chrono::system_clock::now();
		//std::chrono::duration<double, std::micro> tm = endPlayTime - beginPlayTime;
		//double tm_count = tm.count();
		//if (tm_count >= 100000)
		//{
		//	std::chrono::time_point<std::chrono::system_clock> beginPlayTime2 = std::chrono::system_clock::now();
		//	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(beginPlayTime2.time_since_epoch());
		//	long long  timestamp = tmp.count();
		//	std::cout << "-----------timestamp:" << tm.count() << "   " << timestamp << std::endl;//

		//	//std::cout << "-----------timestamp:" << tm.count() << std::endl;
		//	std::this_thread::sleep_for(std::chrono::microseconds(100));
		//	beginPlayTime = endPlayTime;
		//}
		//if (tm_count < 80000)
		//{
		//	std::this_thread::sleep_for(std::chrono::microseconds(1));
		//}

		//beginPlayTime += std::chrono::microseconds(100001);
		//
		//std::cout << "----------------" << GetCurrentTimeStamp(1) << std::endl;


		//std::chrono::time_point<std::chrono::system_clock> beginPlayTime2 = std::chrono::system_clock::now();
		//auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(beginPlayTime2.time_since_epoch());
		//std::time_t timestamp = tmp.count(); //从1970-01-01 00:00:00到当前时间点的时长
		//std::cout << "-----------timestamp:" << timestamp  << "   " << timestamp - timestamp0 << std::endl;//微妙数
		//timestamp0 = timestamp;

		//beginPlayTime = std::chrono::system_clock::now();
		
	}

    return 0;
}

