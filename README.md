# TanwayLidarSDK

## 1 介绍

TanwayLidarSDK是探维科技针对所售雷达产品开发的SDK开发包，用于提供给客户直接的点云信息，支持客户的二次开发。

## 2 SDK的下载

Windwos下载地址:https://github.com/TanwayLab/TanwayLidarSDK

Ubuntu可直接通过命令：

```
git clone https://github.com/TanwayLab/TanwayLidarSDK.git
```

下载，或下载压缩包再解压到指定目录。

## 3 编译支持

- ##### Windows平台

  MSVC（VS2015已测试），将开发包的src文件夹和TanwayLidarSDK.h文件直接拷贝至工程中，如果仅使用本SDK进行测试，可以直接将demo文件夹下的Demo_UseSDK.cpp文件导入到工程中，并作为程序入口（demo文件中含有main()入口），进行编译即可；集成到具体项目中使用可直接将文件夹TanwayLidarSDK下除demo文件夹外的源码拷贝至开发项目中，并添加所有.h文件即可使用。

- ##### Ubuntu平台

  g++ (Ubuntu 7.5.0-3ubuntu1~18.04已测试) ，如果仅使用本SDK进行测试，可进入到下载完成后的TanwayLidarSDK目录下，执行make命令，编译成功后执行./run_demo命令，即可运行示例程序；集成到具体项目中可直接使用除demo文件夹外的源码进行开发。

## 4 接口的使用示例

- ##### 连接实时的雷达设备

  定义SDK实例对象，其中PointXYZ为自定义的点结构体类型。如果安装了pcl库或ROS系统可直接使用pcl::XYZ或pcl::XYZI点结构体，然后将雷达IP地址、本机IP、数据接收端口、雷达型号作为参数。

  ```
  TanwayLidarSDK<PointXYZ> lidar("192.168.111.51", "192.168.111.204", 5600, LT_TensorPro);
  ```

  定义点云数据回调函数、GPS数据回调函数、异常信息回调函数，各回调函数均在子线程中运行，请避免在回调函数中进行耗时操作或直接操作UI对象。

  ```
  lidar.RegPointCloudCallback(pointCloudCallback);
  lidar.RegGPSCallback(gpsCallback);
  lidar.RegExceptionCallback(exceptionCallback);
  ```

  启动实例，此时如果雷达正常连接将可以在上面的回调函数中获取到相应的数据。

  ```
  lidar.Start();
  ```

- ##### 回放.pcap文件

  定义SDK实例对象，仅参数与实时连接雷达时不同，其他一致。参数：.pcap文件路径、雷达型号、雷达的IP地址、数据接收端口。

  ```
  TanwayLidarSDK<PointXYZ> lidar("./test.pcap", LT_TensorPro, "192.168.111.51", 5600);
  lidar.RegPointCloudCallback(pointCloudCallback);
  lidar.RegGPSCallback(gpsCallback);
  lidar.RegExceptionCallback(exceptionCallback);
  lidar.Start();
  ```

## 5 历史版本描述

| 版本号 | 更新时间       | 更新内容                                                     |
| ------ | -------------- | ------------------------------------------------------------ |
| v1.0.0 |                |                                                              |
| v1.0.1 | 2021年12月22日 | 增加点云帧时间戳和单点时间戳                                 |
| v1.0.2 | 2022年03月15日 | 增加对TSP03-32、Scope-192雷达的支持                          |
| v1.0.3 | 2022年03月22日 | 修正Scope-192水平角度解算                                    |
| v1.0.4 | 2022年03月26日 | 修正Scope-192水平角度解算水平偏移的bug;<br />修改默认组修正角度值； |
| v1.0.5 | 2022年04月20日 | 增加Duetto设备的支持；                                       |
| v1.0.6 | 2022年06月21日 | 修改设备解析模块中的二次解析接口，增加可扩展性；             |
| v1.0.7 | 2022年09月20日 | 增加ScopeMiniA2-192设备型号的支持；                          |
| v1.0.8 | 2022年10月15日 | 增加Duetto标定参数的接口；<br />修正ScopeMiniA2-192设备默认的组修正角度值； |
