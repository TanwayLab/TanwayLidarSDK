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
#include "PcapReader.h"
#include <fstream>
#include "CommonDefine.h"


PcapReader::PcapReader(std::string filePath, std::string lidarIP, int localPort, int localDIFPort, PackageCache& packageCache, bool repeat, std::mutex* mutex)
	: m_pacpPath(filePath), m_lidarIP(lidarIP), m_localPort(localPort), m_localDIFPort(localDIFPort), m_packageCache(packageCache), m_repeat(repeat), m_mutex(mutex)
{
	m_pause.store(false);
	run_read.store(false);
	run_exit.store(false);
}


PcapReader::~PcapReader()
{
	run_read.store(false);

	while (!run_exit)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void PcapReader::RereadPcap(std::string pcapPath)
{
	run_read.store(false);

	while (!run_exit)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	m_pacpPath = pcapPath;
	Start();
}

void PcapReader::PausePcap(bool pause)
{
	m_pause.store(pause);
}

void PcapReader::Start()
{
	run_read.store(true);
	run_exit.store(false);
	std::thread(std::bind(&PcapReader::ThreadLoadProcess, this)).detach();
}

void PcapReader::ThreadLoadProcess()
{
	run_exit.store(false);

	std::ifstream inStream(m_pacpPath, std::ios_base::in | std::ios_base::binary);
	if (!inStream)
	{
		USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_OPEN_PCAP_FAILED, "Open pcap file failed!");
		run_exit.store(true);
		return;
	}
	
	while (run_read)
	{
		inStream.seekg(sizeof(Pcap_FileHeader), std::ios::beg);
		if (inStream.eof())
		{
			USE_EXCEPTION_ERROR(TWException::TWEC_ERROR_PCAP_FILE_INVALID, "The pcap file is invalid!");
			run_exit.store(true);
			return;
		}
		else
		{
			USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_OPEN_PCAP_SUCCESS, "Open pcap file successed!");
		}

		std::chrono::time_point<std::chrono::system_clock> beginPlayTime = std::chrono::system_clock::now();
		u_int last_tv_sec = 0;
		u_int last_tv_usec = 0;

		int packageCount = 0;
		while (run_read)
		{
			if (m_pause)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));

				//clear cash time
				last_tv_sec = 0;
				last_tv_usec = 0;
				
				continue;
			}

			packageCount++;

			//pcap pack
			Pcap_PktHdr pcap_pkt_hdr;
			std::streamsize readSize = inStream.read((char*)(&pcap_pkt_hdr), sizeof(Pcap_PktHdr)).gcount();
			if (readSize != sizeof(Pcap_PktHdr)) 
				break;

			if (0 == last_tv_sec && 0 == last_tv_usec)
			{
				last_tv_sec = pcap_pkt_hdr.tv_sec;
				last_tv_usec = pcap_pkt_hdr.tv_usec;
				beginPlayTime = std::chrono::system_clock::now();
			}
			else
			{
				long long diff_usecond = 0;
				if (pcap_pkt_hdr.tv_sec == last_tv_sec)
				{
					if (pcap_pkt_hdr.tv_usec <= last_tv_usec)
						diff_usecond = 0;
					else
						diff_usecond = (long long)(pcap_pkt_hdr.tv_usec) - last_tv_usec;
				}else if (pcap_pkt_hdr.tv_sec > last_tv_sec)
				{
					diff_usecond = (pcap_pkt_hdr.tv_sec - last_tv_sec - 1) * 1000000 +  (1000000- last_tv_usec) + pcap_pkt_hdr.tv_usec;
				}
				else
				{
					diff_usecond = 0;
				}

				last_tv_sec = pcap_pkt_hdr.tv_sec;
				last_tv_usec = pcap_pkt_hdr.tv_usec;

				beginPlayTime += std::chrono::microseconds(diff_usecond);

				std::this_thread::sleep_until(beginPlayTime);
			}
			
			//net pack
			NETHdr net_hdr;
			readSize = inStream.read((char*)(&net_hdr), sizeof(NETHdr)).gcount();
			if (readSize != sizeof(NETHdr))
				break;
			//check invalid
			if (0x0008 != net_hdr.net_type)
			{
				//ignore
				inStream.seekg(pcap_pkt_hdr.len - sizeof(NETHdr), std::ios::cur);
				continue;
			}

			//IP pcak
			IPHdr ip_hdr;
			readSize = inStream.read((char*)(&ip_hdr), sizeof(IPHdr)).gcount();
			if (readSize != sizeof(IPHdr)) 
				break;
			//check invalid
			if (m_lidarIP != ip_hdr.GetSourceIP() || ip_hdr.protocol != 17)
			{
				//ignore
				inStream.seekg(ip_hdr.GetLength() - sizeof(IPHdr), std::ios::cur);
				continue;
			}

			//UDP pack
			UDPHdr udp_hdr;
			readSize = inStream.read((char*)(&udp_hdr), sizeof(UDPHdr)).gcount();
			if (readSize != sizeof(UDPHdr)) 
				break;
			//check invalid
			if (m_localPort != udp_hdr.GetDestPort() && 
				10110 != udp_hdr.GetDestPort() &&
				m_localDIFPort != udp_hdr.GetDestPort())
			{
				//ignore
				inStream.seekg(udp_hdr.GetLength() - sizeof(UDPHdr), std::ios::cur);
				continue;
			}

			//data pack
			TWUDPPackage::Ptr udp_data(new TWUDPPackage);
			udp_data->m_length = udp_hdr.GetLength() - sizeof(UDPHdr);
			udp_data->t_sec = pcap_pkt_hdr.tv_sec;
			udp_data->t_usec = pcap_pkt_hdr.tv_usec;
			readSize = inStream.read(udp_data->m_szData, udp_data->m_length).gcount();
			if (readSize != udp_data->m_length)
			{
				break;
			}
			m_packageCache.PushBackPackage(udp_data);
		}
		
		if (run_read && m_repeat)
		{
			USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_REPEAT_PLAY, "Repeat to read!");
			inStream.clear();
			continue;
		}
		else
			break;
	}

	run_exit.store(true);

	USE_EXCEPTION_TIPS(TWException::TWEC_TIPS_PCAP_EXIT, "Exit reading the PCAP file!");
}

void PcapReader::RegExceptionCallback(const std::function<void(const TWException&)>& callback)
{
	m_funcException = callback;
}
