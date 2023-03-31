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

#pragma pack(push, 1)
//pcap head 24byte
struct Pcap_FileHeader
{
	Pcap_FileHeader() :magic(0xa1b2c3d4), version_major(0x0002), version_minor(0x0004),
		thiszone(0), sigfigs(0), snaplen(65536), linktype(1) {}
	u_int magic;	//0xa1b2c3d4
	u_short version_major;	//0x0002 
	u_short version_minor;	//0x0004
	int thiszone;	/* gmt to local correction */ //0
	u_int sigfigs;	/* accuracy of timestamps */	//0
	u_int snaplen;	/* max length saved portion of each pkt */	//65535 0x000000ff
	u_int linktype;	/* data link type (LINKTYPE_*) */	//0x00000001
};
//pcap pack 16字节
struct Pcap_PktHdr
{
	Pcap_PktHdr() :tv_sec(0), tv_usec(0), caplen(0), len(0) {}
	u_int tv_sec;         /* seconds */
	u_int tv_usec;        /* and microseconds */ 
	u_int caplen;	/* length of portion present data.size()+14+20+8 */
	u_int len;	/* length this packet (off wire) data.size()+14+20+8 */

	bool IsValid()
	{
		if (caplen > 0 && caplen < 1800 && caplen == len && !(tv_sec == tv_usec && tv_sec == caplen))
		{
			return true;
		}
		else
			return false;
	}
};

//net pack 14byte 
struct NETHdr
{
	NETHdr() :net_type(0x0008) {}
	void SetSourceMac(unsigned char* sourceMac)
	{
		memcpy(source_mac, sourceMac, 6);
	}
	unsigned char dest_mac[6] = { 0xff,0xff,0xff,0xff,0xff,0xff };
	unsigned char source_mac[6] = { 0x1a,0x2b,0x3c,0x4d,0x5e,0x6f };
	u_short net_type; //0x0008   //IPv4
};

//IP pack  20byte,Big endian order
struct IPHdr {
	IPHdr() :version_and_header_len(0x45), services_codepoint(0x00), totalLength(0), identification(0x0000), flags(0x0040),
		time_live(0xff), protocol(17), header_checksum(0xffff) {}

	void SetLength(u_short length)
	{
		totalLength = 0x0000 | length << 8;
		totalLength = totalLength | length >> 8;
	};
	u_short GetLength()
	{
		u_short r_length = 0;
		r_length = r_length | totalLength << 8;
		r_length = r_length | totalLength >> 8;
		return r_length;
	}
	std::string GetSourceIP()
	{
		char str_ip[16];
		sprintfT(str_ip, "%u.%u.%u.%u", source_ip[0], source_ip[1], source_ip[2], source_ip[3]);
		return std::string(str_ip);
	}
	//private:
	unsigned char version_and_header_len;  //0x45
	unsigned char services_codepoint; //0x00
	u_short totalLength; //data.size()+20+8
	u_short identification;
	u_short flags; //0x0040 
	unsigned char time_live; //0xff
	unsigned char protocol; //17 UDP
	u_short header_checksum; 
	unsigned char source_ip[4] = { 0,0,0,0 };
	unsigned char dest_ip[4] = { 0,0,0,0 };	

};

//UDP pack
struct UDPHdr
{
	UDPHdr() :sourcePort(0), destPort(0), length(0), check(0) {}
	void SetSourcePort(u_short port)
	{
		sourcePort = 0x0000 | port << 8;
		sourcePort = sourcePort | port >> 8;
	};
	void SetDestPort(u_short port)
	{
		destPort = 0x0000 | port << 8;
		destPort = destPort | port >> 8;
	};
	void SetLength(u_short len)
	{
		length = 0x0000 | len << 8;
		length = length | len >> 8;
	};
	u_short GetDestPort()
	{
		u_short port = 0;
		port = port | destPort << 8;
		port = port | destPort >> 8;
		return port;
	}
	u_short GetLength()
	{
		u_short r_length = 0;
		r_length = r_length | length << 8;
		r_length = r_length | length >> 8;
		return r_length;
	}
	//private:
	u_short	sourcePort;
	u_short	destPort;
	u_short	length;	//data.size()+8
	u_short	check;
};
#pragma pack(pop)

class PcapReader
{
public:
	PcapReader(std::string filePath, std::string lidarIP, int localPort, int localDIFPort, PackageCache& packageCache, bool repeat, std::mutex* mutex);
	~PcapReader();
	void RereadPcap(std::string pcapPath);
	void PausePcap(bool pause);

	void Start();
	void Stop();

	void ThreadLoadProcess();
	void RegExceptionCallback(const std::function<void(const TWException&)>& callback);

private:
	std::string m_pacpPath;
	std::string m_lidarIP;
	int m_localPort;
	int m_localDIFPort;
	bool m_repeat;
	std::atomic<bool>  m_pause;
	std::atomic<bool>  run_read;
	std::atomic<bool>  run_exit;
	std::mutex* m_mutex;

	PackageCache& m_packageCache;

	std::function<void(const TWException&)> m_funcException=NULL;
};

