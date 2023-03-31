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
#include "PackageCache.h"

PackageCache::PackageCache()
{
}

PackageCache::~PackageCache()
{
}

void PackageCache::PushBackPackage(TWUDPPackage::Ptr udpPackage)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_queue.push(udpPackage);
}

TWUDPPackage::Ptr PackageCache::PopFrontPackage()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	TWUDPPackage::Ptr pack = m_queue.front();
	m_queue.pop();
	return pack;
}

int PackageCache::Size()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_queue.size();
}

void PackageCache::Clear()
{
	std::queue<TWUDPPackage::Ptr> empty;
	std::lock_guard<std::mutex> lock(m_mutex);
	swap(empty, m_queue);
}
