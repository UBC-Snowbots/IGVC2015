/**
\file Socket.h

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of the OpenJAUS Software Development Kit (SDK). This 
software is distributed under one of two licenses, the OpenJAUS SDK 
Commercial End User License Agreement or the OpenJAUS SDK Non-Commercial 
End User License Agreement. The appropriate licensing details were included 
in with your developer credentials and software download. See the LICENSE 
file included with this software for full details.
 
THIS SOFTWARE IS PROVIDED BY THE LICENSOR (OPENJAUS LCC) "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LICENSOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. THE LICENSOR DOES NOT WARRANT THAT THE LICENSED SOFTWARE WILL MEET
LICENSEE'S REQUIREMENTS OR THAT THE OPERATION OF THE LICENSED SOFTWARE
WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ERRORS IN THE LICENSED
SOFTWARE WILL BE CORRECTED.

\ Software History
- [2011-08-23] - Added AS6057: Manipulators
- [2011-08-01] - Added AS6060: Environment Sensing
- [2011-06-16] - First Release 

*/

#include "openjaus/system/Socket.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
#include "openjaus/system/Logger.h"
#include <stdio.h>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Socket::Socket() :
	blocking(true),
	port(0),
	descriptor(-1),
	ipAddress(),
	timeout()
{
	// Make sure the system is ready for socket communications
	initialize();
	descriptorCount++;
}
// End of user code

// Start of user code for default destructor:
Socket::~Socket()
{
	CLOSE_SOCKET(descriptor);

	// Free any allocated interface information
	for(std::vector<NetworkInterface *>::iterator i = interfaces.begin();
		i != interfaces.end();
		++i)
	{
		delete (*i);
	}
	interfaces.clear();

	descriptorCount--;
	deinitialize();
}
// End of user code

bool Socket::isBlocking() const
{
	// Start of user code for accessor getBlocking:
	
	return blocking;
	// End of user code
}


uint16_t Socket::getPort() const
{
	// Start of user code for accessor getPort:
	
	return port;
	// End of user code
}


int Socket::getDescriptor() const
{
	// Start of user code for accessor getDescriptor:
	
	return descriptor;
	// End of user code
}


const InetAddress& Socket::getIpAddress() const
{
	// Start of user code for accessor getIpAddress:
	
	return ipAddress;
	// End of user code
}


const Time& Socket::getTimeout() const
{
	// Start of user code for accessor getTimeout:
	
	return timeout;
	// End of user code
}

bool Socket::setTimeout(const Time& timeout)
{
	// Start of user code for accessor setTimeout:
	if(timeout.getMicroseconds() >= 0 && timeout.getSeconds() >= 0)
	{
		this->timeout = timeout;
		blocking = false;
	}
	else
	{
		this->timeout.setSeconds(0);
		this->timeout.setMicroseconds(0);
		blocking = true;
	}

	return !blocking;
	// End of user code
}


const std::vector< NetworkInterface* >& Socket::getInterfaces() const
{
	// Start of user code for accessor getInterfaces:
	return interfaces;
	// End of user code
}



// Class Methods
void Socket::readInterfaces()
{
	// Start of user code for method readInterfaces:
	for(std::vector<NetworkInterface *>::iterator i = interfaces.begin();
		i != interfaces.end();
		++i)
	{
		delete (*i);
	}
	interfaces.clear();

	// Query available interfaces
#ifdef WIN32
	//INTERFACE_INFO InterfaceList[20];
	//unsigned long nBytesReturned;
	//if(WSAIoctl(descriptor, SIO_GET_INTERFACE_LIST, 0, 0, &InterfaceList, sizeof(InterfaceList), &nBytesReturned, 0, 0) == SOCKET_ERROR)
	//{
	//	THROW_EXCEPTION("Could not enumerate network interfaces: " << lastSocketError());
	//}

	//int nNumInterfaces = nBytesReturned / sizeof(INTERFACE_INFO);
	//LOG_DEBUG("Socket: Found " << nNumInterfaces << " network interfaces");
	//for (int i = 0; i < nNumInterfaces; ++i)
	//{
	//	NetworkInterface *netIf = new NetworkInterface();

	//	InetAddress netIfAddress;
	//	sockaddr_in *pAddress = (sockaddr_in *) &InterfaceList[i].iiAddress;
	//	netIfAddress.setValue(pAddress->sin_addr.s_addr);
	//	netIf->setAddress(netIfAddress);
	//	netIf->setName(netIfAddress.toString());

	//	sockaddr_in *netmaskAddress = (sockaddr_in *)&InterfaceList[i].iiNetmask;
	//	netIfAddress.setValue(netmaskAddress->sin_addr.s_addr);
	//	netIf->setNetmask(netIfAddress);

	//	// Broadcast address is calculated because Windows was not returning a valid one
	//	netIfAddress.setValue(pAddress->sin_addr.s_addr | ~(netmaskAddress->sin_addr.s_addr));
	//	netIf->setBroadcast(netIfAddress);

	//	interfaces.push_back(netIf);
	//	LOG_DEBUG("Socket: Found " << *netIf << ", bcast: " << netIf->getBroadcast().toString());
	//}

	IP_ADAPTER_ADDRESSES temp;
	PIP_ADAPTER_ADDRESSES adapterInfo = &temp;
    ULONG outputBufferLength = sizeof(IP_ADAPTER_ADDRESSES);
	ULONG errorNum = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, 0, adapterInfo, &outputBufferLength);
	adapterInfo = (PIP_ADAPTER_ADDRESSES) malloc(outputBufferLength);

	errorNum = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, 0, adapterInfo, &outputBufferLength);
    if(errorNum != NO_ERROR)
	{
		THROW_EXCEPTION("Could not enumerate network interfaces: " << errorString(errorNum));
	}

	PIP_ADAPTER_ADDRESSES adapter = adapterInfo;
	while(adapter)
	{
		if(	adapter->IfType != MIB_IF_TYPE_ETHERNET &&
			adapter->IfType != MIB_IF_TYPE_LOOPBACK &&
			adapter->IfType != IF_TYPE_IEEE80211)
		{	// Ignore all interfaces that are not ethernet
			adapter = adapter->Next;
			continue;
		}

		NetworkInterface *netIf = new NetworkInterface();

		size_t origsize = wcslen(adapter->FriendlyName) + 1;
		size_t convertedChars = 0;
		char newString[128];
		wcstombs_s(&convertedChars, newString, origsize, adapter->FriendlyName, _TRUNCATE);
		netIf->setName(newString);
		if(adapter->IfType == MIB_IF_TYPE_LOOPBACK)
		{	// Override with default name for loopback
			netIf->setName("lo");
		}

		InetAddress netifAddress;
		if(adapter->FirstUnicastAddress)
		{
			struct sockaddr_in *socketAddress = (struct sockaddr_in *)adapter->FirstUnicastAddress->Address.lpSockaddr;
			netifAddress.setValue(socketAddress->sin_addr.s_addr);
			netIf->setAddress(netifAddress);
		}
		else
		{
			LOG("Could not get IP Address of adapter: " << newString << ", check your network settings.");
			delete netIf;
			adapter = adapter->Next;
			continue;
		}

		int prefixLength_bits = 8;	// Assume prefix length is 8 bits (as is case with loopback)
									// XP returns no prefix for loopback. Win 7 does.
		if(adapter->FirstPrefix)
		{	// If a prefix was given then use the first length
			prefixLength_bits = adapter->FirstPrefix->PrefixLength;
		}

		uint32_t netmask = 0xFFFFFFFF >> (32 - prefixLength_bits);
		netifAddress.setValue(netmask);
		netIf->setNetmask(netifAddress);

		netifAddress.setValue(netIf->getAddress().getValue() | ~netifAddress.getValue());
		netIf->setBroadcast(netifAddress);

		LOG_DEBUG("Socket: Found " << *netIf << ", netmask: " << netIf->getNetmask() << ", bcast: " << netIf->getBroadcast());
		interfaces.push_back(netIf);
		adapter = adapter->Next;
	}

	free(adapterInfo);
#else
	struct ifreq ifreqs[20];
	struct ifconf ifc;
	memset(&ifc, 0, sizeof(struct ifconf));
	ifc.ifc_buf = reinterpret_cast<char*>(ifreqs);
	ifc.ifc_len = sizeof(ifreqs);

	if(ioctl(descriptor, SIOCGIFCONF, &ifc) < 0)
	{
		THROW_EXCEPTION("Could not enumerate network interfaces");
	}

	// Iterate through the list of interfaces
	int interfaceCount = ifc.ifc_len / sizeof(struct ifreq);
	for(int i = 0; i < interfaceCount; ++i)
	{
		// Show the device name and IP address
		NetworkInterface *netIf = new NetworkInterface();
		netIf->setName(ifreqs[i].ifr_name);

		InetAddress ipAddress;
		ipAddress.setValue(reinterpret_cast<struct sockaddr_in *>(&ifreqs[i].ifr_addr)->sin_addr.s_addr);
		netIf->setAddress(ipAddress);

		// Get the MAC address
		//		if(ioctl(descriptor, SIOCGIFHWADDR, item) < 0)
		//		{
		//			// perror("ioctl(SIOCGIFHWADDR)");
		//			// return 1;
		//		}

		// Get the broadcast address
		if(ioctl(descriptor, SIOCGIFBRDADDR, &ifreqs[i]) < 0)
		{
			THROW_EXCEPTION("Could not read broadcast address for interface: " << netIf->getName());
		}
		ipAddress.setValue(reinterpret_cast<struct sockaddr_in *>(&ifreqs[i].ifr_broadaddr)->sin_addr.s_addr);
		netIf->setBroadcast(ipAddress);

		interfaces.push_back(netIf);

		LOG_DEBUG("Found Net IF: " << netIf->getName() << " with IP: " << netIf->getAddress());
	}
#endif
	// End of user code
}


bool Socket::isLocalAddress(const InetAddress &ipAddress)
{
	// Start of user code for method isLocalAddress:
	for(unsigned int i = 0; i < interfaces.size(); ++i)
	{
		if(interfaces[i]->getAddress().getValue() == ipAddress.getValue())
		{
			return true;
		}
	}

	return false;
	// End of user code
}


uint64_t Socket::addressHash()
{
	// Start of user code for method addressHash:
	return (ipAddress.hash() << 16) + port;
	// End of user code
}




std::string Socket::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Socket& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods

int Socket::descriptorCount = 0;

void Socket::initialize()
{
#ifdef WIN32
	if(descriptorCount == 0)
	{	// Initialize windows sockets to version 2.2
		WSADATA wsaData;
		if(WSAStartup(MAKEWORD(2, 2), &wsaData))
		{
			THROW_EXCEPTION("Socket: Could not startup windows sockets: Error = " << lastSocketError() );
		}
	}
#endif
}

void Socket::deinitialize()
{
#ifdef WIN32
	if(descriptorCount == 0)
	{
		// Cleanup windows sockets
		WSACleanup();
	}
#endif

}
// End of user code

} // namespace system
} // namespace openjaus

