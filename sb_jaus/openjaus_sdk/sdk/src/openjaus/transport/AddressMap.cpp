/**
\file AddressMap.h

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

#include "openjaus/transport/AddressMap.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/transport/AS5669/JudpAddress.h"
#include "openjaus/transport/AS5669/TCPAddress.h"
// End of user code

namespace openjaus
{
namespace transport
{

// Start of user code for default constructor:
AddressMap::AddressMap()
{
}
// End of user code

// Start of user code for default destructor:
AddressMap::~AddressMap()
{
	// Delete map data
	std::map< int32_t, std::map< TransportType, TransportData * > >::const_iterator i;
	for(i = transportData.begin(); i != transportData.end(); i++)
	{
		std::map< TransportType, TransportData * > map = i->second;
		for(std::map< TransportType, TransportData * >::const_iterator j = map.begin(); j != map.end(); j++)
		{
			delete j->second;
		}
	}
}
// End of user code

const std::map< int32_t, std::map< TransportType, TransportData * > >& AddressMap::getTransportData() const
{
	// Start of user code for accessor getTransportData:
	
	return transportData;
	// End of user code
}



// Class Methods
AddressMap& AddressMap::instance()
{
	// Start of user code for method instance:
	static AddressMap inst;
	return inst;
	// End of user code
}


void AddressMap::setTransportData(const Address &address, TransportData &data)
{
	// Start of user code for method setTransportData:
	TransportData *localData = NULL;

	std::map< int32_t, std::map< TransportType, TransportData * > >::iterator i = instance().transportData.find(address.getHash());
	if(i != instance().transportData.end())
	{
		std::map< TransportType, TransportData * >::iterator j = i->second.find(data.getType());
		if(j != i->second.end())
		{
			localData = j->second;
		}
	}

	switch(data.getType())
	{
		case UDP:
			if(!localData)
			{
				localData = new AS5669::JudpAddress();
			}
			*static_cast<AS5669::JudpAddress *>(localData) = static_cast<AS5669::JudpAddress& >(data);
			break;

		case TCP:
			if(!localData)
			{
				localData = new AS5669::TCPAddress();
			}
			*static_cast<AS5669::TCPAddress *>(localData) = static_cast<AS5669::TCPAddress& >(data);
			break;

		default:
			THROW_EXCEPTION("AddressMap: Unsupported data type");
			return;
	}

	instance().transportData[address.getHash()][data.getType()] = localData;
	// End of user code
}


bool AddressMap::getTransportData(const Address &address, TransportData &data)
{
	// Start of user code for method getTransportData:
	std::map< int32_t, std::map< TransportType, TransportData * > >::iterator i = instance().transportData.find(address.getHash());
	if(i == instance().transportData.end())
	{
		return false;
	}

	std::map< TransportType, TransportData * >::iterator j = i->second.find(data.getType());
	if(j == i->second.end())
	{
		return false;
	}

	switch(data.getType())
	{
		case UDP:
			static_cast<AS5669::JudpAddress& >(data) = *static_cast<AS5669::JudpAddress *>(j->second);
			break;

		case TCP:
			static_cast<AS5669::TCPAddress& >(data) = *static_cast<AS5669::TCPAddress *>(j->second);
			break;

		default:
			THROW_EXCEPTION("AddressMap: Unsupported data type");
	}

	return true;
	// End of user code
}




std::string AddressMap::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "Address Map:" << std::endl;

	std::map< int32_t, std::map< TransportType, TransportData * > >::const_iterator i;
	for(i = transportData.begin(); i != transportData.end(); i++)
	{
		Address address(i->first >> 16, i->first >> 8 && 0xFF, i->first && 0xFF);
		std::map< TransportType, TransportData * > map = i->second;
		for(std::map< TransportType, TransportData * >::const_iterator j = map.begin(); j != map.end(); j++)
		{
			switch(j->first)
			{
				case UDP:
					oss << address << " @ "	<< dynamic_cast<AS5669::JudpAddress*>(j->second)->toString() << std::endl;
					break;

				case TCP:
					oss << address << " @ "	<< dynamic_cast<AS5669::TCPAddress*>(j->second)->toString() << std::endl;
					break;

				default:
					THROW_EXCEPTION("AddressMap: Unsupported data type");
					break;
			}
		}
	}

	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const AddressMap& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace transport
} // namespace openjaus

