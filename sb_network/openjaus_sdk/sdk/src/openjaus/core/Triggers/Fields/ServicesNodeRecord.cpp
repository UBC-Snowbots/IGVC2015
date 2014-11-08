/**
\file ServicesNodeRecord.h

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


#include <openjaus.h>
#include "openjaus/core/Triggers/Fields/ServicesNodeRecord.h"

namespace openjaus
{
namespace core
{

ServicesNodeRecord::ServicesNodeRecord():
	nodeID(),
	servicesComponentList()
{

	fields.push_back(&nodeID);
	nodeID.setName("NodeID");
	nodeID.setOptional(false);
	nodeID.setInterpretation("Node ID.  For single Node or Component reporting this field shall contain the Node ID as specified in the Destination Address of the Query Configuration message");
	nodeID.setValue(0);

	fields.push_back(&servicesComponentList);
	servicesComponentList.setName("ServicesComponentList");
	servicesComponentList.setOptional(false);
	// Nothing to Init

}

ServicesNodeRecord::ServicesNodeRecord(const ServicesNodeRecord &source)
{
	this->copy(const_cast<ServicesNodeRecord&>(source));
}

ServicesNodeRecord::~ServicesNodeRecord()
{

}


uint8_t ServicesNodeRecord::getNodeID(void)
{
	return this->nodeID.getValue();
}

void ServicesNodeRecord::setNodeID(uint8_t value)
{
	this->nodeID.setValue(value);
}

ServicesComponentList& ServicesNodeRecord::getServicesComponentList(void)
{
	return this->servicesComponentList;
}

int ServicesNodeRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(nodeID);
	byteSize += dst->pack(servicesComponentList);
	return byteSize;
}
int ServicesNodeRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(nodeID);
	byteSize += src->unpack(servicesComponentList);
	return byteSize;
}

int ServicesNodeRecord::length(void)
{
	int length = 0;
	length += nodeID.length(); // nodeID
	length += servicesComponentList.length(); // servicesComponentList
	return length;
}

std::string ServicesNodeRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ServicesNodeRecord\">\n";
	oss << nodeID.toXml(level+1); // nodeID
	oss << servicesComponentList.toXml(level+1); // servicesComponentList
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ServicesNodeRecord::copy(ServicesNodeRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->nodeID.setName("NodeID");
	this->nodeID.setOptional(false);
	this->nodeID.setInterpretation("Node ID.  For single Node or Component reporting this field shall contain the Node ID as specified in the Destination Address of the Query Configuration message");
	this->nodeID.setValue(source.getNodeID()); 
 
	this->servicesComponentList.copy(source.getServicesComponentList()); 
 
}

} // namespace core
} // namespace openjaus

