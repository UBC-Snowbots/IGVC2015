/**
\file RSLSubsystemRecord.h

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
#include "openjaus/core/Triggers/Fields/RSLSubsystemRecord.h"

namespace openjaus
{
namespace core
{

RSLSubsystemRecord::RSLSubsystemRecord():
	subsystemID(),
	rSLNodeList()
{

	fields.push_back(&subsystemID);
	subsystemID.setName("SubsystemID");
	subsystemID.setOptional(false);
	subsystemID.setInterpretation("Subsystem ID.");
	subsystemID.setValue(0);

	fields.push_back(&rSLNodeList);
	rSLNodeList.setName("RSLNodeList");
	rSLNodeList.setOptional(false);
	// Nothing to Init

}

RSLSubsystemRecord::RSLSubsystemRecord(const RSLSubsystemRecord &source)
{
	this->copy(const_cast<RSLSubsystemRecord&>(source));
}

RSLSubsystemRecord::~RSLSubsystemRecord()
{

}


uint16_t RSLSubsystemRecord::getSubsystemID(void)
{
	return this->subsystemID.getValue();
}

void RSLSubsystemRecord::setSubsystemID(uint16_t value)
{
	this->subsystemID.setValue(value);
}

ServicesNodeList& RSLSubsystemRecord::getRSLNodeList(void)
{
	return this->rSLNodeList;
}

int RSLSubsystemRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(subsystemID);
	byteSize += dst->pack(rSLNodeList);
	return byteSize;
}
int RSLSubsystemRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(subsystemID);
	byteSize += src->unpack(rSLNodeList);
	return byteSize;
}

int RSLSubsystemRecord::length(void)
{
	int length = 0;
	length += subsystemID.length(); // subsystemID
	length += rSLNodeList.length(); // rSLNodeList
	return length;
}

std::string RSLSubsystemRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RSLSubsystemRecord\">\n";
	oss << subsystemID.toXml(level+1); // subsystemID
	oss << rSLNodeList.toXml(level+1); // rSLNodeList
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void RSLSubsystemRecord::copy(RSLSubsystemRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->subsystemID.setName("SubsystemID");
	this->subsystemID.setOptional(false);
	this->subsystemID.setInterpretation("Subsystem ID.");
	this->subsystemID.setValue(source.getSubsystemID()); 
 
	this->rSLNodeList.copy(source.getRSLNodeList()); 
 
}

} // namespace core
} // namespace openjaus

