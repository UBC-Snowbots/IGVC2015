/**
\file QSLSubsystemRecord.h

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
#include "openjaus/core/Triggers/Fields/QSLSubsystemRecord.h"

namespace openjaus
{
namespace core
{

QSLSubsystemRecord::QSLSubsystemRecord():
	subsystemID(),
	qSLNodeList()
{

	fields.push_back(&subsystemID);
	subsystemID.setName("SubsystemID");
	subsystemID.setOptional(false);
	subsystemID.setInterpretation("Use 65535 (0xFFFF) if service information from  all subsystems in the system is required");
	subsystemID.setValue(0);

	fields.push_back(&qSLNodeList);
	qSLNodeList.setName("QSLNodeList");
	qSLNodeList.setOptional(false);
	// Nothing to Init

}

QSLSubsystemRecord::QSLSubsystemRecord(const QSLSubsystemRecord &source)
{
	this->copy(const_cast<QSLSubsystemRecord&>(source));
}

QSLSubsystemRecord::~QSLSubsystemRecord()
{

}


uint16_t QSLSubsystemRecord::getSubsystemID(void)
{
	return this->subsystemID.getValue();
}

void QSLSubsystemRecord::setSubsystemID(uint16_t value)
{
	this->subsystemID.setValue(value);
}

QSLNodeList& QSLSubsystemRecord::getQSLNodeList(void)
{
	return this->qSLNodeList;
}

int QSLSubsystemRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(subsystemID);
	byteSize += dst->pack(qSLNodeList);
	return byteSize;
}
int QSLSubsystemRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(subsystemID);
	byteSize += src->unpack(qSLNodeList);
	return byteSize;
}

int QSLSubsystemRecord::length(void)
{
	int length = 0;
	length += subsystemID.length(); // subsystemID
	length += qSLNodeList.length(); // qSLNodeList
	return length;
}

std::string QSLSubsystemRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"QSLSubsystemRecord\">\n";
	oss << subsystemID.toXml(level+1); // subsystemID
	oss << qSLNodeList.toXml(level+1); // qSLNodeList
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void QSLSubsystemRecord::copy(QSLSubsystemRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->subsystemID.setName("SubsystemID");
	this->subsystemID.setOptional(false);
	this->subsystemID.setInterpretation("Use 65535 (0xFFFF) if service information from  all subsystems in the system is required");
	this->subsystemID.setValue(source.getSubsystemID()); 
 
	this->qSLNodeList.copy(source.getQSLNodeList()); 
 
}

} // namespace core
} // namespace openjaus

