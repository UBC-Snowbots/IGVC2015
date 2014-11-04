/**
\file QSLComponentRecord.h

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
#include "openjaus/core/Triggers/Fields/QSLComponentRecord.h"

namespace openjaus
{
namespace core
{

QSLComponentRecord::QSLComponentRecord():
	componentID(),
	searchFilter()
{
	this->presenceVector = 0;

	fields.push_back(&componentID);
	componentID.setName("ComponentID");
	componentID.setOptional(false);
	componentID.setInterpretation("Use 255 if service information from  components in the node are required");
	componentID.setValue(0);

	fields.push_back(&searchFilter);
	searchFilter.setName("SearchFilter");
	searchFilter.setOptional(true);
	searchFilter.setInterpretation("An optional filter to apply to the search results.  Only service identifiers that contain this string should be returned.");
	searchFilter.setSizeType(model::fields::UNSIGNED_BYTE);

}

QSLComponentRecord::QSLComponentRecord(const QSLComponentRecord &source)
{
	this->copy(const_cast<QSLComponentRecord&>(source));
}

QSLComponentRecord::~QSLComponentRecord()
{

}


uint8_t QSLComponentRecord::getComponentID(void)
{
	return this->componentID.getValue();
}

void QSLComponentRecord::setComponentID(uint8_t value)
{
	this->componentID.setValue(value);
}

std::string QSLComponentRecord::getSearchFilter(void)
{
	return this->searchFilter.getValue();
}

void QSLComponentRecord::setSearchFilter(std::string value)
{
	this->searchFilter.setValue(value);
}

int QSLComponentRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(componentID);
	if(this->isSearchFilterEnabled())
	{
		byteSize += dst->pack(searchFilter);
	}
	return byteSize;
}
int QSLComponentRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(componentID);
	if(this->isSearchFilterEnabled())
	{
		byteSize += src->unpack(searchFilter);
	}
	return byteSize;
}

int QSLComponentRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += componentID.length(); // componentID
	if(this->isSearchFilterEnabled())
	{
		length += searchFilter.length(); // searchFilter
	}
	return length;
}

std::string QSLComponentRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"QSLComponentRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSearchFilterEnabled value=\"" << std::boolalpha << this->isSearchFilterEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << componentID.toXml(level+1); // componentID
	if(this->isSearchFilterEnabled())
	{
		oss << searchFilter.toXml(level+1); // searchFilter
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void QSLComponentRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t QSLComponentRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool QSLComponentRecord::isSearchFilterEnabled(void) const
{
	return (this->presenceVector & (0x01 << QSLComponentRecord::SEARCHFILTER));
}

void QSLComponentRecord::enableSearchFilter(void)
{
	this->presenceVector |= 0x01 << QSLComponentRecord::SEARCHFILTER;
}

void QSLComponentRecord::disableSearchFilter(void)
{
	this->presenceVector &= ~(0x01 << QSLComponentRecord::SEARCHFILTER);
}


void QSLComponentRecord::copy(QSLComponentRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->componentID.setName("ComponentID");
	this->componentID.setOptional(false);
	this->componentID.setInterpretation("Use 255 if service information from  components in the node are required");
	this->componentID.setValue(source.getComponentID()); 
 
	this->searchFilter.setName("SearchFilter");
	this->searchFilter.setOptional(true);
	this->searchFilter.setInterpretation("An optional filter to apply to the search results.  Only service identifiers that contain this string should be returned.");
	this->searchFilter.setValue(source.getSearchFilter());
	this->searchFilter.setSizeType(model::fields::UNSIGNED_BYTE); 
 
}

} // namespace core
} // namespace openjaus

