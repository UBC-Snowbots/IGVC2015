/**
\file RangeSensorConfigurationListRefArray.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorConfigurationListRefArray.h"

namespace openjaus
{
namespace environment
{

RangeSensorConfigurationListRefArray::RangeSensorConfigurationListRefArray() : rangeSensorConfigurationRec()
{

}

RangeSensorConfigurationListRefArray::~RangeSensorConfigurationListRefArray()
{

}

void RangeSensorConfigurationListRefArray::copy(RangeSensorConfigurationListRefArray& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->rangeSensorConfigurationRec = source.getRangeSensorConfigurationRec();
}

int RangeSensorConfigurationListRefArray::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint16_t>(this->rangeSensorConfigurationRec.size())); // Count
	
	for(size_t i = 0; i < this->rangeSensorConfigurationRec.size(); i++) // Elements
	{
		result += dst->pack(this->rangeSensorConfigurationRec[i]); 
	}
	return result;
}

int RangeSensorConfigurationListRefArray::from(system::Buffer *src)
{
	int result = 0;
	uint16_t count;

	result += src->unpack(count); // Count
	this->rangeSensorConfigurationRec.empty();
	this->rangeSensorConfigurationRec.reserve(count);
	
	for(size_t i = 0; i < count; i++) // Elements
	{
		RangeSensorConfigurationRecord temp;
		result += src->unpack(temp);
		this->rangeSensorConfigurationRec.push_back(temp);
	}
	return result;
}

int RangeSensorConfigurationListRefArray::length(void)
{
	int result = 0;
	result += sizeof(uint16_t); // Count
	
	for(size_t i = 0; i < this->rangeSensorConfigurationRec.size(); i++) // Elements
	{
		result += this->rangeSensorConfigurationRec[i].length(); 
	}
	return result;
}

std::string RangeSensorConfigurationListRefArray::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<List name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<count>" << this->rangeSensorConfigurationRec.size() << "</count>\n";
	
	for(size_t i = 0; i < this->rangeSensorConfigurationRec.size(); i++) // Elements
	{
		oss << this->rangeSensorConfigurationRec.at(i).toXml(level + 1);
	}
	oss << prefix.str() << "</List>\n";
	return oss.str();
}

std::vector<RangeSensorConfigurationRecord>& RangeSensorConfigurationListRefArray::getRangeSensorConfigurationRec()
{
	return this->rangeSensorConfigurationRec;
}

void RangeSensorConfigurationListRefArray::add(RangeSensorConfigurationRecord value)
{
	this->rangeSensorConfigurationRec.push_back(value);
}

RangeSensorConfigurationRecord& RangeSensorConfigurationListRefArray::get(size_t index)
{
	return this->rangeSensorConfigurationRec.at(index);
}

size_t RangeSensorConfigurationListRefArray::size()
{
	return this->rangeSensorConfigurationRec.size();
}

bool RangeSensorConfigurationListRefArray::empty()
{
	return this->rangeSensorConfigurationRec.empty();
}

void RangeSensorConfigurationListRefArray::clear()
{
	this->rangeSensorConfigurationRec.clear();
}

void RangeSensorConfigurationListRefArray::remove(int index)
{
	this->rangeSensorConfigurationRec.erase(this->rangeSensorConfigurationRec.begin() + index);
}


} // namespace environment
} // namespace openjaus



