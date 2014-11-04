/**
\file DigitalVideoSensorConfigurationListRefArray.h

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
#include "openjaus/environment/Triggers/Fields/DigitalVideoSensorConfigurationListRefArray.h"

namespace openjaus
{
namespace environment
{

DigitalVideoSensorConfigurationListRefArray::DigitalVideoSensorConfigurationListRefArray() : digitalVideoSensorConfigurationRec()
{

}

DigitalVideoSensorConfigurationListRefArray::~DigitalVideoSensorConfigurationListRefArray()
{

}

void DigitalVideoSensorConfigurationListRefArray::copy(DigitalVideoSensorConfigurationListRefArray& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->digitalVideoSensorConfigurationRec = source.getDigitalVideoSensorConfigurationRec();
}

int DigitalVideoSensorConfigurationListRefArray::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint16_t>(this->digitalVideoSensorConfigurationRec.size())); // Count
	
	for(size_t i = 0; i < this->digitalVideoSensorConfigurationRec.size(); i++) // Elements
	{
		result += dst->pack(this->digitalVideoSensorConfigurationRec[i]); 
	}
	return result;
}

int DigitalVideoSensorConfigurationListRefArray::from(system::Buffer *src)
{
	int result = 0;
	uint16_t count;

	result += src->unpack(count); // Count
	this->digitalVideoSensorConfigurationRec.empty();
	this->digitalVideoSensorConfigurationRec.reserve(count);
	
	for(size_t i = 0; i < count; i++) // Elements
	{
		DigitalVideoSensorConfigurationRecord temp;
		result += src->unpack(temp);
		this->digitalVideoSensorConfigurationRec.push_back(temp);
	}
	return result;
}

int DigitalVideoSensorConfigurationListRefArray::length(void)
{
	int result = 0;
	result += sizeof(uint16_t); // Count
	
	for(size_t i = 0; i < this->digitalVideoSensorConfigurationRec.size(); i++) // Elements
	{
		result += this->digitalVideoSensorConfigurationRec[i].length(); 
	}
	return result;
}

std::string DigitalVideoSensorConfigurationListRefArray::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<List name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<count>" << this->digitalVideoSensorConfigurationRec.size() << "</count>\n";
	
	for(size_t i = 0; i < this->digitalVideoSensorConfigurationRec.size(); i++) // Elements
	{
		oss << this->digitalVideoSensorConfigurationRec.at(i).toXml(level + 1);
	}
	oss << prefix.str() << "</List>\n";
	return oss.str();
}

std::vector<DigitalVideoSensorConfigurationRecord>& DigitalVideoSensorConfigurationListRefArray::getDigitalVideoSensorConfigurationRec()
{
	return this->digitalVideoSensorConfigurationRec;
}

void DigitalVideoSensorConfigurationListRefArray::add(DigitalVideoSensorConfigurationRecord value)
{
	this->digitalVideoSensorConfigurationRec.push_back(value);
}

DigitalVideoSensorConfigurationRecord& DigitalVideoSensorConfigurationListRefArray::get(size_t index)
{
	return this->digitalVideoSensorConfigurationRec.at(index);
}

size_t DigitalVideoSensorConfigurationListRefArray::size()
{
	return this->digitalVideoSensorConfigurationRec.size();
}

bool DigitalVideoSensorConfigurationListRefArray::empty()
{
	return this->digitalVideoSensorConfigurationRec.empty();
}

void DigitalVideoSensorConfigurationListRefArray::clear()
{
	this->digitalVideoSensorConfigurationRec.clear();
}

void DigitalVideoSensorConfigurationListRefArray::remove(int index)
{
	this->digitalVideoSensorConfigurationRec.erase(this->digitalVideoSensorConfigurationRec.begin() + index);
}


} // namespace environment
} // namespace openjaus



