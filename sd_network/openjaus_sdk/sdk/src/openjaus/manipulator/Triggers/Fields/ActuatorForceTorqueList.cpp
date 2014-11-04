/**
\file ActuatorForceTorqueList.h

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
#include "openjaus/manipulator/Triggers/Fields/ActuatorForceTorqueList.h"

namespace openjaus
{
namespace manipulator
{

ActuatorForceTorqueList::ActuatorForceTorqueList() : actuatorForceTorqueRec()
{

}

ActuatorForceTorqueList::~ActuatorForceTorqueList()
{

}

void ActuatorForceTorqueList::copy(ActuatorForceTorqueList& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->actuatorForceTorqueRec = source.getActuatorForceTorqueRec();
}

int ActuatorForceTorqueList::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->actuatorForceTorqueRec.size())); // Count
	
	for(size_t i = 0; i < this->actuatorForceTorqueRec.size(); i++) // Elements
	{
		result += dst->pack(this->actuatorForceTorqueRec[i]); 
	}
	return result;
}

int ActuatorForceTorqueList::from(system::Buffer *src)
{
	int result = 0;
	uint8_t count;

	result += src->unpack(count); // Count
	this->actuatorForceTorqueRec.empty();
	this->actuatorForceTorqueRec.reserve(count);
	
	for(size_t i = 0; i < count; i++) // Elements
	{
		ActuatorForceTorqueRecord temp;
		result += src->unpack(temp);
		this->actuatorForceTorqueRec.push_back(temp);
	}
	return result;
}

int ActuatorForceTorqueList::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Count
	
	for(size_t i = 0; i < this->actuatorForceTorqueRec.size(); i++) // Elements
	{
		result += this->actuatorForceTorqueRec[i].length(); 
	}
	return result;
}

std::string ActuatorForceTorqueList::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<List name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<count>" << this->actuatorForceTorqueRec.size() << "</count>\n";
	
	for(size_t i = 0; i < this->actuatorForceTorqueRec.size(); i++) // Elements
	{
		oss << this->actuatorForceTorqueRec.at(i).toXml(level + 1);
	}
	oss << prefix.str() << "</List>\n";
	return oss.str();
}

std::vector<ActuatorForceTorqueRecord>& ActuatorForceTorqueList::getActuatorForceTorqueRec()
{
	return this->actuatorForceTorqueRec;
}

void ActuatorForceTorqueList::add(ActuatorForceTorqueRecord value)
{
	this->actuatorForceTorqueRec.push_back(value);
}

ActuatorForceTorqueRecord& ActuatorForceTorqueList::get(size_t index)
{
	return this->actuatorForceTorqueRec.at(index);
}

size_t ActuatorForceTorqueList::size()
{
	return this->actuatorForceTorqueRec.size();
}

bool ActuatorForceTorqueList::empty()
{
	return this->actuatorForceTorqueRec.empty();
}

void ActuatorForceTorqueList::clear()
{
	this->actuatorForceTorqueRec.clear();
}

void ActuatorForceTorqueList::remove(int index)
{
	this->actuatorForceTorqueRec.erase(this->actuatorForceTorqueRec.begin() + index);
}


} // namespace manipulator
} // namespace openjaus



