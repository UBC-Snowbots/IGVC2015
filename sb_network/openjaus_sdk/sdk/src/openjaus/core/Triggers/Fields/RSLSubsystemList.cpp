/**
\file RSLSubsystemList.h

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
#include "openjaus/core/Triggers/Fields/RSLSubsystemList.h"

namespace openjaus
{
namespace core
{

RSLSubsystemList::RSLSubsystemList() : rSLSubsystemRec()
{

}

RSLSubsystemList::~RSLSubsystemList()
{

}

void RSLSubsystemList::copy(RSLSubsystemList& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->rSLSubsystemRec = source.getRSLSubsystemRec();
}

int RSLSubsystemList::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->rSLSubsystemRec.size())); // Count
	
	for(size_t i = 0; i < this->rSLSubsystemRec.size(); i++) // Elements
	{
		result += dst->pack(this->rSLSubsystemRec[i]); 
	}
	return result;
}

int RSLSubsystemList::from(system::Buffer *src)
{
	int result = 0;
	uint8_t count;

	result += src->unpack(count); // Count
	this->rSLSubsystemRec.empty();
	this->rSLSubsystemRec.reserve(count);
	
	for(size_t i = 0; i < count; i++) // Elements
	{
		RSLSubsystemRecord temp;
		result += src->unpack(temp);
		this->rSLSubsystemRec.push_back(temp);
	}
	return result;
}

int RSLSubsystemList::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Count
	
	for(size_t i = 0; i < this->rSLSubsystemRec.size(); i++) // Elements
	{
		result += this->rSLSubsystemRec[i].length(); 
	}
	return result;
}

std::string RSLSubsystemList::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<List name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<count>" << this->rSLSubsystemRec.size() << "</count>\n";
	
	for(size_t i = 0; i < this->rSLSubsystemRec.size(); i++) // Elements
	{
		oss << this->rSLSubsystemRec.at(i).toXml(level + 1);
	}
	oss << prefix.str() << "</List>\n";
	return oss.str();
}

std::vector<RSLSubsystemRecord>& RSLSubsystemList::getRSLSubsystemRec()
{
	return this->rSLSubsystemRec;
}

void RSLSubsystemList::add(RSLSubsystemRecord value)
{
	this->rSLSubsystemRec.push_back(value);
}

RSLSubsystemRecord& RSLSubsystemList::get(size_t index)
{
	return this->rSLSubsystemRec.at(index);
}

size_t RSLSubsystemList::size()
{
	return this->rSLSubsystemRec.size();
}

bool RSLSubsystemList::empty()
{
	return this->rSLSubsystemRec.empty();
}

void RSLSubsystemList::clear()
{
	this->rSLSubsystemRec.clear();
}

void RSLSubsystemList::remove(int index)
{
	this->rSLSubsystemRec.erase(this->rSLSubsystemRec.begin() + index);
}


} // namespace core
} // namespace openjaus



