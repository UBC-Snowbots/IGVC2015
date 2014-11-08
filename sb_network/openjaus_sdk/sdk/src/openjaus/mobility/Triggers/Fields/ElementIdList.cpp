/**
\file ElementIdList.h

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
#include "openjaus/mobility/Triggers/Fields/ElementIdList.h"

namespace openjaus
{
namespace mobility
{

ElementIdList::ElementIdList() : elementUID()
{

}

ElementIdList::~ElementIdList()
{

}

void ElementIdList::copy(ElementIdList& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->elementUID = source.getElementUID();
}

int ElementIdList::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint16_t>(this->elementUID.size())); // Count
	
	for(size_t i = 0; i < this->elementUID.size(); i++) // Elements
	{
		result += dst->pack(this->elementUID[i]); 
	}
	return result;
}

int ElementIdList::from(system::Buffer *src)
{
	int result = 0;
	uint16_t count;

	result += src->unpack(count); // Count
	this->elementUID.empty();
	this->elementUID.reserve(count);
	
	for(size_t i = 0; i < count; i++) // Elements
	{
		model::fields::UnsignedShort temp;
		result += src->unpack(temp);
		this->elementUID.push_back(temp);
	}
	return result;
}

int ElementIdList::length(void)
{
	int result = 0;
	result += sizeof(uint16_t); // Count
	
	for(size_t i = 0; i < this->elementUID.size(); i++) // Elements
	{
		result += this->elementUID[i].length(); 
	}
	return result;
}

std::string ElementIdList::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<List name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<count>" << this->elementUID.size() << "</count>\n";
	
	for(size_t i = 0; i < this->elementUID.size(); i++) // Elements
	{
		oss << this->elementUID.at(i).toXml(level + 1);
	}
	oss << prefix.str() << "</List>\n";
	return oss.str();
}

std::vector<model::fields::UnsignedShort>& ElementIdList::getElementUID()
{
	return this->elementUID;
}

void ElementIdList::add(model::fields::UnsignedShort value)
{
	this->elementUID.push_back(value);
}

model::fields::UnsignedShort& ElementIdList::get(size_t index)
{
	return this->elementUID.at(index);
}

size_t ElementIdList::size()
{
	return this->elementUID.size();
}

bool ElementIdList::empty()
{
	return this->elementUID.empty();
}

void ElementIdList::clear()
{
	this->elementUID.clear();
}

void ElementIdList::remove(int index)
{
	this->elementUID.erase(this->elementUID.begin() + index);
}


} // namespace mobility
} // namespace openjaus



