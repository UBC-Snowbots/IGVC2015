/**
\file QSLComponentList.h

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
#include "openjaus/core/Triggers/Fields/QSLComponentList.h"

namespace openjaus
{
namespace core
{

QSLComponentList::QSLComponentList() : qSLComponentRec()
{

}

QSLComponentList::~QSLComponentList()
{

}

void QSLComponentList::copy(QSLComponentList& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->qSLComponentRec = source.getQSLComponentRec();
}

int QSLComponentList::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->qSLComponentRec.size())); // Count
	
	for(size_t i = 0; i < this->qSLComponentRec.size(); i++) // Elements
	{
		result += dst->pack(this->qSLComponentRec[i]); 
	}
	return result;
}

int QSLComponentList::from(system::Buffer *src)
{
	int result = 0;
	uint8_t count;

	result += src->unpack(count); // Count
	this->qSLComponentRec.empty();
	this->qSLComponentRec.reserve(count);
	
	for(size_t i = 0; i < count; i++) // Elements
	{
		QSLComponentRecord temp;
		result += src->unpack(temp);
		this->qSLComponentRec.push_back(temp);
	}
	return result;
}

int QSLComponentList::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Count
	
	for(size_t i = 0; i < this->qSLComponentRec.size(); i++) // Elements
	{
		result += this->qSLComponentRec[i].length(); 
	}
	return result;
}

std::string QSLComponentList::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<List name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<count>" << this->qSLComponentRec.size() << "</count>\n";
	
	for(size_t i = 0; i < this->qSLComponentRec.size(); i++) // Elements
	{
		oss << this->qSLComponentRec.at(i).toXml(level + 1);
	}
	oss << prefix.str() << "</List>\n";
	return oss.str();
}

std::vector<QSLComponentRecord>& QSLComponentList::getQSLComponentRec()
{
	return this->qSLComponentRec;
}

void QSLComponentList::add(QSLComponentRecord value)
{
	this->qSLComponentRec.push_back(value);
}

QSLComponentRecord& QSLComponentList::get(size_t index)
{
	return this->qSLComponentRec.at(index);
}

size_t QSLComponentList::size()
{
	return this->qSLComponentRec.size();
}

bool QSLComponentList::empty()
{
	return this->qSLComponentRec.empty();
}

void QSLComponentList::clear()
{
	this->qSLComponentRec.clear();
}

void QSLComponentList::remove(int index)
{
	this->qSLComponentRec.erase(this->qSLComponentRec.begin() + index);
}


} // namespace core
} // namespace openjaus



