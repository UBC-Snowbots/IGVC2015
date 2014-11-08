/**
\file JointVelocityRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/JointVelocityRecord.h"

namespace openjaus
{
namespace manipulator
{

JointVelocityRecord::JointVelocityRecord():
	jointVelocity()
{

	fields.push_back(&jointVelocity);
	jointVelocity.setName("JointVelocity");
	jointVelocity.setOptional(false);
	jointVelocity.setInterpretation("Scaled integer representing the command velocity for this joint. Each joint must have a corresponding entry in the list.  Units and scale range are based on the joint type.");
	// Nothing to Init

}

JointVelocityRecord::JointVelocityRecord(const JointVelocityRecord &source)
{
	this->copy(const_cast<JointVelocityRecord&>(source));
}

JointVelocityRecord::~JointVelocityRecord()
{

}


JointVelocityVariant& JointVelocityRecord::getJointVelocity(void)
{
	return this->jointVelocity;
}

int JointVelocityRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(jointVelocity);
	return byteSize;
}
int JointVelocityRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(jointVelocity);
	return byteSize;
}

int JointVelocityRecord::length(void)
{
	int length = 0;
	length += jointVelocity.length(); // jointVelocity
	return length;
}

std::string JointVelocityRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"JointVelocityRecord\">\n";
	oss << jointVelocity.toXml(level+1); // jointVelocity
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void JointVelocityRecord::copy(JointVelocityRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->jointVelocity.copy(source.getJointVelocity());
}

} // namespace manipulator
} // namespace openjaus

