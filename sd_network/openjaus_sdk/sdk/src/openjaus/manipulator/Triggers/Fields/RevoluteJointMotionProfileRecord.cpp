/**
\file RevoluteJointMotionProfileRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/RevoluteJointMotionProfileRecord.h"

namespace openjaus
{
namespace manipulator
{

RevoluteJointMotionProfileRecord::RevoluteJointMotionProfileRecord():
	jointMaxSpeed_rps(),
	jointMaxAccelerationRate_rps2(),
	jointMaxDecelerationRate_rps2()
{

	fields.push_back(&jointMaxSpeed_rps);
	jointMaxSpeed_rps.setName("JointMaxSpeed");
	jointMaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&jointMaxAccelerationRate_rps2);
	jointMaxAccelerationRate_rps2.setName("JointMaxAccelerationRate");
	jointMaxAccelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&jointMaxDecelerationRate_rps2);
	jointMaxDecelerationRate_rps2.setName("JointMaxDecelerationRate");
	jointMaxDecelerationRate_rps2.setOptional(false);
	// Nothing to init

}

RevoluteJointMotionProfileRecord::RevoluteJointMotionProfileRecord(const RevoluteJointMotionProfileRecord &source)
{
	this->copy(const_cast<RevoluteJointMotionProfileRecord&>(source));
}

RevoluteJointMotionProfileRecord::~RevoluteJointMotionProfileRecord()
{

}


double RevoluteJointMotionProfileRecord::getJointMaxSpeed_rps(void)
{
	return this->jointMaxSpeed_rps.getValue();
}

void RevoluteJointMotionProfileRecord::setJointMaxSpeed_rps(double value)
{
	this->jointMaxSpeed_rps.setValue(value);
}

double RevoluteJointMotionProfileRecord::getJointMaxAccelerationRate_rps2(void)
{
	return this->jointMaxAccelerationRate_rps2.getValue();
}

void RevoluteJointMotionProfileRecord::setJointMaxAccelerationRate_rps2(double value)
{
	this->jointMaxAccelerationRate_rps2.setValue(value);
}

double RevoluteJointMotionProfileRecord::getJointMaxDecelerationRate_rps2(void)
{
	return this->jointMaxDecelerationRate_rps2.getValue();
}

void RevoluteJointMotionProfileRecord::setJointMaxDecelerationRate_rps2(double value)
{
	this->jointMaxDecelerationRate_rps2.setValue(value);
}

int RevoluteJointMotionProfileRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(jointMaxSpeed_rps);
	byteSize += dst->pack(jointMaxAccelerationRate_rps2);
	byteSize += dst->pack(jointMaxDecelerationRate_rps2);
	return byteSize;
}
int RevoluteJointMotionProfileRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(jointMaxSpeed_rps);
	byteSize += src->unpack(jointMaxAccelerationRate_rps2);
	byteSize += src->unpack(jointMaxDecelerationRate_rps2);
	return byteSize;
}

int RevoluteJointMotionProfileRecord::length(void)
{
	int length = 0;
	length += jointMaxSpeed_rps.length(); // jointMaxSpeed_rps
	length += jointMaxAccelerationRate_rps2.length(); // jointMaxAccelerationRate_rps2
	length += jointMaxDecelerationRate_rps2.length(); // jointMaxDecelerationRate_rps2
	return length;
}

std::string RevoluteJointMotionProfileRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RevoluteJointMotionProfileRecord\">\n";
	oss << jointMaxSpeed_rps.toXml(level+1); // jointMaxSpeed_rps
	oss << jointMaxAccelerationRate_rps2.toXml(level+1); // jointMaxAccelerationRate_rps2
	oss << jointMaxDecelerationRate_rps2.toXml(level+1); // jointMaxDecelerationRate_rps2
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void RevoluteJointMotionProfileRecord::copy(RevoluteJointMotionProfileRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->jointMaxSpeed_rps.setName("JointMaxSpeed");
	this->jointMaxSpeed_rps.setOptional(false);
	this->jointMaxSpeed_rps.setValue(source.getJointMaxSpeed_rps()); 
 
	this->jointMaxAccelerationRate_rps2.setName("JointMaxAccelerationRate");
	this->jointMaxAccelerationRate_rps2.setOptional(false);
	this->jointMaxAccelerationRate_rps2.setValue(source.getJointMaxAccelerationRate_rps2()); 
 
	this->jointMaxDecelerationRate_rps2.setName("JointMaxDecelerationRate");
	this->jointMaxDecelerationRate_rps2.setOptional(false);
	this->jointMaxDecelerationRate_rps2.setValue(source.getJointMaxDecelerationRate_rps2()); 
 
}

} // namespace manipulator
} // namespace openjaus

