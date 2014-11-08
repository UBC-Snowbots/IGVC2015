/**
\file PrismaticJointMotionProfileRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/PrismaticJointMotionProfileRecord.h"

namespace openjaus
{
namespace manipulator
{

PrismaticJointMotionProfileRecord::PrismaticJointMotionProfileRecord():
	jointMaxSpeed_mps(),
	jointMaxAccelerationRate_mps2(),
	jointMaxDecelerationRate_mps2()
{

	fields.push_back(&jointMaxSpeed_mps);
	jointMaxSpeed_mps.setName("JointMaxSpeed");
	jointMaxSpeed_mps.setOptional(false);
	// Nothing to init

	fields.push_back(&jointMaxAccelerationRate_mps2);
	jointMaxAccelerationRate_mps2.setName("JointMaxAccelerationRate");
	jointMaxAccelerationRate_mps2.setOptional(false);
	// Nothing to init

	fields.push_back(&jointMaxDecelerationRate_mps2);
	jointMaxDecelerationRate_mps2.setName("JointMaxDecelerationRate");
	jointMaxDecelerationRate_mps2.setOptional(false);
	// Nothing to init

}

PrismaticJointMotionProfileRecord::PrismaticJointMotionProfileRecord(const PrismaticJointMotionProfileRecord &source)
{
	this->copy(const_cast<PrismaticJointMotionProfileRecord&>(source));
}

PrismaticJointMotionProfileRecord::~PrismaticJointMotionProfileRecord()
{

}


double PrismaticJointMotionProfileRecord::getJointMaxSpeed_mps(void)
{
	return this->jointMaxSpeed_mps.getValue();
}

void PrismaticJointMotionProfileRecord::setJointMaxSpeed_mps(double value)
{
	this->jointMaxSpeed_mps.setValue(value);
}

double PrismaticJointMotionProfileRecord::getJointMaxAccelerationRate_mps2(void)
{
	return this->jointMaxAccelerationRate_mps2.getValue();
}

void PrismaticJointMotionProfileRecord::setJointMaxAccelerationRate_mps2(double value)
{
	this->jointMaxAccelerationRate_mps2.setValue(value);
}

double PrismaticJointMotionProfileRecord::getJointMaxDecelerationRate_mps2(void)
{
	return this->jointMaxDecelerationRate_mps2.getValue();
}

void PrismaticJointMotionProfileRecord::setJointMaxDecelerationRate_mps2(double value)
{
	this->jointMaxDecelerationRate_mps2.setValue(value);
}

int PrismaticJointMotionProfileRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(jointMaxSpeed_mps);
	byteSize += dst->pack(jointMaxAccelerationRate_mps2);
	byteSize += dst->pack(jointMaxDecelerationRate_mps2);
	return byteSize;
}
int PrismaticJointMotionProfileRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(jointMaxSpeed_mps);
	byteSize += src->unpack(jointMaxAccelerationRate_mps2);
	byteSize += src->unpack(jointMaxDecelerationRate_mps2);
	return byteSize;
}

int PrismaticJointMotionProfileRecord::length(void)
{
	int length = 0;
	length += jointMaxSpeed_mps.length(); // jointMaxSpeed_mps
	length += jointMaxAccelerationRate_mps2.length(); // jointMaxAccelerationRate_mps2
	length += jointMaxDecelerationRate_mps2.length(); // jointMaxDecelerationRate_mps2
	return length;
}

std::string PrismaticJointMotionProfileRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"PrismaticJointMotionProfileRecord\">\n";
	oss << jointMaxSpeed_mps.toXml(level+1); // jointMaxSpeed_mps
	oss << jointMaxAccelerationRate_mps2.toXml(level+1); // jointMaxAccelerationRate_mps2
	oss << jointMaxDecelerationRate_mps2.toXml(level+1); // jointMaxDecelerationRate_mps2
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void PrismaticJointMotionProfileRecord::copy(PrismaticJointMotionProfileRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->jointMaxSpeed_mps.setName("JointMaxSpeed");
	this->jointMaxSpeed_mps.setOptional(false);
	this->jointMaxSpeed_mps.setValue(source.getJointMaxSpeed_mps()); 
 
	this->jointMaxAccelerationRate_mps2.setName("JointMaxAccelerationRate");
	this->jointMaxAccelerationRate_mps2.setOptional(false);
	this->jointMaxAccelerationRate_mps2.setValue(source.getJointMaxAccelerationRate_mps2()); 
 
	this->jointMaxDecelerationRate_mps2.setName("JointMaxDecelerationRate");
	this->jointMaxDecelerationRate_mps2.setOptional(false);
	this->jointMaxDecelerationRate_mps2.setValue(source.getJointMaxDecelerationRate_mps2()); 
 
}

} // namespace manipulator
} // namespace openjaus

