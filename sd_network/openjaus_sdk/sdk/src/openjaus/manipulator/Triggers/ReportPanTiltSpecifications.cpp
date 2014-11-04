
/**
\file ReportPanTiltSpecifications.h

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
#include "openjaus/manipulator/Triggers/ReportPanTiltSpecifications.h"

namespace openjaus
{
namespace manipulator
{

ReportPanTiltSpecifications::ReportPanTiltSpecifications() : 
	model::Message(),
	panTiltCoordinateSysX_m(),
	panTiltCoordinateSysY_m(),
	panTiltCoordinateSysZ_m(),
	dComponentOfUnitQuaternionQ(),
	aComponentOfUnitQuaternionQ(),
	bComponentOfUnitQuaternionQ(),
	cComponentOfUnitQuaternionQ(),
	joint1MinValue_rad(),
	joint1MaxValue_rad(),
	joint1MaxSpeed_rps(),
	joint2MinValue_rad(),
	joint2MaxValue_rad(),
	joint2MaxSpeed_rps()
{
	this->id = ReportPanTiltSpecifications::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&panTiltCoordinateSysX_m);
	panTiltCoordinateSysX_m.setName("PanTiltCoordinateSysX");
	panTiltCoordinateSysX_m.setOptional(true);
	panTiltCoordinateSysX_m.setInterpretation("x coordinate of origin of pan tilt coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&panTiltCoordinateSysY_m);
	panTiltCoordinateSysY_m.setName("PanTiltCoordinateSysY");
	panTiltCoordinateSysY_m.setOptional(true);
	panTiltCoordinateSysY_m.setInterpretation("x coordinate of origin of pan tilt coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&panTiltCoordinateSysZ_m);
	panTiltCoordinateSysZ_m.setName("PanTiltCoordinateSysZ");
	panTiltCoordinateSysZ_m.setOptional(true);
	panTiltCoordinateSysZ_m.setInterpretation("x coordinate of origin of pan tilt coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&dComponentOfUnitQuaternionQ);
	dComponentOfUnitQuaternionQ.setName("DComponentOfUnitQuaternionQ");
	dComponentOfUnitQuaternionQ.setOptional(true);
	dComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&aComponentOfUnitQuaternionQ);
	aComponentOfUnitQuaternionQ.setName("AComponentOfUnitQuaternionQ");
	aComponentOfUnitQuaternionQ.setOptional(true);
	aComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&bComponentOfUnitQuaternionQ);
	bComponentOfUnitQuaternionQ.setName("BComponentOfUnitQuaternionQ");
	bComponentOfUnitQuaternionQ.setOptional(true);
	bComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&cComponentOfUnitQuaternionQ);
	cComponentOfUnitQuaternionQ.setName("CComponentOfUnitQuaternionQ");
	cComponentOfUnitQuaternionQ.setOptional(true);
	cComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&joint1MinValue_rad);
	joint1MinValue_rad.setName("Joint1MinValue");
	joint1MinValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxValue_rad);
	joint1MaxValue_rad.setName("Joint1MaxValue");
	joint1MaxValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxSpeed_rps);
	joint1MaxSpeed_rps.setName("Joint1MaxSpeed");
	joint1MaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MinValue_rad);
	joint2MinValue_rad.setName("Joint2MinValue");
	joint2MinValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxValue_rad);
	joint2MaxValue_rad.setName("Joint2MaxValue");
	joint2MaxValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxSpeed_rps);
	joint2MaxSpeed_rps.setName("Joint2MaxSpeed");
	joint2MaxSpeed_rps.setOptional(false);
	// Nothing to init

}

ReportPanTiltSpecifications::ReportPanTiltSpecifications(model::Message *message) :
	model::Message(message),
	panTiltCoordinateSysX_m(),
	panTiltCoordinateSysY_m(),
	panTiltCoordinateSysZ_m(),
	dComponentOfUnitQuaternionQ(),
	aComponentOfUnitQuaternionQ(),
	bComponentOfUnitQuaternionQ(),
	cComponentOfUnitQuaternionQ(),
	joint1MinValue_rad(),
	joint1MaxValue_rad(),
	joint1MaxSpeed_rps(),
	joint2MinValue_rad(),
	joint2MaxValue_rad(),
	joint2MaxSpeed_rps()
{
	this->id = ReportPanTiltSpecifications::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&panTiltCoordinateSysX_m);
	panTiltCoordinateSysX_m.setName("PanTiltCoordinateSysX");
	panTiltCoordinateSysX_m.setOptional(true);
	panTiltCoordinateSysX_m.setInterpretation("x coordinate of origin of pan tilt coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&panTiltCoordinateSysY_m);
	panTiltCoordinateSysY_m.setName("PanTiltCoordinateSysY");
	panTiltCoordinateSysY_m.setOptional(true);
	panTiltCoordinateSysY_m.setInterpretation("x coordinate of origin of pan tilt coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&panTiltCoordinateSysZ_m);
	panTiltCoordinateSysZ_m.setName("PanTiltCoordinateSysZ");
	panTiltCoordinateSysZ_m.setOptional(true);
	panTiltCoordinateSysZ_m.setInterpretation("x coordinate of origin of pan tilt coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&dComponentOfUnitQuaternionQ);
	dComponentOfUnitQuaternionQ.setName("DComponentOfUnitQuaternionQ");
	dComponentOfUnitQuaternionQ.setOptional(true);
	dComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&aComponentOfUnitQuaternionQ);
	aComponentOfUnitQuaternionQ.setName("AComponentOfUnitQuaternionQ");
	aComponentOfUnitQuaternionQ.setOptional(true);
	aComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&bComponentOfUnitQuaternionQ);
	bComponentOfUnitQuaternionQ.setName("BComponentOfUnitQuaternionQ");
	bComponentOfUnitQuaternionQ.setOptional(true);
	bComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&cComponentOfUnitQuaternionQ);
	cComponentOfUnitQuaternionQ.setName("CComponentOfUnitQuaternionQ");
	cComponentOfUnitQuaternionQ.setOptional(true);
	cComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&joint1MinValue_rad);
	joint1MinValue_rad.setName("Joint1MinValue");
	joint1MinValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxValue_rad);
	joint1MaxValue_rad.setName("Joint1MaxValue");
	joint1MaxValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxSpeed_rps);
	joint1MaxSpeed_rps.setName("Joint1MaxSpeed");
	joint1MaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MinValue_rad);
	joint2MinValue_rad.setName("Joint2MinValue");
	joint2MinValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxValue_rad);
	joint2MaxValue_rad.setName("Joint2MaxValue");
	joint2MaxValue_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxSpeed_rps);
	joint2MaxSpeed_rps.setName("Joint2MaxSpeed");
	joint2MaxSpeed_rps.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportPanTiltSpecifications::~ReportPanTiltSpecifications()
{

}


double ReportPanTiltSpecifications::getPanTiltCoordinateSysX_m(void)
{
	return this->panTiltCoordinateSysX_m.getValue();
}

void ReportPanTiltSpecifications::setPanTiltCoordinateSysX_m(double value)
{
	this->panTiltCoordinateSysX_m.setValue(value);
}

double ReportPanTiltSpecifications::getPanTiltCoordinateSysY_m(void)
{
	return this->panTiltCoordinateSysY_m.getValue();
}

void ReportPanTiltSpecifications::setPanTiltCoordinateSysY_m(double value)
{
	this->panTiltCoordinateSysY_m.setValue(value);
}

double ReportPanTiltSpecifications::getPanTiltCoordinateSysZ_m(void)
{
	return this->panTiltCoordinateSysZ_m.getValue();
}

void ReportPanTiltSpecifications::setPanTiltCoordinateSysZ_m(double value)
{
	this->panTiltCoordinateSysZ_m.setValue(value);
}

double ReportPanTiltSpecifications::getDComponentOfUnitQuaternionQ(void)
{
	return this->dComponentOfUnitQuaternionQ.getValue();
}

void ReportPanTiltSpecifications::setDComponentOfUnitQuaternionQ(double value)
{
	this->dComponentOfUnitQuaternionQ.setValue(value);
}

double ReportPanTiltSpecifications::getAComponentOfUnitQuaternionQ(void)
{
	return this->aComponentOfUnitQuaternionQ.getValue();
}

void ReportPanTiltSpecifications::setAComponentOfUnitQuaternionQ(double value)
{
	this->aComponentOfUnitQuaternionQ.setValue(value);
}

double ReportPanTiltSpecifications::getBComponentOfUnitQuaternionQ(void)
{
	return this->bComponentOfUnitQuaternionQ.getValue();
}

void ReportPanTiltSpecifications::setBComponentOfUnitQuaternionQ(double value)
{
	this->bComponentOfUnitQuaternionQ.setValue(value);
}

double ReportPanTiltSpecifications::getCComponentOfUnitQuaternionQ(void)
{
	return this->cComponentOfUnitQuaternionQ.getValue();
}

void ReportPanTiltSpecifications::setCComponentOfUnitQuaternionQ(double value)
{
	this->cComponentOfUnitQuaternionQ.setValue(value);
}

double ReportPanTiltSpecifications::getJoint1MinValue_rad(void)
{
	return this->joint1MinValue_rad.getValue();
}

void ReportPanTiltSpecifications::setJoint1MinValue_rad(double value)
{
	this->joint1MinValue_rad.setValue(value);
}

double ReportPanTiltSpecifications::getJoint1MaxValue_rad(void)
{
	return this->joint1MaxValue_rad.getValue();
}

void ReportPanTiltSpecifications::setJoint1MaxValue_rad(double value)
{
	this->joint1MaxValue_rad.setValue(value);
}

double ReportPanTiltSpecifications::getJoint1MaxSpeed_rps(void)
{
	return this->joint1MaxSpeed_rps.getValue();
}

void ReportPanTiltSpecifications::setJoint1MaxSpeed_rps(double value)
{
	this->joint1MaxSpeed_rps.setValue(value);
}

double ReportPanTiltSpecifications::getJoint2MinValue_rad(void)
{
	return this->joint2MinValue_rad.getValue();
}

void ReportPanTiltSpecifications::setJoint2MinValue_rad(double value)
{
	this->joint2MinValue_rad.setValue(value);
}

double ReportPanTiltSpecifications::getJoint2MaxValue_rad(void)
{
	return this->joint2MaxValue_rad.getValue();
}

void ReportPanTiltSpecifications::setJoint2MaxValue_rad(double value)
{
	this->joint2MaxValue_rad.setValue(value);
}

double ReportPanTiltSpecifications::getJoint2MaxSpeed_rps(void)
{
	return this->joint2MaxSpeed_rps.getValue();
}

void ReportPanTiltSpecifications::setJoint2MaxSpeed_rps(double value)
{
	this->joint2MaxSpeed_rps.setValue(value);
}

int ReportPanTiltSpecifications::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isPanTiltCoordinateSysXEnabled())
	{
		byteSize += dst->pack(panTiltCoordinateSysX_m);
	}
	if(this->isPanTiltCoordinateSysYEnabled())
	{
		byteSize += dst->pack(panTiltCoordinateSysY_m);
	}
	if(this->isPanTiltCoordinateSysZEnabled())
	{
		byteSize += dst->pack(panTiltCoordinateSysZ_m);
	}
	if(this->isDComponentOfUnitQuaternionQEnabled())
	{
		byteSize += dst->pack(dComponentOfUnitQuaternionQ);
	}
	if(this->isAComponentOfUnitQuaternionQEnabled())
	{
		byteSize += dst->pack(aComponentOfUnitQuaternionQ);
	}
	if(this->isBComponentOfUnitQuaternionQEnabled())
	{
		byteSize += dst->pack(bComponentOfUnitQuaternionQ);
	}
	if(this->isCComponentOfUnitQuaternionQEnabled())
	{
		byteSize += dst->pack(cComponentOfUnitQuaternionQ);
	}
	byteSize += dst->pack(joint1MinValue_rad);
	byteSize += dst->pack(joint1MaxValue_rad);
	byteSize += dst->pack(joint1MaxSpeed_rps);
	byteSize += dst->pack(joint2MinValue_rad);
	byteSize += dst->pack(joint2MaxValue_rad);
	byteSize += dst->pack(joint2MaxSpeed_rps);
	return byteSize;
}

int ReportPanTiltSpecifications::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isPanTiltCoordinateSysXEnabled())
	{
		byteSize += src->unpack(panTiltCoordinateSysX_m);
	}
	if(this->isPanTiltCoordinateSysYEnabled())
	{
		byteSize += src->unpack(panTiltCoordinateSysY_m);
	}
	if(this->isPanTiltCoordinateSysZEnabled())
	{
		byteSize += src->unpack(panTiltCoordinateSysZ_m);
	}
	if(this->isDComponentOfUnitQuaternionQEnabled())
	{
		byteSize += src->unpack(dComponentOfUnitQuaternionQ);
	}
	if(this->isAComponentOfUnitQuaternionQEnabled())
	{
		byteSize += src->unpack(aComponentOfUnitQuaternionQ);
	}
	if(this->isBComponentOfUnitQuaternionQEnabled())
	{
		byteSize += src->unpack(bComponentOfUnitQuaternionQ);
	}
	if(this->isCComponentOfUnitQuaternionQEnabled())
	{
		byteSize += src->unpack(cComponentOfUnitQuaternionQ);
	}
	byteSize += src->unpack(joint1MinValue_rad);
	byteSize += src->unpack(joint1MaxValue_rad);
	byteSize += src->unpack(joint1MaxSpeed_rps);
	byteSize += src->unpack(joint2MinValue_rad);
	byteSize += src->unpack(joint2MaxValue_rad);
	byteSize += src->unpack(joint2MaxSpeed_rps);
	return byteSize;
}

int ReportPanTiltSpecifications::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	if(this->isPanTiltCoordinateSysXEnabled())
	{
		length += panTiltCoordinateSysX_m.length(); // panTiltCoordinateSysX_m
	}
	if(this->isPanTiltCoordinateSysYEnabled())
	{
		length += panTiltCoordinateSysY_m.length(); // panTiltCoordinateSysY_m
	}
	if(this->isPanTiltCoordinateSysZEnabled())
	{
		length += panTiltCoordinateSysZ_m.length(); // panTiltCoordinateSysZ_m
	}
	if(this->isDComponentOfUnitQuaternionQEnabled())
	{
		length += dComponentOfUnitQuaternionQ.length(); // dComponentOfUnitQuaternionQ
	}
	if(this->isAComponentOfUnitQuaternionQEnabled())
	{
		length += aComponentOfUnitQuaternionQ.length(); // aComponentOfUnitQuaternionQ
	}
	if(this->isBComponentOfUnitQuaternionQEnabled())
	{
		length += bComponentOfUnitQuaternionQ.length(); // bComponentOfUnitQuaternionQ
	}
	if(this->isCComponentOfUnitQuaternionQEnabled())
	{
		length += cComponentOfUnitQuaternionQ.length(); // cComponentOfUnitQuaternionQ
	}
	length += joint1MinValue_rad.length(); // joint1MinValue_rad
	length += joint1MaxValue_rad.length(); // joint1MaxValue_rad
	length += joint1MaxSpeed_rps.length(); // joint1MaxSpeed_rps
	length += joint2MinValue_rad.length(); // joint2MinValue_rad
	length += joint2MaxValue_rad.length(); // joint2MaxValue_rad
	length += joint2MaxSpeed_rps.length(); // joint2MaxSpeed_rps
	return length;
}

std::string ReportPanTiltSpecifications::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportPanTiltSpecifications\"";
	oss << " id=\"0x4620\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPanTiltCoordinateSysXEnabled value=\"" << std::boolalpha << this->isPanTiltCoordinateSysXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPanTiltCoordinateSysYEnabled value=\"" << std::boolalpha << this->isPanTiltCoordinateSysYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPanTiltCoordinateSysZEnabled value=\"" << std::boolalpha << this->isPanTiltCoordinateSysZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isDComponentOfUnitQuaternionQEnabled value=\"" << std::boolalpha << this->isDComponentOfUnitQuaternionQEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAComponentOfUnitQuaternionQEnabled value=\"" << std::boolalpha << this->isAComponentOfUnitQuaternionQEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isBComponentOfUnitQuaternionQEnabled value=\"" << std::boolalpha << this->isBComponentOfUnitQuaternionQEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isCComponentOfUnitQuaternionQEnabled value=\"" << std::boolalpha << this->isCComponentOfUnitQuaternionQEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isPanTiltCoordinateSysXEnabled())
	{
		oss << panTiltCoordinateSysX_m.toXml(level+1); // panTiltCoordinateSysX_m
	}
	if(this->isPanTiltCoordinateSysYEnabled())
	{
		oss << panTiltCoordinateSysY_m.toXml(level+1); // panTiltCoordinateSysY_m
	}
	if(this->isPanTiltCoordinateSysZEnabled())
	{
		oss << panTiltCoordinateSysZ_m.toXml(level+1); // panTiltCoordinateSysZ_m
	}
	if(this->isDComponentOfUnitQuaternionQEnabled())
	{
		oss << dComponentOfUnitQuaternionQ.toXml(level+1); // dComponentOfUnitQuaternionQ
	}
	if(this->isAComponentOfUnitQuaternionQEnabled())
	{
		oss << aComponentOfUnitQuaternionQ.toXml(level+1); // aComponentOfUnitQuaternionQ
	}
	if(this->isBComponentOfUnitQuaternionQEnabled())
	{
		oss << bComponentOfUnitQuaternionQ.toXml(level+1); // bComponentOfUnitQuaternionQ
	}
	if(this->isCComponentOfUnitQuaternionQEnabled())
	{
		oss << cComponentOfUnitQuaternionQ.toXml(level+1); // cComponentOfUnitQuaternionQ
	}
	oss << joint1MinValue_rad.toXml(level+1); // joint1MinValue_rad
	oss << joint1MaxValue_rad.toXml(level+1); // joint1MaxValue_rad
	oss << joint1MaxSpeed_rps.toXml(level+1); // joint1MaxSpeed_rps
	oss << joint2MinValue_rad.toXml(level+1); // joint2MinValue_rad
	oss << joint2MaxValue_rad.toXml(level+1); // joint2MaxValue_rad
	oss << joint2MaxSpeed_rps.toXml(level+1); // joint2MaxSpeed_rps
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportPanTiltSpecifications::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportPanTiltSpecifications::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportPanTiltSpecifications::isPanTiltCoordinateSysXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSX_M));
}

void ReportPanTiltSpecifications::enablePanTiltCoordinateSysX(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSX_M;
}

void ReportPanTiltSpecifications::disablePanTiltCoordinateSysX(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSX_M);
}

bool ReportPanTiltSpecifications::isPanTiltCoordinateSysYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSY_M));
}

void ReportPanTiltSpecifications::enablePanTiltCoordinateSysY(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSY_M;
}

void ReportPanTiltSpecifications::disablePanTiltCoordinateSysY(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSY_M);
}

bool ReportPanTiltSpecifications::isPanTiltCoordinateSysZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSZ_M));
}

void ReportPanTiltSpecifications::enablePanTiltCoordinateSysZ(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSZ_M;
}

void ReportPanTiltSpecifications::disablePanTiltCoordinateSysZ(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::PANTILTCOORDINATESYSZ_M);
}

bool ReportPanTiltSpecifications::isDComponentOfUnitQuaternionQEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::DCOMPONENTOFUNITQUATERNIONQ));
}

void ReportPanTiltSpecifications::enableDComponentOfUnitQuaternionQ(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::DCOMPONENTOFUNITQUATERNIONQ;
}

void ReportPanTiltSpecifications::disableDComponentOfUnitQuaternionQ(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::DCOMPONENTOFUNITQUATERNIONQ);
}

bool ReportPanTiltSpecifications::isAComponentOfUnitQuaternionQEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::ACOMPONENTOFUNITQUATERNIONQ));
}

void ReportPanTiltSpecifications::enableAComponentOfUnitQuaternionQ(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::ACOMPONENTOFUNITQUATERNIONQ;
}

void ReportPanTiltSpecifications::disableAComponentOfUnitQuaternionQ(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::ACOMPONENTOFUNITQUATERNIONQ);
}

bool ReportPanTiltSpecifications::isBComponentOfUnitQuaternionQEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::BCOMPONENTOFUNITQUATERNIONQ));
}

void ReportPanTiltSpecifications::enableBComponentOfUnitQuaternionQ(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::BCOMPONENTOFUNITQUATERNIONQ;
}

void ReportPanTiltSpecifications::disableBComponentOfUnitQuaternionQ(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::BCOMPONENTOFUNITQUATERNIONQ);
}

bool ReportPanTiltSpecifications::isCComponentOfUnitQuaternionQEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPanTiltSpecifications::CCOMPONENTOFUNITQUATERNIONQ));
}

void ReportPanTiltSpecifications::enableCComponentOfUnitQuaternionQ(void)
{
	this->presenceVector |= 0x01 << ReportPanTiltSpecifications::CCOMPONENTOFUNITQUATERNIONQ;
}

void ReportPanTiltSpecifications::disableCComponentOfUnitQuaternionQ(void)
{
	this->presenceVector &= ~(0x01 << ReportPanTiltSpecifications::CCOMPONENTOFUNITQUATERNIONQ);
}

} // namespace manipulator
} // namespace openjaus


