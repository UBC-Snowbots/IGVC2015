
/**
\file ReportPlatformSpecifications.h

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
#include "openjaus/ugv/Triggers/ReportPlatformSpecifications.h"

namespace openjaus
{
namespace ugv
{

ReportPlatformSpecifications::ReportPlatformSpecifications() : 
	model::Message(),
	mobilityPlatformName(),
	front_m(),
	back_m(),
	right_m(),
	left_m(),
	bottom_m(),
	top_m(),
	xcg_m(),
	ycg_m(),
	zcg_m(),
	wheelBase_m(),
	staticPitchOver_rad(),
	staticRollOver_rad()
{
	this->id = ReportPlatformSpecifications::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&mobilityPlatformName);
	mobilityPlatformName.setName("MobilityPlatformName");
	mobilityPlatformName.setOptional(false);
	mobilityPlatformName.setMaxLength(15);

	fields.push_back(&front_m);
	front_m.setName("Front");
	front_m.setOptional(true);
	front_m.setValue(0);

	fields.push_back(&back_m);
	back_m.setName("Back");
	back_m.setOptional(true);
	back_m.setValue(0);

	fields.push_back(&right_m);
	right_m.setName("Right");
	right_m.setOptional(true);
	right_m.setValue(0);

	fields.push_back(&left_m);
	left_m.setName("Left");
	left_m.setOptional(true);
	left_m.setValue(0);

	fields.push_back(&bottom_m);
	bottom_m.setName("Bottom");
	bottom_m.setOptional(true);
	bottom_m.setValue(0);

	fields.push_back(&top_m);
	top_m.setName("Top");
	top_m.setOptional(true);
	top_m.setValue(0);

	fields.push_back(&xcg_m);
	xcg_m.setName("Xcg");
	xcg_m.setOptional(true);
	xcg_m.setValue(0);

	fields.push_back(&ycg_m);
	ycg_m.setName("Ycg");
	ycg_m.setOptional(true);
	ycg_m.setValue(0);

	fields.push_back(&zcg_m);
	zcg_m.setName("Zcg");
	zcg_m.setOptional(true);
	zcg_m.setValue(0);

	fields.push_back(&wheelBase_m);
	wheelBase_m.setName("WheelBase");
	wheelBase_m.setOptional(true);
	wheelBase_m.setValue(0);

	fields.push_back(&staticPitchOver_rad);
	staticPitchOver_rad.setName("StaticPitchOver");
	staticPitchOver_rad.setOptional(true);
	staticPitchOver_rad.setValue(0);

	fields.push_back(&staticRollOver_rad);
	staticRollOver_rad.setName("StaticRollOver");
	staticRollOver_rad.setOptional(true);
	staticRollOver_rad.setValue(0);

}

ReportPlatformSpecifications::ReportPlatformSpecifications(model::Message *message) :
	model::Message(message),
	mobilityPlatformName(),
	front_m(),
	back_m(),
	right_m(),
	left_m(),
	bottom_m(),
	top_m(),
	xcg_m(),
	ycg_m(),
	zcg_m(),
	wheelBase_m(),
	staticPitchOver_rad(),
	staticRollOver_rad()
{
	this->id = ReportPlatformSpecifications::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&mobilityPlatformName);
	mobilityPlatformName.setName("MobilityPlatformName");
	mobilityPlatformName.setOptional(false);
	mobilityPlatformName.setMaxLength(15);

	fields.push_back(&front_m);
	front_m.setName("Front");
	front_m.setOptional(true);
	front_m.setValue(0);

	fields.push_back(&back_m);
	back_m.setName("Back");
	back_m.setOptional(true);
	back_m.setValue(0);

	fields.push_back(&right_m);
	right_m.setName("Right");
	right_m.setOptional(true);
	right_m.setValue(0);

	fields.push_back(&left_m);
	left_m.setName("Left");
	left_m.setOptional(true);
	left_m.setValue(0);

	fields.push_back(&bottom_m);
	bottom_m.setName("Bottom");
	bottom_m.setOptional(true);
	bottom_m.setValue(0);

	fields.push_back(&top_m);
	top_m.setName("Top");
	top_m.setOptional(true);
	top_m.setValue(0);

	fields.push_back(&xcg_m);
	xcg_m.setName("Xcg");
	xcg_m.setOptional(true);
	xcg_m.setValue(0);

	fields.push_back(&ycg_m);
	ycg_m.setName("Ycg");
	ycg_m.setOptional(true);
	ycg_m.setValue(0);

	fields.push_back(&zcg_m);
	zcg_m.setName("Zcg");
	zcg_m.setOptional(true);
	zcg_m.setValue(0);

	fields.push_back(&wheelBase_m);
	wheelBase_m.setName("WheelBase");
	wheelBase_m.setOptional(true);
	wheelBase_m.setValue(0);

	fields.push_back(&staticPitchOver_rad);
	staticPitchOver_rad.setName("StaticPitchOver");
	staticPitchOver_rad.setOptional(true);
	staticPitchOver_rad.setValue(0);

	fields.push_back(&staticRollOver_rad);
	staticRollOver_rad.setName("StaticRollOver");
	staticRollOver_rad.setOptional(true);
	staticRollOver_rad.setValue(0);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportPlatformSpecifications::~ReportPlatformSpecifications()
{

}


std::string ReportPlatformSpecifications::getMobilityPlatformName(void)
{
	return this->mobilityPlatformName.getValue();
}

void ReportPlatformSpecifications::setMobilityPlatformName(std::string value)
{
	this->mobilityPlatformName.setValue(value);
}

uint16_t ReportPlatformSpecifications::getFront_m(void)
{
	return this->front_m.getValue();
}

void ReportPlatformSpecifications::setFront_m(uint16_t value)
{
	this->front_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getBack_m(void)
{
	return this->back_m.getValue();
}

void ReportPlatformSpecifications::setBack_m(uint16_t value)
{
	this->back_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getRight_m(void)
{
	return this->right_m.getValue();
}

void ReportPlatformSpecifications::setRight_m(uint16_t value)
{
	this->right_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getLeft_m(void)
{
	return this->left_m.getValue();
}

void ReportPlatformSpecifications::setLeft_m(uint16_t value)
{
	this->left_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getBottom_m(void)
{
	return this->bottom_m.getValue();
}

void ReportPlatformSpecifications::setBottom_m(uint16_t value)
{
	this->bottom_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getTop_m(void)
{
	return this->top_m.getValue();
}

void ReportPlatformSpecifications::setTop_m(uint16_t value)
{
	this->top_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getXcg_m(void)
{
	return this->xcg_m.getValue();
}

void ReportPlatformSpecifications::setXcg_m(uint16_t value)
{
	this->xcg_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getYcg_m(void)
{
	return this->ycg_m.getValue();
}

void ReportPlatformSpecifications::setYcg_m(uint16_t value)
{
	this->ycg_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getZcg_m(void)
{
	return this->zcg_m.getValue();
}

void ReportPlatformSpecifications::setZcg_m(uint16_t value)
{
	this->zcg_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getWheelBase_m(void)
{
	return this->wheelBase_m.getValue();
}

void ReportPlatformSpecifications::setWheelBase_m(uint16_t value)
{
	this->wheelBase_m.setValue(value);
}

uint16_t ReportPlatformSpecifications::getStaticPitchOver_rad(void)
{
	return this->staticPitchOver_rad.getValue();
}

void ReportPlatformSpecifications::setStaticPitchOver_rad(uint16_t value)
{
	this->staticPitchOver_rad.setValue(value);
}

uint16_t ReportPlatformSpecifications::getStaticRollOver_rad(void)
{
	return this->staticRollOver_rad.getValue();
}

void ReportPlatformSpecifications::setStaticRollOver_rad(uint16_t value)
{
	this->staticRollOver_rad.setValue(value);
}

int ReportPlatformSpecifications::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(mobilityPlatformName);
	if(this->isFrontEnabled())
	{
		byteSize += dst->pack(front_m);
	}
	if(this->isBackEnabled())
	{
		byteSize += dst->pack(back_m);
	}
	if(this->isRightEnabled())
	{
		byteSize += dst->pack(right_m);
	}
	if(this->isLeftEnabled())
	{
		byteSize += dst->pack(left_m);
	}
	if(this->isBottomEnabled())
	{
		byteSize += dst->pack(bottom_m);
	}
	if(this->isTopEnabled())
	{
		byteSize += dst->pack(top_m);
	}
	if(this->isXcgEnabled())
	{
		byteSize += dst->pack(xcg_m);
	}
	if(this->isYcgEnabled())
	{
		byteSize += dst->pack(ycg_m);
	}
	if(this->isZcgEnabled())
	{
		byteSize += dst->pack(zcg_m);
	}
	if(this->isWheelBaseEnabled())
	{
		byteSize += dst->pack(wheelBase_m);
	}
	if(this->isStaticPitchOverEnabled())
	{
		byteSize += dst->pack(staticPitchOver_rad);
	}
	if(this->isStaticRollOverEnabled())
	{
		byteSize += dst->pack(staticRollOver_rad);
	}
	return byteSize;
}

int ReportPlatformSpecifications::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(mobilityPlatformName);
	if(this->isFrontEnabled())
	{
		byteSize += src->unpack(front_m);
	}
	if(this->isBackEnabled())
	{
		byteSize += src->unpack(back_m);
	}
	if(this->isRightEnabled())
	{
		byteSize += src->unpack(right_m);
	}
	if(this->isLeftEnabled())
	{
		byteSize += src->unpack(left_m);
	}
	if(this->isBottomEnabled())
	{
		byteSize += src->unpack(bottom_m);
	}
	if(this->isTopEnabled())
	{
		byteSize += src->unpack(top_m);
	}
	if(this->isXcgEnabled())
	{
		byteSize += src->unpack(xcg_m);
	}
	if(this->isYcgEnabled())
	{
		byteSize += src->unpack(ycg_m);
	}
	if(this->isZcgEnabled())
	{
		byteSize += src->unpack(zcg_m);
	}
	if(this->isWheelBaseEnabled())
	{
		byteSize += src->unpack(wheelBase_m);
	}
	if(this->isStaticPitchOverEnabled())
	{
		byteSize += src->unpack(staticPitchOver_rad);
	}
	if(this->isStaticRollOverEnabled())
	{
		byteSize += src->unpack(staticRollOver_rad);
	}
	return byteSize;
}

int ReportPlatformSpecifications::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint16_t); // PresenceVector
	length += mobilityPlatformName.length(); // mobilityPlatformName
	if(this->isFrontEnabled())
	{
		length += front_m.length(); // front_m
	}
	if(this->isBackEnabled())
	{
		length += back_m.length(); // back_m
	}
	if(this->isRightEnabled())
	{
		length += right_m.length(); // right_m
	}
	if(this->isLeftEnabled())
	{
		length += left_m.length(); // left_m
	}
	if(this->isBottomEnabled())
	{
		length += bottom_m.length(); // bottom_m
	}
	if(this->isTopEnabled())
	{
		length += top_m.length(); // top_m
	}
	if(this->isXcgEnabled())
	{
		length += xcg_m.length(); // xcg_m
	}
	if(this->isYcgEnabled())
	{
		length += ycg_m.length(); // ycg_m
	}
	if(this->isZcgEnabled())
	{
		length += zcg_m.length(); // zcg_m
	}
	if(this->isWheelBaseEnabled())
	{
		length += wheelBase_m.length(); // wheelBase_m
	}
	if(this->isStaticPitchOverEnabled())
	{
		length += staticPitchOver_rad.length(); // staticPitchOver_rad
	}
	if(this->isStaticRollOverEnabled())
	{
		length += staticRollOver_rad.length(); // staticRollOver_rad
	}
	return length;
}

std::string ReportPlatformSpecifications::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportPlatformSpecifications\"";
	oss << " id=\"0x4502\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFrontEnabled value=\"" << std::boolalpha << this->isFrontEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isBackEnabled value=\"" << std::boolalpha << this->isBackEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRightEnabled value=\"" << std::boolalpha << this->isRightEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isLeftEnabled value=\"" << std::boolalpha << this->isLeftEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isBottomEnabled value=\"" << std::boolalpha << this->isBottomEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTopEnabled value=\"" << std::boolalpha << this->isTopEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isXcgEnabled value=\"" << std::boolalpha << this->isXcgEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYcgEnabled value=\"" << std::boolalpha << this->isYcgEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isZcgEnabled value=\"" << std::boolalpha << this->isZcgEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isWheelBaseEnabled value=\"" << std::boolalpha << this->isWheelBaseEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isStaticPitchOverEnabled value=\"" << std::boolalpha << this->isStaticPitchOverEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isStaticRollOverEnabled value=\"" << std::boolalpha << this->isStaticRollOverEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << mobilityPlatformName.toXml(level+1); // mobilityPlatformName
	if(this->isFrontEnabled())
	{
		oss << front_m.toXml(level+1); // front_m
	}
	if(this->isBackEnabled())
	{
		oss << back_m.toXml(level+1); // back_m
	}
	if(this->isRightEnabled())
	{
		oss << right_m.toXml(level+1); // right_m
	}
	if(this->isLeftEnabled())
	{
		oss << left_m.toXml(level+1); // left_m
	}
	if(this->isBottomEnabled())
	{
		oss << bottom_m.toXml(level+1); // bottom_m
	}
	if(this->isTopEnabled())
	{
		oss << top_m.toXml(level+1); // top_m
	}
	if(this->isXcgEnabled())
	{
		oss << xcg_m.toXml(level+1); // xcg_m
	}
	if(this->isYcgEnabled())
	{
		oss << ycg_m.toXml(level+1); // ycg_m
	}
	if(this->isZcgEnabled())
	{
		oss << zcg_m.toXml(level+1); // zcg_m
	}
	if(this->isWheelBaseEnabled())
	{
		oss << wheelBase_m.toXml(level+1); // wheelBase_m
	}
	if(this->isStaticPitchOverEnabled())
	{
		oss << staticPitchOver_rad.toXml(level+1); // staticPitchOver_rad
	}
	if(this->isStaticRollOverEnabled())
	{
		oss << staticRollOver_rad.toXml(level+1); // staticRollOver_rad
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportPlatformSpecifications::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t ReportPlatformSpecifications::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportPlatformSpecifications::isFrontEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::FRONT_M));
}

void ReportPlatformSpecifications::enableFront(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::FRONT_M;
}

void ReportPlatformSpecifications::disableFront(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::FRONT_M);
}

bool ReportPlatformSpecifications::isBackEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::BACK_M));
}

void ReportPlatformSpecifications::enableBack(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::BACK_M;
}

void ReportPlatformSpecifications::disableBack(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::BACK_M);
}

bool ReportPlatformSpecifications::isRightEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::RIGHT_M));
}

void ReportPlatformSpecifications::enableRight(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::RIGHT_M;
}

void ReportPlatformSpecifications::disableRight(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::RIGHT_M);
}

bool ReportPlatformSpecifications::isLeftEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::LEFT_M));
}

void ReportPlatformSpecifications::enableLeft(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::LEFT_M;
}

void ReportPlatformSpecifications::disableLeft(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::LEFT_M);
}

bool ReportPlatformSpecifications::isBottomEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::BOTTOM_M));
}

void ReportPlatformSpecifications::enableBottom(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::BOTTOM_M;
}

void ReportPlatformSpecifications::disableBottom(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::BOTTOM_M);
}

bool ReportPlatformSpecifications::isTopEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::TOP_M));
}

void ReportPlatformSpecifications::enableTop(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::TOP_M;
}

void ReportPlatformSpecifications::disableTop(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::TOP_M);
}

bool ReportPlatformSpecifications::isXcgEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::XCG_M));
}

void ReportPlatformSpecifications::enableXcg(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::XCG_M;
}

void ReportPlatformSpecifications::disableXcg(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::XCG_M);
}

bool ReportPlatformSpecifications::isYcgEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::YCG_M));
}

void ReportPlatformSpecifications::enableYcg(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::YCG_M;
}

void ReportPlatformSpecifications::disableYcg(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::YCG_M);
}

bool ReportPlatformSpecifications::isZcgEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::ZCG_M));
}

void ReportPlatformSpecifications::enableZcg(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::ZCG_M;
}

void ReportPlatformSpecifications::disableZcg(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::ZCG_M);
}

bool ReportPlatformSpecifications::isWheelBaseEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::WHEELBASE_M));
}

void ReportPlatformSpecifications::enableWheelBase(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::WHEELBASE_M;
}

void ReportPlatformSpecifications::disableWheelBase(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::WHEELBASE_M);
}

bool ReportPlatformSpecifications::isStaticPitchOverEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::STATICPITCHOVER_RAD));
}

void ReportPlatformSpecifications::enableStaticPitchOver(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::STATICPITCHOVER_RAD;
}

void ReportPlatformSpecifications::disableStaticPitchOver(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::STATICPITCHOVER_RAD);
}

bool ReportPlatformSpecifications::isStaticRollOverEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportPlatformSpecifications::STATICROLLOVER_RAD));
}

void ReportPlatformSpecifications::enableStaticRollOver(void)
{
	this->presenceVector |= 0x01 << ReportPlatformSpecifications::STATICROLLOVER_RAD;
}

void ReportPlatformSpecifications::disableStaticRollOver(void)
{
	this->presenceVector &= ~(0x01 << ReportPlatformSpecifications::STATICROLLOVER_RAD);
}

} // namespace ugv
} // namespace openjaus


