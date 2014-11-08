
/**
\file WrenchEffortRec.h

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

#ifndef WRENCHEFFORTRECORD_H
#define WRENCHEFFORTRECORD_H

#include <openjaus.h>
#include "openjaus/mobility/Triggers/Fields/PropulsiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PropulsiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PropulsiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PropulsiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PropulsiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PropulsiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/ResistiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/ResistiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/ResistiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/ResistiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/ResistiveEffortScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/ResistiveEffortScaledInteger.h"

namespace openjaus
{
namespace mobility
{

class OPENJAUS_EXPORT WrenchEffortRecord : public openjaus::model::fields::Record
{
public:
	WrenchEffortRecord();
	WrenchEffortRecord(const WrenchEffortRecord &source);
	~WrenchEffortRecord();

	void copy(WrenchEffortRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint16_t value);
	uint16_t getPresenceVector(void) const;
	bool isPropulsiveLinearEffortXEnabled(void) const;
	void enablePropulsiveLinearEffortX(void);
	void disablePropulsiveLinearEffortX(void);

	bool isPropulsiveLinearEffortYEnabled(void) const;
	void enablePropulsiveLinearEffortY(void);
	void disablePropulsiveLinearEffortY(void);

	bool isPropulsiveLinearEffortZEnabled(void) const;
	void enablePropulsiveLinearEffortZ(void);
	void disablePropulsiveLinearEffortZ(void);

	bool isPropulsiveRotationalEffortXEnabled(void) const;
	void enablePropulsiveRotationalEffortX(void);
	void disablePropulsiveRotationalEffortX(void);

	bool isPropulsiveRotationalEffortYEnabled(void) const;
	void enablePropulsiveRotationalEffortY(void);
	void disablePropulsiveRotationalEffortY(void);

	bool isPropulsiveRotationalEffortZEnabled(void) const;
	void enablePropulsiveRotationalEffortZ(void);
	void disablePropulsiveRotationalEffortZ(void);

	bool isResistiveLinearEffortXEnabled(void) const;
	void enableResistiveLinearEffortX(void);
	void disableResistiveLinearEffortX(void);

	bool isResistiveLinearEffortYEnabled(void) const;
	void enableResistiveLinearEffortY(void);
	void disableResistiveLinearEffortY(void);

	bool isResistiveLinearEffortZEnabled(void) const;
	void enableResistiveLinearEffortZ(void);
	void disableResistiveLinearEffortZ(void);

	bool isResistiveRotationalEffortXEnabled(void) const;
	void enableResistiveRotationalEffortX(void);
	void disableResistiveRotationalEffortX(void);

	bool isResistiveRotationalEffortYEnabled(void) const;
	void enableResistiveRotationalEffortY(void);
	void disableResistiveRotationalEffortY(void);

	bool isResistiveRotationalEffortZEnabled(void) const;
	void enableResistiveRotationalEffortZ(void);
	void disableResistiveRotationalEffortZ(void);


	double getPropulsiveLinearEffortX_percent(void);
	void setPropulsiveLinearEffortX_percent(double value);

	double getPropulsiveLinearEffortY_percent(void);
	void setPropulsiveLinearEffortY_percent(double value);

	double getPropulsiveLinearEffortZ_percent(void);
	void setPropulsiveLinearEffortZ_percent(double value);

	double getPropulsiveRotationalEffortX_percent(void);
	void setPropulsiveRotationalEffortX_percent(double value);

	double getPropulsiveRotationalEffortY_percent(void);
	void setPropulsiveRotationalEffortY_percent(double value);

	double getPropulsiveRotationalEffortZ_percent(void);
	void setPropulsiveRotationalEffortZ_percent(double value);

	double getResistiveLinearEffortX_percent(void);
	void setResistiveLinearEffortX_percent(double value);

	double getResistiveLinearEffortY_percent(void);
	void setResistiveLinearEffortY_percent(double value);

	double getResistiveLinearEffortZ_percent(void);
	void setResistiveLinearEffortZ_percent(double value);

	double getResistiveRotationalEffortX_percent(void);
	void setResistiveRotationalEffortX_percent(double value);

	double getResistiveRotationalEffortY_percent(void);
	void setResistiveRotationalEffortY_percent(double value);

	double getResistiveRotationalEffortZ_percent(void);
	void setResistiveRotationalEffortZ_percent(double value);

protected:
	PropulsiveEffortScaledInteger propulsiveLinearEffortX_percent;
	PropulsiveEffortScaledInteger propulsiveLinearEffortY_percent;
	PropulsiveEffortScaledInteger propulsiveLinearEffortZ_percent;
	PropulsiveEffortScaledInteger propulsiveRotationalEffortX_percent;
	PropulsiveEffortScaledInteger propulsiveRotationalEffortY_percent;
	PropulsiveEffortScaledInteger propulsiveRotationalEffortZ_percent;
	ResistiveEffortScaledInteger resistiveLinearEffortX_percent;
	ResistiveEffortScaledInteger resistiveLinearEffortY_percent;
	ResistiveEffortScaledInteger resistiveLinearEffortZ_percent;
	ResistiveEffortScaledInteger resistiveRotationalEffortX_percent;
	ResistiveEffortScaledInteger resistiveRotationalEffortY_percent;
	ResistiveEffortScaledInteger resistiveRotationalEffortZ_percent;

	uint16_t presenceVector;
	enum pvEnum {PROPULSIVELINEAREFFORTX_PERCENT = 0, PROPULSIVELINEAREFFORTY_PERCENT = 1, PROPULSIVELINEAREFFORTZ_PERCENT = 2, PROPULSIVEROTATIONALEFFORTX_PERCENT = 3, PROPULSIVEROTATIONALEFFORTY_PERCENT = 4, PROPULSIVEROTATIONALEFFORTZ_PERCENT = 5, RESISTIVELINEAREFFORTX_PERCENT = 6, RESISTIVELINEAREFFORTY_PERCENT = 7, RESISTIVELINEAREFFORTZ_PERCENT = 8, RESISTIVEROTATIONALEFFORTX_PERCENT = 9, RESISTIVEROTATIONALEFFORTY_PERCENT = 10, RESISTIVEROTATIONALEFFORTZ_PERCENT = 11};
};

} // namespace mobility
} // namespace openjaus

#endif // WRENCHEFFORTRECORD_H

