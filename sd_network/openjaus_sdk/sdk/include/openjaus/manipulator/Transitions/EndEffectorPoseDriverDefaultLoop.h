/**
\file EndEffectorPoseDriverDefaultLoop.h

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

#ifndef ENDEFFECTORPOSEDRIVERDEFAULTLOOP_H
#define ENDEFFECTORPOSEDRIVERDEFAULTLOOP_H

#include <openjaus.h>
#include "openjaus/core/TransportInterface.h"
#include "openjaus/manipulator/ManipulatorEndEffectorPoseDriverInterface.h"

namespace openjaus
{
namespace manipulator
{

class OPENJAUS_EXPORT EndEffectorPoseDriverDefaultLoop : public model::Transition
{
public:
	EndEffectorPoseDriverDefaultLoop();
	virtual ~EndEffectorPoseDriverDefaultLoop();
	
	bool processTrigger(model::Trigger* trigger);
	model::Message* getResponse(model::Trigger* trigger);
	bool setInterface(ManipulatorEndEffectorPoseDriverInterface *cmptInterface);
	bool setTransportInterface(core::TransportInterface *cmptInterface);
	
private:
	ManipulatorEndEffectorPoseDriverInterface *cmptInterface;
	core::TransportInterface *transport;

};

} // namespace manipulator
} // namespace openjaus

#endif // ENDEFFECTORPOSEDRIVERDEFAULTLOOP_H
