/**
\file Service.h

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
#ifndef MODEL_SERVICE_H
#define MODEL_SERVICE_H

#include "openjaus/model/StateMachine.h"
#include "openjaus/model/Service.h"
#include "openjaus/model/Trigger.h"
#include "openjaus/model/Action.h"
#include "openjaus/model/Condition.h"
#include "openjaus/model/Transition.h"
#include "openjaus/model/State.h"
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "json/json.h"
// End of user code

namespace openjaus
{
namespace model
{
class StateMachine;
class Service;
class Trigger;
class Action;
class Condition;
class Transition;
class State;

/// \class Service Service.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Service 
{
public:
	Service(); 
	virtual ~Service();
	// Start of user code for additional constructors
	Service(const Service& copy);
	// End of user code
	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of uri.
	std::string getUri() const;

	/// Accessor to set value of uri.
	/// \param uri The value of the new uri.
	bool setUri(std::string uri);

	/// Accessor to get the value of description.
	std::string getDescription() const;

	/// Accessor to set value of description.
	/// \param description The value of the new description.
	bool setDescription(std::string description);

	/// Accessor to get the value of versionMajor.
	int getVersionMajor() const;

	/// Accessor to set value of versionMajor.
	/// \param versionMajor The value of the new versionMajor.
	bool setVersionMajor(int versionMajor);

	/// Accessor to get the value of versionMinor.
	int getVersionMinor() const;

	/// Accessor to set value of versionMinor.
	/// \param versionMinor The value of the new versionMinor.
	bool setVersionMinor(int versionMinor);

	/// Accessor to get the value of inheritsFrom.
	Service* getInheritsFrom() const;

	/// Accessor to set value of inheritsFrom.
	/// \param inheritsFrom The value of the new inheritsFrom.
	bool setInheritsFrom(Service* inheritsFrom);

	/// Accessor to get the value of triggers.
	const std::vector< Trigger* >& getTriggers() const;

	/// Accessor to set value of triggers.
	/// \param triggers The value of the new triggers.
	bool setTriggers(const Trigger& triggers);

	/// Accessor to get the value of stateMachines.
	const std::vector< StateMachine* >& getStateMachines() const;

	/// Accessor to set value of stateMachines.
	/// \param stateMachines The value of the new stateMachines.
	bool setStateMachines(const StateMachine& stateMachines);

	/// Accessor to get the value of actions.
	const std::vector< Action* >& getActions() const;

	/// Accessor to set value of actions.
	/// \param actions The value of the new actions.
	bool setActions(const Action& actions);

	/// Accessor to get the value of conditions.
	const std::vector< Condition* >& getConditions() const;

	/// Accessor to set value of conditions.
	/// \param conditions The value of the new conditions.
	bool setConditions(const Condition& conditions);

	/// Accessor to get the value of transitions.
	const std::vector< Transition* >& getTransitions() const;

	/// Accessor to set value of transitions.
	/// \param transitions The value of the new transitions.
	bool setTransitions(const Transition& transitions);

	/// Accessor to get the value of states.
	const std::vector< State* >& getStates() const;

	/// Accessor to set value of states.
	/// \param states The value of the new states.
	bool setStates(const State& states);

	/// Operation getStateMachine.
	/// \param name 
	 StateMachine getStateMachine(std::string name);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Service& object);

protected:
	// Member attributes & references
	std::string name;
	std::string uri;
	std::string description;
	int versionMajor;
	int versionMinor;
	Service *inheritsFrom;
	std::vector< Trigger* > triggers;
	std::vector< StateMachine* > stateMachines;
	std::vector< Action* > actions;
	std::vector< Condition* > conditions;
	std::vector< Transition* > transitions;
	std::vector< State* > states;

// Start of user code for additional member data
public:
	static const int ANY_VERSION = -1;

	Json::Value toJson() const;
	std::string toXml(unsigned char level=0) const;
// End of user code

}; // class Service

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_SERVICE_H

