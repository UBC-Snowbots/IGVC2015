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

#include "openjaus/model/Service.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Service::Service() :
	name(),
	uri(),
	description(),
	versionMajor(),
	versionMinor(),
	inheritsFrom(NULL),
	triggers(),
	stateMachines(),
	actions(),
	conditions()
{
}

Service::Service(const Service& copy) :
			// TODO: Find out why this errors: name(copy.name),
			//description(copy.description),
			uri(copy.uri),
			versionMajor(copy.versionMajor),
			versionMinor(copy.versionMinor),
			inheritsFrom(NULL),
			triggers(),
			stateMachines(),
			actions(),
			conditions()
{
}
// End of user code

// Start of user code for default destructor:
Service::~Service()
{
}
// End of user code

std::string Service::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool Service::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


std::string Service::getUri() const
{
	// Start of user code for accessor getUri:
	
	return uri;
	// End of user code
}

bool Service::setUri(std::string uri)
{
	// Start of user code for accessor setUri:
	this->uri = uri;
	return true;
	// End of user code
}


std::string Service::getDescription() const
{
	// Start of user code for accessor getDescription:
	
	return description;
	// End of user code
}

bool Service::setDescription(std::string description)
{
	// Start of user code for accessor setDescription:
	this->description = description;
	return true;
	// End of user code
}


int Service::getVersionMajor() const
{
	// Start of user code for accessor getVersionMajor:
	
	return versionMajor;
	// End of user code
}

bool Service::setVersionMajor(int versionMajor)
{
	// Start of user code for accessor setVersionMajor:
	this->versionMajor = versionMajor;
	return true;
	// End of user code
}


int Service::getVersionMinor() const
{
	// Start of user code for accessor getVersionMinor:
	
	return versionMinor;
	// End of user code
}

bool Service::setVersionMinor(int versionMinor)
{
	// Start of user code for accessor setVersionMinor:
	this->versionMinor = versionMinor;
	return true;
	// End of user code
}


Service* Service::getInheritsFrom() const
{
	// Start of user code for accessor getInheritsFrom:
	
	return inheritsFrom;
	// End of user code
}

bool Service::setInheritsFrom(Service* inheritsFrom)
{
	// Start of user code for accessor setInheritsFrom:
	this->inheritsFrom = inheritsFrom;
	return true;
	// End of user code
}


const std::vector< Trigger* >& Service::getTriggers() const
{
	// Start of user code for accessor getTriggers:
	
	return triggers;
	// End of user code
}

bool Service::setTriggers(const Trigger& triggers)
{
	// Start of user code for accessor setTriggers:
	return true;
	// End of user code
}


const std::vector< StateMachine* >& Service::getStateMachines() const
{
	// Start of user code for accessor getStateMachines:
	
	return stateMachines;
	// End of user code
}

bool Service::setStateMachines(const StateMachine& stateMachines)
{
	// Start of user code for accessor setStateMachines:
	return true;
	// End of user code
}


const std::vector< Action* >& Service::getActions() const
{
	// Start of user code for accessor getActions:
	
	return actions;
	// End of user code
}

bool Service::setActions(const Action& actions)
{
	// Start of user code for accessor setActions:
	return true;
	// End of user code
}


const std::vector< Condition* >& Service::getConditions() const
{
	// Start of user code for accessor getConditions:
	
	return conditions;
	// End of user code
}

bool Service::setConditions(const Condition& conditions)
{
	// Start of user code for accessor setConditions:
	return true;
	// End of user code
}


const std::vector< Transition* >& Service::getTransitions() const
{
	// Start of user code for accessor getTransitions:
	
	return transitions;
	// End of user code
}

bool Service::setTransitions(const Transition& transitions)
{
	// Start of user code for accessor setTransitions:
	return true;
	// End of user code
}


const std::vector< State* >& Service::getStates() const
{
	// Start of user code for accessor getStates:
	
	return states;
	// End of user code
}

bool Service::setStates(const State& states)
{
	// Start of user code for accessor setStates:
	return true;
	// End of user code
}



// Class Methods
StateMachine Service::getStateMachine(std::string name)
{
	// Start of user code for method getStateMachine:
	StateMachine result;

	return result;
	// End of user code
}




std::string Service::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "Service: "<< this->uri << " (" << this->versionMajor << "." << this->versionMinor << ")\n";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Service& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
Json::Value Service::toJson() const
{
	Json::Value root;
	root["URI"] = this->uri;
	root["versionMajor"] = this->versionMajor;
	root["versionMinor"] = this->versionMinor;
	return root;
}

std::string Service::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<service";
	oss << " uri=\"" << this->uri << "\"";
	oss << " versionMajor=\"" << this->versionMajor << "\"";
	oss << " versionMinor=\"" << this->versionMinor << "\"";
	oss << " />\n";
	return oss.str();
}

// End of user code

} // namespace model
} // namespace openjaus

