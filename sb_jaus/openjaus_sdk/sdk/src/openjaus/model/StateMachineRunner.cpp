/**
\file StateMachineRunner.h

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

#include "openjaus/model/StateMachineRunner.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/transport/Wrapper.h"
#include "openjaus/model/Message.h"
#include "openjaus/model/InternalEvent.h"
#include "openjaus/system/Exception.h"
#include "openjaus/system/Buffer.h"
#include "openjaus/system/Logger.h"

#include <iostream>
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
StateMachineRunner::StateMachineRunner() :
		Thread(),
		name(),
		queue(),
		stateMachine(NULL)
{
}
// End of user code

// Start of user code for default destructor:
StateMachineRunner::~StateMachineRunner()
{
	transport::Wrapper* wrapper = dynamic_cast<transport::Wrapper *>(queue.pop());
	while(wrapper != NULL)
	{
		delete wrapper;
		wrapper = dynamic_cast<transport::Wrapper *>(queue.pop());
	}

}
// End of user code

std::string StateMachineRunner::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool StateMachineRunner::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


const system::PriorityQueue& StateMachineRunner::getQueue() const
{
	// Start of user code for accessor getQueue:
	return queue;
	// End of user code
}


StateMachine* StateMachineRunner::getStateMachine() const
{
	// Start of user code for accessor getStateMachine:
	return stateMachine;
	// End of user code
}

bool StateMachineRunner::setStateMachine(StateMachine* stateMachine)
{
	// Start of user code for accessor setStateMachine:
	this->stateMachine = stateMachine;
	return true;
	// End of user code
}



// Class Methods
bool StateMachineRunner::push(Trigger *trigger)
{
	// Start of user code for method push:
	return queue.push(dynamic_cast<system::Prioritized *>(trigger));
	// End of user code
}


void StateMachineRunner::stop()
{
	// Start of user code for method stop:
	running = false;
	queue.signalAll();
	// End of user code
}




std::string StateMachineRunner::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const StateMachineRunner& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
void* StateMachineRunner::run()
{
	stateMachine->entry();

	while(running)
	{
		system::Prioritized* input = queue.pop();
		transport::Wrapper* wrapper = dynamic_cast<transport::Wrapper *>(input);
		InternalEvent* event = dynamic_cast<InternalEvent *>(input);

		if(wrapper)
		{
			try
			{
				Message message;
				message.setType(wrapper->getType());
				message.setSource(wrapper->getSource());
				message.setDestination(wrapper->getDestination());
				message.setPayload(wrapper->getPayload());
				message.setSequenceNumber(wrapper->getSequenceNumber());
				dynamic_cast<system::Buffer *>(wrapper->getPayload())->reset();
				message.from(dynamic_cast<system::Buffer *>(wrapper->getPayload()));
				message.setTransportData(wrapper->getTransportData());

				stateMachine->processTrigger(&message);

				if(wrapper->getTransportData())
				{
					delete wrapper->getTransportData();
				}
				delete wrapper;
			}
			catch(system::Exception& e)
			{
				system::Logger::log(e);
			}
		}
		else if(event)
		{
			try
			{
				stateMachine->processTrigger(event);
				delete event;
			}
			catch(system::Exception& e)
			{
				system::Logger::log(e);
			}
		}
		else
		{
			queue.timedWait(500);
		}
	}

	stateMachine->exit();
	return NULL;
}
// End of user code

} // namespace model
} // namespace openjaus

