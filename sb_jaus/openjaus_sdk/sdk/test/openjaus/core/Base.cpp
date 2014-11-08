/**
\file Base.cpp

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
#include "openjaus/core.h"
// Start of user code for additional includes:
#include "openjaus/core/Base.h"
#if defined(WIN32)
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif
// End of user code


int main(void)
{
	// Start of user code for main:
	openjaus::system::Application::setTerminalMode();
	try
	{
		openjaus::core::Base* component = new openjaus::core::Base();
		component->run();

		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					LOG(component->getSystemTree()->toString());
					break;

				case 'm':
					std::cout << openjaus::transport::AddressMap::instance().toString();
					break;

				case 'c':
					component->requestControl(openjaus::transport::Address(1, 1, 2));
					break;

				case 'r':
					component->releaseControl(openjaus::transport::Address(1, 1, 2));
					break;

				case 'a':
					LOG(openjaus::transport::AddressMap::instance().toString());
					break;

				case 's':
				{
					openjaus::core::QueryHeartbeatPulse *qhb = new openjaus::core::QueryHeartbeatPulse();
					qhb->setDestination(openjaus::transport::Address(1,1,2));
					qhb->setMustArrive(true);
					component->sendMessage(qhb);
					break;
				}
			}
		}
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}

	return 0;
	// End of user code
}


