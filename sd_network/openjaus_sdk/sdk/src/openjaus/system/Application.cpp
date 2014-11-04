/**
\file Application.h

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

#include "openjaus/system/Application.h"
#include <sstream>
// Start of user code for additional includes
#ifdef WIN32
#else
#include <unistd.h>
#include <string.h>
#endif

#include <iostream>
#include <fstream>
#include "openjaus/system/Exception.h"
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Application::Application() :
		terminalModified(false)
{
#ifdef WIN32
	handleStdin = NULL;
#endif
}
// End of user code

// Start of user code for default destructor:
Application::~Application()
{
	if(terminalModified)
	{
#ifdef WIN32
#else
		tcsetattr(0, TCSANOW, &termio);
#endif
	}
}
// End of user code


// Class Methods
Configuration& Application::settings()
{
	// Start of user code for method settings:
	static Configuration appConfig;
	static bool loaded = false;

	if(!loaded)
	{
		loaded = true;
		appConfig.load(getName().append(".conf"));
	}

	return appConfig;
	// End of user code
}


int Application::processId()
{
	// Start of user code for method processId:
	int result = 0;
#ifdef WIN32
#else
	result = getpid();
#endif
	return result;
	// End of user code
}


std::string Application::getName()
{
	// Start of user code for method getName:
	std::string arg0 = getArg(0);
	if(arg0.at(0) == '"')
	{
		arg0 = arg0.substr(1);
	}

	std::string binaryName = arg0;
	size_t pos = arg0.find_last_of("/\\");
	if(pos != arg0.npos)
	{
		binaryName = arg0.substr(pos+1);
	}

	pos = binaryName.find_first_of("\".");
	if(pos == binaryName.npos)
	{
		return binaryName;
	}
	return binaryName.substr(0, pos);
	// End of user code
}


std::string Application::getArg(int number)
{
	// Start of user code for method getArg(int number):
	std::string result;
	static std::vector<std::string> args;
	std::string currentArg;

	// If args not loaded yet
	if(args.empty())
	{
#ifdef WIN32
		std::istringstream commandLine(GetCommandLine());
		while(!commandLine.eof()) // While there are remaining characters
		{
			commandLine >> currentArg;
			args.push_back(currentArg);
		}
#else
		// Determine command line file to parse
		std::ostringstream fileName;
		fileName << "/proc/" << processId() << "/cmdline";

		LOG_DEBUG("Application: Attempting to read cmdline: " << fileName.str());

		// Open command line file for reading
		std::ifstream file(fileName.str().c_str());
		if(!file.is_open())
		{
			THROW_EXCEPTION("Application Command Line File: " << fileName.str() << " not found.");
		}

		// Read command line one character at a time
		char c;
		file.get(c);
		// While there are remaining characters
		while(!file.eof())
		{
			// If this is a null character save the currentArg
			if(c == '\0')
			{
				LOG_DEBUG("Application: Read arg: " << currentArg);

				args.push_back(currentArg);

				// Start a new blank currentArg
				currentArg = std::string();
			}
			else
			{
				currentArg.append(1, c);
			}

			// Get next character
			file.get(c);
		}
#endif
	}

	if(static_cast<unsigned int>(number) < args.size())
	{
		result = args[number];
	}

	return result;
	// End of user code
}


std::string Application::getArg(std::string argKey)
{
	// Start of user code for method getArg(std::string argKey):
	static bool argumentsProcessed = false;
	static std::map<std::string, std::string> argMap;
	static const std::string qualifier("--oj");

	if(!argumentsProcessed)
	{
		argumentsProcessed = true;

		argMap.clear();

		int i = 1;
		std::string arg = getArg(i);
		while(!arg.empty())
		{
			if(arg.find(qualifier))
			{
				arg = getArg(++i);
				continue;
			}

			// Attempt to find end of key at equals sign
			size_t endKeyPose = arg.find("=", 4);

			std::string foundKey;
			std::string value;

			if(endKeyPose == std::string::npos)
			{
				foundKey = arg.substr(4);
				value = "1";
			}
			else
			{
				foundKey = arg.substr(4, endKeyPose-4);
				value = arg.substr(endKeyPose+1);
			}

			if(!foundKey.empty())
			{
				argMap.insert(std::pair<std::string, std::string>(foundKey,value));
			}

			arg = getArg(++i);
		}
	}


	if(argMap.count(argKey))
	{
		return argMap[argKey];
	}

	return std::string();
	// End of user code
}


bool Application::isLogSpaceOn(std::string nameSpace)
{
	// Start of user code for method isLogSpaceOn:
	std::string verbose = getArg("verbose");
	if(!verbose.empty())
	{
		return true;
	}

	std::string value = getArg(nameSpace);
	if(!value.empty())
	{
		return true;
	}

	return false;
	// End of user code
}


Application& Application::instance()
{
	// Start of user code for method instance:
	static Application inst;
	return inst;
	// End of user code
}


bool Application::setTerminalMode()
{
	// Start of user code for method setTerminalMode:
	// TODO: Add enumeration of different terminal modes
#ifdef WIN32
	// Setup the console window's input handle
	instance().handleStdin = GetStdHandle(STD_INPUT_HANDLE);
	if(instance().handleStdin == INVALID_HANDLE_VALUE)
	{
		return false;
	}
#else
	struct termios newTermio;
	tcgetattr(0,&instance().termio);
	memcpy(&newTermio,&instance().termio,sizeof(struct termios));

	// Disable canonical mode, and set buffer size to 0 byte(s)
	newTermio.c_lflag &= (~ICANON);
	newTermio.c_lflag &= (~ECHO);
	newTermio.c_cc[VTIME] = 0;
	newTermio.c_cc[VMIN] = 1;
	if(tcsetattr(0,TCSANOW,&newTermio))
	{
		return false;
	}

	instance().terminalModified = true;
#endif
	return true;
	// End of user code
}


unsigned char Application::getChar()
{
	// Start of user code for method getChar:
#ifdef WIN32
	if(!instance().handleStdin)
	{
		return 0;
	}
	
	// Console parameters
	INPUT_RECORD inputEvents[128];
	DWORD count = 0;

	// Check for user input here
	ReadConsoleInput(	instance().handleStdin,	// input buffer handle
						inputEvents,			// buffer to read into
						128,					// size of read buffer
						&count);				// number of records read

    // Parse console input events
	for(unsigned int i = 0; i < count; i++)
	{
		switch(inputEvents[i].EventType)
		{
			case KEY_EVENT: // keyboard input
				if(inputEvents[i].Event.KeyEvent.bKeyDown)
				{
					return inputEvents[i].Event.KeyEvent.uChar.AsciiChar;
				}
				break;
			default:
				break;
		}
	}
	return 0;
#else
	int count = 0;
	unsigned char choice[8] = {0};
	memset(choice, 0, 8);
	count = read(0, &choice, 8);
	return count == 1? choice[0] : 0;
#endif
	// End of user code
}


Setting* Application::getSetting(std::string key)
{
	// Start of user code for method getSetting:
	std::map< std::string, Setting * >& settingMap = const_cast<std::map< std::string, Setting * >& >(settings().getSettingMap());
	if(!settingMap.count(key))
	{
		return NULL;
	}
	return settingMap[key];
	// End of user code
}




std::string Application::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Application& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace system
} // namespace openjaus

