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
#ifndef SYSTEM_APPLICATION_H
#define SYSTEM_APPLICATION_H

#include "openjaus/system/Configuration.h"
#include "openjaus/system/Application.h"
#include "openjaus/system/Setting.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#ifdef WIN32
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#else
	#error "No terminal implementation defined for this platform."
#endif
// End of user code

namespace openjaus
{
namespace system
{
class Configuration;
class Application;
class Setting;

/// \class Application Application.h

class OPENJAUS_EXPORT Application 
{
private:
	Application(); 
	virtual ~Application();
	// Start of user code for additional constructors
	// End of user code

public:

	static Configuration& settings();

	/// Operation processId.
	static int processId();

	/// Operation getName.
	static std::string getName();

	/// Operation getArg.
	/// \param number 
	static std::string getArg(int number);

	/// Operation getArg.
	/// \param argKey 
	static std::string getArg(std::string argKey);

	/// Operation isLogSpaceOn.
	/// \param nameSpace 
	static bool isLogSpaceOn(std::string nameSpace);


	static Application& instance();

	/// Operation setTerminalMode.
	static bool setTerminalMode();

	/// Operation getChar.
	static unsigned char getChar();


	/// \param key 
	static Setting* getSetting(std::string key);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Application& object);

protected:

// Start of user code for additional member data
#ifdef WIN32
	HANDLE handleStdin;
#else
	struct termios termio;
#endif
	bool terminalModified;

public:
	template <class Type>
	inline static Type setting(std::string key, Type defaultValue, bool controllerWritable = false, bool globalReadable = false, bool controllerReadable = true)
	{
		return settings().value< Type >(key, defaultValue, controllerWritable, globalReadable, controllerReadable);
	}

	inline static std::string setting(std::string key, std::string defaultValue, bool controllerWritable = false, bool globalReadable = false, bool controllerReadable = true)
	{
		return settings().value(key, defaultValue, controllerWritable, globalReadable, controllerReadable);
	}

	inline static bool comment(std::string key, std::string comment)
	{
		return settings().comment(key, comment);
	}

// End of user code

}; // class Application

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_APPLICATION_H

