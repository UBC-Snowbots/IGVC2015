/**
\file Configuration.h

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
#ifndef SYSTEM_CONFIGURATION_H
#define SYSTEM_CONFIGURATION_H

#include <map>
#include "openjaus/system/Setting.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "openjaus/system/Logger.h"
#include <iostream>
#include <sstream>
#include <map>
// End of user code

namespace openjaus
{
namespace system
{
class Setting;

/// \class Configuration Configuration.h

class OPENJAUS_EXPORT Configuration 
{
public:
	Configuration(); 
	virtual ~Configuration();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of fileName.
	std::string getFileName() const;

	/// Accessor to set value of fileName.
	/// \param fileName The value of the new fileName.
	bool setFileName(std::string fileName);

	/// Accessor to get the value of settingMap.
	const std::map< std::string, Setting * >& getSettingMap() const;

	/// Accessor to set value of settingMap.
	/// \param settingMap The value of the new settingMap.
	bool setSettingMap(std::map< std::string, Setting * > settingMap);

	/// Operation load.
	/// \param fileName 
	 void load(std::string fileName);

	/// Operation save.
	 void save();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Configuration& object);

protected:
	// Member attributes & references
	std::string fileName;
	std::map< std::string, Setting * > settingMap;

// Start of user code for additional member data
public:
	template <class Type>
	Type value(std::string key, Type defaultValue, bool controllerWritable = false, bool globalReadable = false, bool controllerReadable = true);

	std::string value(std::string key, std::string defaultValue, bool controllerWritable, bool globalReadable, bool controllerReadable);
	bool comment(std::string key, std::string comment);
// End of user code

}; // class Configuration

// Start of user code for inline functions
template <class Type>
inline Type Configuration::value(std::string key, Type defaultValue, bool controllerWritable, bool globalReadable, bool controllerReadable)
{
	Type valueType;

	Setting *setting;
	if(settingMap.count(key))
	{
		setting = settingMap[key];
		std::istringstream iss(setting->getValueStr(), std::istringstream::in);
		iss >> std::boolalpha >> valueType;
		if(iss.fail())
		{
			LOG("Could Not Parse Key: " << key <<
				", from value string: " << setting->getValueStr() <<
				", using default: " << defaultValue);
			valueType = defaultValue;
		}
	}
	else
	{
		setting = new Setting();
		setting->setKey(key);

		std::ostringstream oss;
		oss << std::boolalpha << defaultValue;
		setting->setValueStr(oss.str());

		valueType = defaultValue;
	}

	setting->setGlobalRead(globalReadable);
	setting->setControllerRead(controllerReadable);
	setting->setControllerWrite(controllerWritable);

	settingMap[key] = setting;

	return valueType;
}

inline std::string Configuration::value(std::string key, std::string defaultValue, bool controllerWritable, bool globalReadable, bool controllerReadable)
{
	Setting *setting;
	if(settingMap.count(key))
	{
		setting = settingMap[key];
		return setting->getValueStr();
	}

	LOG_DEBUG("Creating new setting: " << key);
	setting = new Setting();
	setting->setKey(key);
	setting->setValueStr(defaultValue);
	setting->setGlobalRead(globalReadable);
	setting->setControllerRead(controllerReadable);
	setting->setControllerWrite(controllerWritable);
	settingMap[key] = setting;
	return defaultValue;
}

inline bool Configuration::comment(std::string key, std::string comment)
{
	Setting *setting;
	if(settingMap.count(key))
	{
		setting = settingMap[key];
		setting->setComment(comment);
		return true;
	}
	else
	{
		return false;
	}
}

// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_CONFIGURATION_H

