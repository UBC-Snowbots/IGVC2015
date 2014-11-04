local function addConsoleApp(mainFile)
	project(path.getbasename(mainFile))
		kind "ConsoleApp"
		location ".build"
		targetdir "bin"
		links { "openjaus-mobility", "openjaus-environment", "openjaus-core", "openjaus", "json" }
		files { mainFile }
end

local function addServiceSet(ssName)
	project("openjaus-" .. ssName)
		kind "SharedLib"
		location ".build"
		targetdir "lib"
		links { "json", "openjaus", "openjaus-core"}
		files { "src/openjaus/" .. ssName .. "/**.cpp", "include/openjaus/**.h" }
		configuration "windows"
			postbuildcommands { "copy ..\\lib\\openjaus-" .. ssName .. ".dll ..\\bin" }
end

solution "OpenJAUSv4.0"
	configurations { "Debug", "Release" }
	platforms { "Native" }
	language "C++"
	objdir ".build"
	os.mkdir("bin")
	includedirs { "include" }

	configuration "Debug"
		defines { "DEBUG" }
		flags { "Symbols", "ExtraWarnings" }
 
	configuration "Release"
		defines { "NDEBUG" }
		flags { "Optimize" }
	
	configuration "not windows"
		links { "pthread", "rt" }

	configuration "windows"
		local ver = os.getversion()
		if(os.getversion().majorversion < 6) then
			defines { "_WIN32_WINNT=0x0501", "WIN32", "_CRT_SECURE_NO_WARNINGS"  }
		else
			defines { "_WIN32_WINNT=0x0600", "WIN32", "_CRT_SECURE_NO_WARNINGS"  }
		end
		flags { "NoMinimalRebuild" }
		buildoptions { "/wd4100", "/wd4800", "/wd4244", "/wd4251", "/wd4127", "/MP" }
		
	project "json"
		kind "SharedLib"
		location ".build"
		targetdir "lib"
		files { "src/json/**.cpp" }
		configuration "windows"
			postbuildcommands { "copy ..\\lib\\json.dll ..\\bin" }

	project "openjaus"
		kind "SharedLib"
		location ".build"
		targetdir "lib"
		links { "json" }
		files { "src/openjaus/model/**.cpp", "src/openjaus/system/**.cpp", "src/openjaus/transport/**.cpp", "include/openjaus/**.h" }
		configuration "windows"
			postbuildcommands { "copy ..\\lib\\openjaus.dll ..\\bin" }

	project "openjaus-core"
		kind "SharedLib"
		location ".build"
		targetdir "lib"
		links { "json", "openjaus" }
		files { "src/openjaus/core/**.cpp", "include/openjaus/**.h" }
		configuration "windows"
			postbuildcommands { "copy ..\\lib\\openjaus-core.dll ..\\bin" }

	addServiceSet("environment")
	addServiceSet("mobility")
	addServiceSet("manipulator")
	addServiceSet("ugv")
	
	addConsoleApp("test/openjaus/core/Base.cpp")
	addConsoleApp("test/openjaus/core/Managed.cpp")
	addConsoleApp("test/openjaus/core/PingTest.cpp")
	addConsoleApp("test/openjaus/core/LargeMessageTest.cpp")
	addConsoleApp("test/openjaus/mobility/PdDemo.cpp")
	addConsoleApp("test/openjaus/mobility/GposDemo.cpp")
	addConsoleApp("test/openjaus/mobility/GposClientDemo.cpp")
	addConsoleApp("test/openjaus/environment/StillImageSensorDemo.cpp")
	addConsoleApp("test/openjaus/environment/StillImageClientDemo.cpp")
