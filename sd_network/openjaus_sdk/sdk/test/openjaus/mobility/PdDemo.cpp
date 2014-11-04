#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/core/Base.h"
#include "openjaus/mobility.h"
#include "openjaus/mobility/PrimitiveDriver.h"

#if defined(WIN32)

#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif

using namespace openjaus;

class PdComponent : public openjaus::mobility::PrimitiveDriver
{
public:
	PdComponent() :
		PrimitiveDriver()
	{
		name = "PdDemo";
	}

	bool setWrenchEffort(openjaus::mobility::SetWrenchEffort *setWrenchEffort)
	{
		// Start of user code for action setWrenchEffort(SetWrenchEffort *setWrenchEffort):
		std::cout << "Got wrench" << std::endl;
		return true;
		// End of user code
	}

	bool isControllingPdClient(openjaus::mobility::SetWrenchEffort *setWrenchEffort)
	{
		// Start of user code for action isControllingPdClient(SetWrenchEffort *setWrenchEffort):
		return (setWrenchEffort->getSource() == this->controllerAddress);
		// End of user code
	}
};

int main(void)
{
	openjaus::system::Application::setTerminalMode();

	try
	{
		PdComponent* component = new PdComponent();
		component->run();
		component->initialized();

		std::cout << "Menu:\n";
		std::cout << "t - Print System Tree\n";
		std::cout << "ESC - Exit Component\n";

		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					std::cout << component->getSystemTree()->toString();
					break;
			}
		}

		delete component;
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}

	return 0;
	// End of user code
}
