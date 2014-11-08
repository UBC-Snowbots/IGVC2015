// File Header Here

#include <openjaus.h>
#include "openjaus/core.h"
// Start of user code for additional includes:
#if defined(WIN32)
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif
// End of user code


namespace openjaus
{
namespace core
{

class ConfigurationService : public Configuration
{
public:
	ConfigurationService()
	{
	};
	
	// Start of user code for Constructor:
	system::Timer timer;
	// End of user code
};

} // namespace core
} // namespace openjaus

int main(void)
{
	// Start of user code for main:
	struct termios newTermio;
	struct termios storedTermio;

	tcgetattr(0,&storedTermio);
	memcpy(&newTermio,&storedTermio,sizeof(struct termios));

	// Disable canonical mode, and set buffer size to 0 byte(s)
	newTermio.c_lflag &= (~ICANON);
	newTermio.c_lflag &= (~ECHO);
	newTermio.c_cc[VTIME] = 0;
	newTermio.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&newTermio);

	try
	{
		openjaus::core::ConfigurationService *service = new openjaus::core::ConfigurationService();

		bool running = true;
		while(running)
		{
			char choice[8] = {0};

			memset(choice, 0, 8);
			int count = read(0, &choice, 8);
			if(count == 1 && choice[0] == 27) // ESC
			{
				running = false;
				break;
			}
			else if(count == 1 && choice[0] == 't')
			{
				//LOG(service->getSystemTree()->toString());
			}

			sleep(1);
		}
		delete service;
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}

	tcsetattr(0, TCSANOW, &storedTermio);

	return 0;	
	// End of user code
}


