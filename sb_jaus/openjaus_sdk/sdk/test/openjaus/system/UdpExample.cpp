#include <openjaus.h>


int main(void)
{
	namespace ojsys = openjaus::system;
	ojsys::Application::setTerminalMode();
	ojsys::DatagramSocket udpSocket(ojsys::InetAddress::anyAddress(), ojsys::InetAddress::ANY_PORT);

	float x = 5.0f;
	int y = 4;
	ojsys::Packet data;
	data.pack(x);
	data.pack(y);
	data.setAddress(ojsys::InetAddress::getByName("192.168.0.2"));
	data.setPort(3794);
	udpSocket.send(data);

	udpSocket.receive(data);
	data.unpack(x);
	data.unpack(y);

	try
	{
		GposComponent* component = new GposComponent();
		component->run();

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
