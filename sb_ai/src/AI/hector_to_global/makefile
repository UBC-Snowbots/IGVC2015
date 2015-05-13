all: hector_to_global

hector_to_global: test.o  hector_to_global.o
	g++  -Wall -std=c++11 hector_to_global.cpp test.o -o $@

test.o: test.cpp
	g++  -Wall -std=c++11 -c test.cpp

hector_to_global.o: hector_to_global.cpp
	g++  -Wall -std=c++11 -c hector_to_global.cpp

clean:
	rm *o hector_to_global
