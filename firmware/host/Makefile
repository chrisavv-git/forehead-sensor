# Host build of the firmware core: unit tests and the simulator (fhsim). make test | make fhsim
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wno-unused-parameter
SAN ?= -fsanitize=address,undefined -fno-omit-frame-pointer
CORE := $(wildcard ../core/*.cpp)
HDRS := $(wildcard ../core/*.h) simbody.h

all: test_core fhsim

test_core: test_core.cpp simbody.cpp $(CORE) $(HDRS)
	$(CXX) $(CXXFLAGS) $(SAN) -o $@ test_core.cpp simbody.cpp $(CORE) -lm

fhsim: fhsim.cpp simbody.cpp $(CORE) $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ fhsim.cpp simbody.cpp $(CORE) -lm -lutil

test: test_core
	./test_core

clean:
	rm -f test_core fhsim

.PHONY: all test clean
