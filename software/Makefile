
CC = g++
CFLAGS  = -g -Wall -std=c++11
INCLUDES = -I include/

default: all

%.o: example/%.cpp
	mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -o build/$@ -c $^ -lwiringPi  -lncurses

%.o: src/%.cpp
	mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -o build/$@ -c $^ -lwiringPi -lncurses

%.o: tests/%.cpp
	mkdir -p build
	pkg-config --atleast-version=3.1 catch2
	$(CC) $(CFLAGS) $(INCLUDES) $(shell pkg-config --cflags catch2) -std=c++17 -o build/$@ -c $^ -lwiringPi -lncurses

app: example.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/example $(addprefix build/,$^) -pthread -lwiringPi -lncurses

app_pd: example_pd.o ESPNOW_manager.o ESPNOW_types.o Link_manager.o ETHERNET_types.o master_board_interface.o motor.o motor_driver.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/example_pd $(addprefix build/,$^) -pthread -lwiringPi -lncurses

sdk_tests: test_protocol.o
	mkdir -p bin
	$(CC) $(CFLAGS) -o bin/sdk_tests $(addprefix build/,$^) $(shell pkg-config --libs catch2) -lCatch2Main -pthread -lwiringPi -lncurses

test : sdk_tests
	bin/sdk_tests

all: clear clean app app_pd

clean: 
	#$(RM) build/*
	$(RM) bin/*

clear:
	clear
