#declare variables
CC = gcc
CCP = g++

cppsrc = $(wildcard src/*.cpp)
csrc = $(wildcard lib/*/*.c)

obj = $(cppsrc:.cpp=.o) $(csrc:.c=.o)

.PHONY : all
all: miniCar startUp manualCar manualCar_js musicCar

miniCar: $(obj) miniCar.cpp
	$(CCP) -o $@ $^ -l wiringPi -pthread

startUp: $(obj) startUp.cpp
	$(CCP) -o $@ $^ -l wiringPi

manualCar: $(obj) manualCar.cpp
	$(CCP) -o $@ $^ -l wiringPi

manualCar_js: $(obj) manualCar_js.cpp
	$(CCP) -o $@ $^ -l wiringPi

musicCar: $(obj) musicCar.cpp
	$(CCP) -o $@ $^ -l wiringPi

%.o : %.c
	$(CC) $< -c -o $@

%.o : %.cpp
	$(CCP) $< -c -o $@ -I include/

.PHONY : clean
clean: 
	rm -f $(obj) miniCar startUp


 
