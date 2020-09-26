#declare variables
TARGET = miniCar
CC = gcc
CCP = g++

cppsrc = $(wildcard *.cpp)\
	   $(wildcard src/*.cpp)
csrc = $(wildcard lib/*/*.c)

obj = $(cppsrc:.cpp=.o) $(csrc:.c=.o)

$(TARGET): $(obj)
	$(CCP) -o $@ $^ -l wiringPi	

%.o : %.c
	$(CC) $< -c -o $@

%.o : %.cpp
	$(CCP) $< -c -o $@ -I include/

clean: 
	rm -f $(obj) $(TARGET)

 
