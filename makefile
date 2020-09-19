#declare variables
CC = g++
cppsrc = $(wildcard *.cpp)\
	   $(wildcard src/*.cpp)

csrc = $(wildcard lib/*/*.c)

obj = $(cppsrc:.cpp=.o)\
	  $(csrc:.c=.o)

all: $(obj)
	$(CC) -o miniCar $^ -l wiringPi	

clean: 
	rm -f $(obj)
	rm -f miniCar

 
