#declare variables
CC = g++
csrc = $(wildcard *.cpp)
obj = $(csrc:.cpp=.o)

all: $(obj)
	$(CC) -o miniCar $^ -l wiringPi	

clean: 
	rm -f $(obj)

