test: test.o PigpioMS.o
	g++ -Wall -o test test.o PigpioMS.o -lpigpio -std=c++11 -pthread -lrt
PigpioMS.o: PigpioMS.cpp
	g++ -Wall -c PigpioMS.cpp -lpigpio -std=c++11 -pthread -lrt
test.o: test.cpp
	g++ -Wall -c test.cpp -lpigpio -std=c++11 -pthread -lrt
clean:
	rm -f *.o test
