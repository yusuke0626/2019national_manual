Main: PigpioMS.o RasPiDS3.o main.o GY521.o
	g++ -Wall main.cpp -o Main RasPiDS3.o PigpioMS.o GY521.o -lpigpio -std=c++11 -pthread -lrt
GY521.o: ./Sensor-master/GY521/GY521.cpp
	g++ -Wall -c ./Sensor-master/GY521/GY521.cpp -std=c++11 -pthread
PigpioMS.o: ./PigpioMS/PigpioMS.cpp
	g++ -Wall -c ./PigpioMS/PigpioMS.cpp -lpigpio -std=c++11 -pthread
Limit: 
	g++ -Wall limit_test.cpp -o Limit -lpigpio -std=c++11 -pthread -lrt
RasPiDS3.o: ./RasPiDS3/RasPiDS3.cpp
	g++ -Wall -c ./RasPiDS3/RasPiDS3.cpp -std=c++11 -pthread
main.o: main.cpp
	g++ -Wall -c main.cpp -lpigpio -std=c++11 -pthread -lrt
clean: 
	rm -f *.o Do
