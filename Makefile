all: main

main: main.cpp
	g++ -O0 -g -lfftw3 -lncurses -lportaudio -o main main.cpp -std=c++11
