build: main.cc micro_rrt.h matplotlibcpp.h
	g++ main.cc -o main -std=c++11 -I/usr/include/python2.7 -lpython2.7

run: build
	./main

all: build run

