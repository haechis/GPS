gnss.o: gnss.h gnss.cpp
	g++ -c gnss.cpp

INC = /eigen-3.4.0

main.o: gnss.h main.cpp
	g++ -c main.cpp

try: gnss.o main.o
	g++ -o -I$(INC) try main.o gnss.o 

all: try

clear:
	rm -f gnss.o main.o try

	