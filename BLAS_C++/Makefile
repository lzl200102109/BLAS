CC       = g++
CFLAGS   = -Wall
LIB = -lm -llapack -lcblas -lblas
LPINC  = -I/usr/include/



all:	ex1 ex2

ex1 : ex1.cpp
	$(CC) $(CFLAGS) ex1.cpp $(LIB) -o ex1

ex2 : ex2.cpp
	$(CC) $(CFLAGS) ex2.cpp $(LIB) -o ex2

	
clean:
	rm -f ex1 ex2
