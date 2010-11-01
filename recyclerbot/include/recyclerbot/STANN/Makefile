
CFLAGS = -O3 -fPIC
RANLIB = true
LIBDIR	= ../
FLAGS = -O3 -Wall -pedantic -ansi -Wextra -fopenmp -Wconversion

stann:
	g++ -O3 $(CFLAGS) -shared -o libSTANN.so STANN.cpp -I./
#$(LIBDIR)/$(STANNLIB): STANN.cpp
#$(MAKELIB) $(STANNLIB) STANN.cpp
#$(RANLIB) $(STANNLIB)
	mv libSTANN.so ../../../lib
