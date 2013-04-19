default: all

CFLAGS := -I./include -g --std=gnu99
CXXFLAGS := -I./include -g

CC := gcc
CXX := g++

BINARIES := hubo-read-trajectory
all : $(BINARIES)

LIBS := -lach -lrt -lm -lc

hubo-read-trajectory: src/hubo-read-trajectory.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)


clean:
	rm -f $(BINARIES) src/*.o
