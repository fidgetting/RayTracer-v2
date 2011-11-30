###############################################################################
# Copyright (C) 2011 Alex Norton                                              #
#                                                                             #
# This program is free software; you can redistribute it and/or modify it     #
# under the terms of the BSD 2-Clause License.                                #
#                                                                             #
# This program is distributed in the hope that it will be useful, but         #
# WITHOUT ANY WARRENTY; without even the implied warranty of                  #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                        #
###############################################################################

BIN_DIR = ../bin

CXX     = g++
CFLAGS  = -g -Wall -O3 -std=c++0x
INCPATH = -I.

OBJS = $(BIN_DIR)/vector.o \


HEAD = Makefile               \
       matrix.tpp             \
       vector.h               \

all: $(OBJS)

debug:
	$(MAKE) all DEF=-DDEBUG

$(OBJS) : $(BIN_DIR)/%.o : %.cpp $(HEAD)
	$(CXX) -c $(DEF) $(INCPATH) $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJS)
