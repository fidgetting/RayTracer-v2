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
CFLAGS  = -g -Wall -O3 -std=c++0x `pkg-config opencv --cflags`
INCPATH = -I. -I../obj -I../util
EXE     = ../model
LIBS    = `pkg-config opencv --libs` -L/usr/lib -lQtGui -lQtCore -lpthread

OBJS = $(BIN_DIR)/transform.o \
       $(BIN_DIR)/model.o     \
       $(BIN_DIR)/surface.o   \
       $(BIN_DIR)/ray.o       \
       $(BIN_DIR)/camera.o    \

HEAD = Makefile               \
       transform.h            \
       surface.h              \
       model.h                \
       ray.h                  \
       camera.h               \
       queue.tpp              \
       ../obj/objstream.hpp   \
       ../util/vector.h       \
       ../util/matrix.tpp     \

all: $(EXE)
$(EXE): $(OBJS) $(BIN_DIR)/*.o $(HEAD)
	$(CXX) -o $(EXE) $(BIN_DIR)/*.o $(INCPATH) $(CFLAGS) $(LIBS)

debug: clean
	$(MAKE) all DEF=-DDEBUG

$(OBJS) : $(BIN_DIR)/%.o : %.cpp $(HEAD)
	$(CXX) -c $(DEF) $(INCPATH) $(CFLAGS) $< -o $@

clean:
	rm -f $(EXE) $(OBJS)
