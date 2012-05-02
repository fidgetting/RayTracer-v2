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
LINK    = g++
CFLAGS  = -g -O3 -Wall -std=c++0x `pkg-config opencv --cflags`
INCPATH = -I. -I../obj -I../util
EXE     = ../model
LIBS    = `pkg-config opencv --libs` -L/usr/lib -lpthread \
          -lboost_program_options -lboost_filesystem -lboost_system

OBJS = $(BIN_DIR)/model.o     \
       $(BIN_DIR)/surface.o   \
       $(BIN_DIR)/ray.o       \
       $(BIN_DIR)/camera.o    \
       $(BIN_DIR)/main.o      \

HEAD = Makefile               \
       surface.h              \
       model.h                \
       ray.h                  \
       camera.h               \
       ../obj/objstream.hpp   \
       ../util/vector.h       \
       ../util/matrix.tpp     \
       ../util/threadpool.hpp \

all: $(EXE)
$(EXE): $(OBJS) $(BIN_DIR)/*.o $(HEAD)
	$(LINK) -o $(EXE) $(BIN_DIR)/*.o $(INCPATH) $(CFLAGS) $(LIBS)

debug:
	$(MAKE) all DEF=-DDEBUG

$(OBJS) : $(BIN_DIR)/%.o : %.cpp $(HEAD)
	$(CXX) -c $(DEF) $(INCPATH) $(CFLAGS) $< -o $@

clean:
	rm -f $(EXE) $(OBJS)
