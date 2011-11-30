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

YACC    = bison
LEX     = flex
LINK    = g++
CXX     = g++
CFLAGS  = -Wall -O3 -Werror -DYY_NO_INPUT -g -std=c++0x
INCPATH = -I. -I../util

OBJS = $(BIN_DIR)/parser.o     \
       $(BIN_DIR)/lexer.o      \
       $(BIN_DIR)/objstream.o  \

HEAD = objstream.hpp \
       Makefile

all: $(OBJS)

debug:
	$(MAKE) all DEF=-DDEBUG

parser.cpp: parser.y $(HEAD)
	$(YACC) --defines=parser.hpp $< -o $@

lexer.cpp: lexer.lex parser.cpp $(HEAD)
	$(LEX) --nounput -o $@ $<

$(OBJS): $(BIN_DIR)/%.o: %.cpp $(HEAD)
	$(CXX) -c $(INCPATH) $(CFLAGS) $(DEF) $< -o $@

clean:
	rm -f $(LIB)
	rm -f parser.cpp parser.hpp lexer.cpp
	rm -f $(OBJS)
