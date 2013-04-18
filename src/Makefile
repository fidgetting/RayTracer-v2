###############################################################################
# Copyright (C) 2013 Alex Norton                                              #
#                                                                             #
# This program is free software; you can redistribute it and/or modify it     #
# under the terms of the BSD 2-Clause License.                                #
#                                                                             #
# This program is distributed in the hope that it will be useful, but         #
# WITHOUT ANY WARRENTY; without even the implied warranty of                  #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                        #
###############################################################################

BIN_DIR = ../bin

YACC = bison
LEX  = lex
LINK = clang++
CXX  = clang++
NVCC = clang++

CUFLAGS  = -g -O3
DISABLED = -Wno-gnu-array-member-paren-init
CFLAGS   = -Wall -DYY_NO_INPUT -std=c++11 -g -O3  `pkg-config gtkmm-3.0 --cflags`
INCPATH  = -Iload/ -Imodel/ -Irender/ -Iutil/ -Igui/
LIBRARY  = -lboost_filesystem -lboost_system -lboost_program_options \
           -lboost_chrono -lboost_thread `pkg-config gtkmm-3.0 --libs`

EXES = $(patsubst %.cpp, ../%, $(wildcard *.cpp))
EOBJ = $(patsubst ../%, %.o, $(EXES))
OBJS = $(patsubst %.cpp, %.o, $(wildcard */*.cpp))
THRU = $(patsubst %.cu, %.o,  $(wildcard */*.cu))
HEAD = $(wildcard */*.hpp) util/Matrix.tpp

all: load/Parser.cpp load/Lexer.cpp 
	$(MAKE) -C . reload

load/Parser.cpp: load/Parser.y
	$(YACC) --defines=load/Parser.hpp $< -o $@

load/Lexer.cpp: load/Lexer.lex load/Parser.cpp
	$(LEX) --nounput -o $@ $<

reload: $(EXES)
$(EXES): ../%: %.cpp $(HEAD) $(OBJS) $(EOBJ) $(THRU)
	$(LINK) $(OBJS) $(THRU) $*.o $(LIBRARY) -o $@

$(OBJS): %.o: %.cpp $(HEAD) Makefile
	$(CXX) -c $(INCPATH) $(DISABLED) $(CFLAGS) $(DEF) $< -o $@

$(THRU): %.o: %.cu $(HEAD) Makefile
	$(NVCC) -c $(INCPATH) $(CUFLAGS) $< -o $@

$(EOBJ): %.o: %.cpp $(HEAD) Makefile
	$(CXX) -c $(INCPATH) $(DISABLED) $(CFLAGS) $(DEF) $< -o $@

clean:
	rm -f $(OBJS) $(THRU)
	rm -f load/Parser.cpp
	rm -f load/Parser.hpp
	rm -f load/Lexer.cpp
	rm -f $(EXES) $(EOBJ)
