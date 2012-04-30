/* ****************************************************************************
 * Copyright (C) 2011 Alex Norton                                             *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify it    *
 * under the terms of the BSD 2-Clause License.                               *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRENTY; without even the implied warranty of                 *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 **************************************************************************** */
 
%{

#include <objstream.hpp>
using namespace obj;

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

extern int yylex();
extern int yyline;
extern int yyposs;
extern FILE* yyin;

FILE* yyin_tmp;

std::string        objn = "default";
std::string        mtln = "default_model_material";
std::string        mtlr = "";
std::vector<int>   verts;
std::vector<int>   texts;
std::vector<int>   norms;
std::istringstream istr;

int    store;
int id_gen = 0;

void yyerror(const char* msg) {
  std::cout << msg << " " << yyline << ":" << yyposs << std::endl;
}

%}

%union {
  char str_t[256];
}

%token         NEWMTL KAMBIENT KDIFFUSE KSPECULAR PHONG ILLUM
%token         USEMTL MTLLIB
%token         VERTEX TEXTURE NORMAL
%token         GROUP
%token         FACE
%token         SLASH
%token         OBJF
%token         CAMERA
%token         WIREFRAME
%token <str_t> NUM_LIT
%token <str_t> STRING_LIT

%type <str_t> model
%type <str_t> stmt
%type <str_t> stmtlist
%type <str_t> expr
%type <str_t> exprlist

%start model

%% /* Grammar rules and actions */

model:
    stmtlist
  | /* epsilon */
;

stmtlist:
    stmtlist stmt
  | stmt
;

stmt:
    VERTEX NUM_LIT NUM_LIT NUM_LIT
    { dest->push_v(objstream::vertex(atof($2), atof($3), atof($4))); }
    
  | VERTEX NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    { dest->push_v(objstream::vertex(atof($2), atof($3), atof($4), atof($5))); }
    
  | TEXTURE NUM_LIT NUM_LIT
    { dest->push_t(objstream::texture(atof($2), atof($3))); }
    
  | NORMAL NUM_LIT NUM_LIT NUM_LIT
    { dest->push_n(objstream::vertex(atof($2), atof($3), atof($4))); }
    
  | FACE
    { verts.clear(); texts.clear(); norms.clear(); }
    exprlist
    {
      objstream::face f(verts, texts, norms, mtln);
      f.id = id_gen++;
    
      (*dest)[objn].push_f(f);
    }
    
  | USEMTL STRING_LIT
    { mtln = $2; }
    
  | MTLLIB STRING_LIT
    { dest->push_mtllib($2); }
    
  | GROUP STRING_LIT
    { objn = $2; }
    
  | NEWMTL STRING_LIT
    { mtlr = $2; dest->mat(mtlr) = objstream::material(mtlr); }
    
  | PHONG NUM_LIT
    { dest->mat(mtlr).phong() = atof($2); }  
    
  | ILLUM NUM_LIT
    { dest->mat(mtlr).illum() = atoi($2); }
    
  | KAMBIENT NUM_LIT NUM_LIT NUM_LIT
    {
      dest->mat(mtlr).ka()[0] = atof($2);
      dest->mat(mtlr).ka()[1] = atof($3);
      dest->mat(mtlr).ka()[2] = atof($4);
    }
    
  | KDIFFUSE NUM_LIT NUM_LIT NUM_LIT
    {
      dest->mat(mtlr).kd()[0] = atof($2);
      dest->mat(mtlr).kd()[1] = atof($3);
      dest->mat(mtlr).kd()[2] = atof($4);
    }
    
  | KSPECULAR NUM_LIT NUM_LIT NUM_LIT
    {
      dest->mat(mtlr).ks()[0] = atof($2);
      dest->mat(mtlr).ks()[1] = atof($3);
      dest->mat(mtlr).ks()[2] = atof($4);
    }
;

exprlist:
    exprlist expr
  | expr
;

expr:
    NUM_LIT
    { store = atoi($1) - 1; verts.push_back(store); }
  | NUM_LIT SLASH NUM_LIT
    { store = atoi($1) - 1; verts.push_back(store);
      store = atoi($3) - 1; texts.push_back(store); }
  | NUM_LIT SLASH NUM_LIT SLASH NUM_LIT
    { store = atoi($1) - 1; verts.push_back(store);
      store = atoi($3) - 1; texts.push_back(store);
      store = atoi($5) - 1; norms.push_back(store); }
  | NUM_LIT SLASH SLASH NUM_LIT
  	{ store = atoi($1) - 1; verts.push_back(store);
  	  store = atoi($4) - 1; norms.push_back(store); }
;

%%




