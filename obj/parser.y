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

std::string        objn = "default";
std::string        mtln = "";
std::vector<int>   verts;
std::vector<int>   texts;
std::vector<int>   norms;
std::istringstream istr;

int    store;
int id_gen = 0;

void yyerror(const char* msg) {
  std::cout << msg << std::endl;
}

%}

%union {
  char str_t[256];
}

%token         ROTATE TRANSLATE SCALE ARBITRARY
%token         VERTEX TEXTURE NORMAL
%token         MATERIAL LIGHT SHADER TRACER USEMTL
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
    { (*dest)[objn].push_v(objstream::vertex(atof($2), atof($3), atof($4))); }
    
  | VERTEX NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    { (*dest)[objn].push_v(
         objstream::vertex(atof($2), atof($3), atof($4), atof($5))); }
    
  | TEXTURE NUM_LIT NUM_LIT
    { (*dest)[objn].push_t(objstream::texture(atof($2), atof($3))); }
    
  | NORMAL NUM_LIT NUM_LIT NUM_LIT
    { (*dest)[objn].push_n(objstream::vertex(atof($2), atof($3), atof($4))); }
    
  | FACE
    { verts.clear(); texts.clear(); norms.clear(); }
    exprlist
    {
      objstream::face f(verts, texts, norms, mtln);
      f.id = id_gen++;
    
      (*dest)[objn].push_f(f);
    }
  
  | ROTATE STRING_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    { (*dest)[$2].push_m(
         new objstream::rotate(atof($3), atof($4), atof($5), atof($6))); }
    
  | TRANSLATE STRING_LIT NUM_LIT NUM_LIT NUM_LIT
    { (*dest)[$2].push_m(
         new objstream::translate(atof($3), atof($4), atof($5))); }
  
  | SCALE STRING_LIT NUM_LIT NUM_LIT NUM_LIT
    { (*dest)[$2].push_m(
         new objstream::scale(atof($3), atof($4), atof($5))); }
  
  | ARBITRARY STRING_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
                         NUM_LIT NUM_LIT NUM_LIT NUM_LIT
                         NUM_LIT NUM_LIT NUM_LIT NUM_LIT
                         NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    {
      objstream::arbitrary* ar = new objstream::arbitrary();
      (*ar)[0][0] = atof($3);  (*ar)[0][1] = atof($4);  (*ar)[0][2] = atof($5);  (*ar)[0][3] = atof($6);
      (*ar)[1][0] = atof($7);  (*ar)[1][1] = atof($8);  (*ar)[1][2] = atof($9);  (*ar)[1][3] = atof($10);
      (*ar)[2][0] = atof($11); (*ar)[2][1] = atof($12); (*ar)[2][2] = atof($13); (*ar)[2][3] = atof($14);
      (*ar)[3][0] = atof($15); (*ar)[3][1] = atof($16); (*ar)[3][2] = atof($17); (*ar)[3][3] = atof($18);
      (*dest)[$2].push_m(ar);
    }
    
  | CAMERA STRING_LIT NUM_LIT 
                      NUM_LIT NUM_LIT NUM_LIT
                      NUM_LIT NUM_LIT NUM_LIT
                      NUM_LIT NUM_LIT NUM_LIT
    {
      objstream::camera c($2, atof($3));
      c.fp() [0] = atof($4);  c.fp() [1] = atof($5);  c.fp() [2] = atof($6);
      c.vpn()[0] = atof($7);  c.vpn()[1] = atof($8);  c.vpn()[2] = atof($9);
      c.vup()[0] = atof($10); c.vup()[1] = atof($11); c.vup()[2] = atof($12);
      dest->cam($2) = c;
    }
    
  | LIGHT NUM_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    {
      objstream::light* l = new objstream::light();
      
      l->poss()[0] = atof($2); l->poss()[1] = atof($3);
      l->poss()[2] = atof($4); l->poss()[3] = atof($5);
      
      l->illu()[0] = atof($6); l->illu()[1] = atof($7); l->illu()[2] = atof($8);
      
      dest->push_l(l);
    }
    
  | MATERIAL STRING_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    {
      objstream::material m($2);
      
      m.rgb()[0] = atof($3); m.rgb()[1] = atof($4); m.rgb()[2] = atof($5);
      m.s() = atof($6);
      m.t() = atof($8);
      m.alpha() = atof($7);
      
      dest->mat($2) = m;
    }
    
  | WIREFRAME STRING_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    {
      objstream::view* w = new objstream::view($2);
      
      w->minx() = atoi($3);
      w->miny() = atoi($4);
      w->maxx() = atoi($5);
      w->maxy() = atoi($6);
      
      w->type() = objstream::view::wireframe;
    
      dest->push(w);
    }
    
  | SHADER STRING_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    {
	  objstream::view* s = new objstream::view($2);
      
      s->minx() = atoi($3);
      s->miny() = atoi($4);
      s->maxx() = atoi($5);
      s->maxy() = atoi($6);
      
      s->type() = objstream::view::shader;
    
      dest->push(s);
    }
    
  | TRACER STRING_LIT NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    {
      objstream::view* t = new objstream::view($2);
      
      t->minx() = atoi($3);
      t->miny() = atoi($4);
      t->maxx() = atoi($5);
      t->maxy() = atoi($6);
      
      t->type() = objstream::view::tracer;
      
      dest->push(t);
    }
    
  | USEMTL STRING_LIT
    { mtln = $2; }
    
  | GROUP STRING_LIT
    { objn = $2; }
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




