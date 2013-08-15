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

/* local includes */
#include <Vector.hpp>
#include <ObjLoader.hpp>
using namespace ray;
using namespace obj;

/* std inlcudes */
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

extern int yylex();
extern int yyline;
extern int yyposs;
extern FILE* yyin;

std::string        objn = "default";
std::string        mtln = "default_model_material";
std::string        mtlr = "";

std::vector<Vector> verts;
std::vector<Vector> texts;
std::vector<Vector> norms;

std::vector<int> faceVerts;
std::vector<int> faceTexts;
std::vector<int> faceNorms;

extern std::vector<std::string> objLibs;
extern ObjLoader*               objDest;

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
    { verts.push_back(Vector(atof($2), atof($3), atof($4))); }
    
  | VERTEX NUM_LIT NUM_LIT NUM_LIT NUM_LIT
    { verts.push_back(Vector(atof($2), atof($3), atof($4))); }
    
  | TEXTURE NUM_LIT NUM_LIT
    { texts.push_back(Vector(atof($2), atof($3), 0.0)); }
    
  | NORMAL NUM_LIT NUM_LIT NUM_LIT
    { norms.push_back(Vector(atof($2), atof($3), atof($4))); }
    
  | FACE
    { faceVerts.clear(); faceTexts.clear(); faceNorms.clear(); }
    exprlist
    {
      objDest->pushPolygon(
        ObjLoader::Polygon(
          faceVerts,
          faceTexts,
          faceNorms,
          mtln));
    }
    
  | USEMTL STRING_LIT
    { mtln = $2; }
    
  | MTLLIB STRING_LIT
    { objLibs.push_back($2); }
    
  | GROUP STRING_LIT
    { objn = $2; }
    
  | NEWMTL STRING_LIT
    { mtlr = $2; objDest->makeMaterial(mtlr); }
    
  | PHONG NUM_LIT
    { objDest->getMaterial(mtlr).phong = atof($2); }  
    
  | ILLUM NUM_LIT
    { objDest->getMaterial(mtlr).illum = atoi($2); }
    
  | KAMBIENT NUM_LIT NUM_LIT NUM_LIT
    { objDest->getMaterial(mtlr).ka = Vector(atof($2), atof($3), atof($4)); }
    
  | KDIFFUSE NUM_LIT NUM_LIT NUM_LIT
    { objDest->getMaterial(mtlr).kd = Vector(atof($2), atof($3), atof($4)); }
    
  | KSPECULAR NUM_LIT NUM_LIT NUM_LIT
    { objDest->getMaterial(mtlr).ks = Vector(atof($2), atof($3), atof($4)); }
;

exprlist:
    exprlist expr
  | expr
;

expr:
    NUM_LIT
    { faceVerts.push_back(atoi($1) - 1);
      faceTexts.push_back(-1);
      faceNorms.push_back(-1);}
  | NUM_LIT SLASH NUM_LIT
    { faceVerts.push_back(atoi($1) - 1);
      faceTexts.push_back(atoi($3) - 1);
      faceNorms.push_back(-1); }
  | NUM_LIT SLASH NUM_LIT SLASH NUM_LIT
    { faceVerts.push_back(atoi($1) - 1);
      faceTexts.push_back(atoi($3) - 1);
      faceNorms.push_back(atoi($5) - 1); }
  | NUM_LIT SLASH SLASH NUM_LIT
    { faceVerts.push_back(atoi($1) - 1);
      faceTexts.push_back(-1);
      faceNorms.push_back(atoi($4) - 1); }
;

%%




