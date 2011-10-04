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
  #include <cstring>
  #include <iostream>
  #include <parser.hpp>

  int yyline = 0;
  int yyposs = 0;

#ifdef DEBUG
#define yyposs_inc yyposs += strlen(yytext); std::cout << yytext;
#else
#define yyposs_inc yyposs += strlen(yytext);
#endif

%}

num [0-9]
not_num [a-zA-Z_~\-\(\)]
id  {not_num}({not_num}|{num}){2,}
eol (\n|\r|\r\n)
ws  ([ \t])+
comment ^("#").*$
float -?{num}+(\.{num}+)?

%%


"usemtl" { yyposs_inc; return MTL;     }
"mtllib" { yyposs_inc; return MTLLIB;  }

{id}     { yyposs_inc; strcpy(yylval.str_t, yytext); return STRING_LIT; }
{float}  { yyposs_inc; strcpy(yylval.str_t, yytext); return NUM_LIT;  }

"g"      { yyposs_inc; return GROUP;     }
"v"      { yyposs_inc; return VERTEX;    }
"vt"     { yyposs_inc; return TEXTURE;   }
"vn"     { yyposs_inc; return NORMAL;    }
"f"      { yyposs_inc; return FACE;      }
"/"      { yyposs_inc; return SLASH;     }
"r"      { yyposs_inc; return ROTATE;    }
"t"      { yyposs_inc; return TRANSLATE; }
"s"      { yyposs_inc; return SCALE;     }
"a"      { yyposs_inc; return ARBITRARY; }

{comment}   { yyposs_inc; }
{ws}        { yyposs_inc; }
{eol}       {
#ifdef DEBUG
  std::cout << std::endl;
#endif 
  yyposs = 0; yyline++;
}
. { std::cout << "Illegal character: " << yytext << std::endl; }

%%

int yywrap(void) {
  return 1;
}

