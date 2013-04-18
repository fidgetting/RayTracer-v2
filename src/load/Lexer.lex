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
  #include <Parser.hpp>

  /* std includes */
  #include <cstring>
  #include <iostream>

  int yyline = 0;
  int yyposs = 0;

#ifdef DEBUG_PARSE
#define yyposs_inc      yyposs += strlen(yytext); std::cout << yytext;
#define yyposs_ret(ret) yyposs += strlen(yytext); std::cout <<  "[" << yytext \
                        << "," << ret << "]"; return ret
#else
#define yyposs_inc      yyposs += strlen(yytext)
#define yyposs_ret(ret) yyposs += strlen(yytext); return ret
#endif

%}

num [0-9]
not_num [a-zA-Z_~\(\).]
id  {not_num}({not_num}|{num}){2,}
eol (\n|\r|\r\n)
ws  ([ \t])+
comment ^("#").*$
double [-+]?{num}*\.?{num}+([eE][-+]?{num}+)?

%%


"usemtl" { yyposs_ret(USEMTL);    }
"newmtl" { yyposs_ret(NEWMTL);    }
"mtllib" { yyposs_ret(MTLLIB);    }
"illum"  { yyposs_ret(ILLUM);     }

{double}  { strcpy(yylval.str_t, yytext); yyposs_ret(NUM_LIT); }
{id}     { strcpy(yylval.str_t, yytext); yyposs_ret(STRING_LIT); }

"g"      { yyposs_ret(GROUP);     }
"v"      { yyposs_ret(VERTEX);    }
"vt"     { yyposs_ret(TEXTURE);   }
"vn"     { yyposs_ret(NORMAL);    }
"f"      { yyposs_ret(FACE);      }
"/"      { yyposs_ret(SLASH);     }

"Ka"     { yyposs_ret(KAMBIENT);  }
"Kd"     { yyposs_ret(KDIFFUSE);  }
"Ks"     { yyposs_ret(KSPECULAR); }
"Ns"     { yyposs_ret(PHONG);     }

{comment}   { yyposs_inc; }
{ws}        { yyposs_inc; }
{eol}       {
#ifdef DEBUG_PARSE
  std::cout << std::endl;
#endif 
  yyposs = 0; yyline++;
}
. { std::cout << "Illegal character: " << yytext << std::endl; }

%%

int yywrap(void) {
  return 1;
}

