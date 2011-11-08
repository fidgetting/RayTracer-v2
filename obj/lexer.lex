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
#define yyposs_inc      yyposs += strlen(yytext); std::cout << yytext;
#define yyposs_ret(ret) yyposs += strlen(yytext); std::cout <<  "[" << yytext \
                        << "," << ret << "]"; return ret;
#else
#define yyposs_inc      yyposs += strlen(yytext);
#define yyposs_ret(ret) yyposs += strlen(yytext); return ret;
#endif

%}

num [0-9]
not_num [a-zA-Z_~\(\)]
id  {not_num}({not_num}|{num}){2,}
eol (\n|\r|\r\n)
ws  ([ \t])+
comment ^("#").*$
float -?{num}+(\.{num}+)?

%%


"usemtl" { yyposs_ret(USEMTL)     }

{float}  { strcpy(yylval.str_t, yytext); yyposs_ret(NUM_LIT); }
{id}     { strcpy(yylval.str_t, yytext); yyposs_ret(STRING_LIT); }

"g"      { yyposs_ret(GROUP);     }
"v"      { yyposs_ret(VERTEX);    }
"vt"     { yyposs_ret(TEXTURE);   }
"vn"     { yyposs_ret(NORMAL);    }
"f"      { yyposs_ret(FACE);      }
"/"      { yyposs_ret(SLASH);     }
"r"      { yyposs_ret(ROTATE);    }
"t"      { yyposs_ret(TRANSLATE); }
"s"      { yyposs_ret(SCALE);     }
"a"      { yyposs_ret(ARBITRARY); }
"c"      { yyposs_ret(CAMERA);    }
"w"      { yyposs_ret(WIREFRAME); }
"l"      { yyposs_ret(LIGHT);     }
"m"      { yyposs_ret(MATERIAL);  }
"gr"     { yyposs_ret(SHADER);    }

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

