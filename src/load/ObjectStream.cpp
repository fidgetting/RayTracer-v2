/*
 * ObjectStream.cpp
 *
 *  Created on: Jan 10, 2013
 *      Author: norton
 */

/* local includes */
#include <ObjectStream.hpp>
#include <ObjLoader.hpp>

namespace ray {

  /**
   * Static function used to match the end of a string.
   *
   * @param str  the string to match
   * @param end  what the end should match
   * @return     true if str ends with end
   */
  static bool stringEndsWith(std::string str, std::string end) {

    if(str.length() < end.length())
      return false;

    for(auto stri = str.rbegin(), stre = end.rbegin();
        stre != end.rbegin(); stri++, stre++) {
      if(*stri != *stre)
        return false;
    }

    return true;
  }

  /**
   * Factory function for loading Object files. This will check what type of
   * file was provided based on the file type suffix.
   *
   * @param fname  the name of the file
   * @return       a ObjectStream pointer
   */
  ObjectStream::ptr ObjectStream::loadObject(std::string fname) {

    if(stringEndsWith(fname, obj::ObjLoader::suffix))
      return std::make_shared<obj::ObjLoader>(fname);

    return ObjectStream::ptr(nullptr);
  }
}



