/*
 * Rays.cpp
 *
 *  Created on: Jan 9, 2013
 *      Author: norton
 */

/* local includes */
#include <ObjectStream.hpp>
#include <Model.hpp>
#include <Vector.hpp>
#include <Debug.hpp>

/* std includes */
#include <iostream>

/* boost includes */
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

/* other */
#include <gtkmm.h>
#include <gdk/gdk.h>

const char* usage = "Usage: Tracer <model file> <output file>";

Glib::RefPtr<Gdk::Pixbuf> copyOut(const ray::Matrix<ray::Pixel> img) {
  Glib::RefPtr<Gdk::Pixbuf> ret = Gdk::Pixbuf::create(
      Gdk::COLORSPACE_RGB,
      false,
      8,
      img.cols(),
      img.rows());
  ray::Pixel* data = reinterpret_cast<ray::Pixel*>(ret->get_pixels());

  for(int i = 0; i < img.rows(); i++) {
    for(int j = 0; j < img.cols(); j++) {
      data[j + i * img.cols()] = img[i][j];
    }
  }

  return ret;
}

int main(int argc, char** argv) {
  Glib::RefPtr<Gtk::Application> app =
      Gtk::Application::create(argc, argv);

  if(argc != 3) {
    std::cout << usage << std::endl;
    return -1;
  }
  
  /* validate inputs */
  fs::path model  = argv[1];
  fs::path result = argv[2];
  
  if(!fs::is_regular_file(model)) {
    std::cout << usage << std::endl;
    return -1;
  }
  
  if(fs::is_directory(result)) {
    result = result / "out.png";
  }

  /* load the model */
  auto stream = ray::ObjectStream::loadObject(model);

  ray::Model  model;
  ray::Camera camera;

  ray::Model::fromObjectStream(stream, model, camera);

  /* render the image */
  try {
    copyOut(model.click(camera, 1024, 1024))->save(
      result.string(), result.extension().string().substr(1));
  } catch(Gdk::PixbufError& error) {
    std::cout << error.what() << std::endl;
  }

  return 0;
}


