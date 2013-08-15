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
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/* other */
#include <gtkmm.h>
#include <gdk/gdk.h>

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

  /* check program options */
  po::options_description opt("Options");
  opt.add_options()
      ("help,h",                           "produce help message")
      ("model",  po::value<std::string>(), "model input file")
      ("output", po::value<std::string>(), "image output file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(opt).run(), vm);
  po::notify(vm);

  if(vm.count("help") || !vm.count("model") || !vm.count("output")) {
    opt.print(std::cout);
    return -1;
  }

  /* load the model */
  auto stream = ray::ObjectStream::loadObject(vm["model"].as<std::string>());

  ray::Model  model;
  ray::Camera camera;

  ray::Model::fromObjectStream(stream, model, camera);

  /* render the image */
  fs::path output(vm["output"].as<std::string>());

  try {
    copyOut(model.click(camera, 1024, 1024))->save(
      output.string(), output.extension().string().substr(1));
  } catch(Gdk::PixbufError& error) {
    std::cout << error.what() << std::endl;
  }

  return 0;
}


