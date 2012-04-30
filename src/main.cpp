/*
 * main.cpp
 *
 *  Created on: Dec 1, 2011
 *      Author: norton
 */

#include <camera.h>
#include <model.h>
#include <objstream.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

/* ************************************************************************** */
/* *** main function for ray tracer ***************************************** */
/* ************************************************************************** */

int main(int argc, char** argv) {

  po::options_description opt("Options");
  opt.add_options()
      ("help,h", "produce help message")
      ("smooth,s", "turn off smooth shading")
      ("interactive,i", "turn on the render animation")
      ("model,m", po::value<std::string>(), "model input file")
      ("X,x", po::value<int>(), "the x location of the debug pixel")
      ("Y,y", po::value<int>(), "the y location of the debug pixel");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(opt).run(), vm);
  po::notify(vm);

  if(vm.count("help")) {
    opt.print(std::cout);
    return -1;
  }

  obj::objstream obj(vm["model"].as<std::string>());
  ray::model::smooth_shading = !vm.count("smooth");
  ray::camera::x_print = vm.count("X") ? vm["X"].as<int>() : 0;
  ray::camera::y_print = vm.count("Y") ? vm["Y"].as<int>() : 0;
  ray::camera::animation = vm.count("interactive");
  ray::model m;

  m.build(obj);
  ray::camera c(m);
  ray::display d(m, c, false);
  d.exec();

  return 0;
}

