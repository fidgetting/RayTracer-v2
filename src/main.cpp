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
  std::string f_command;
  std::string f_model;

  po::options_description opt("Options");
  opt.add_options()
      ("help,h", "produce help message")
      ("smooth,s", "turn on smooth shading")
      ("vertex,v", po::value<int>(), "turn on vertex spheres")
      ("interactive,i", "turn on the render animation")
      ("cmd,c", po::value<std::string>(), "command input file")
      ("mod,m", po::value<std::string>(), "model input file")
      ("X,x", po::value<int>(), "the x location of the debug pixel")
      ("Y,y", po::value<int>(), "the y location of the debug pixel");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(opt).run(), vm);
  po::notify(vm);

  if(vm.count("help")) {
    opt.print(std::cout);
    return -1;
  }

  obj::objstream obj(vm["mod"].as<std::string>());
  obj::objstream cmd(vm["cmd"].as<std::string>());
  ray::model::smooth_shading = vm.count("smooth");
  ray::model::vertex_spheres = vm.count("vertex") ? vm["vertex"].as<int>() : 0;
  ray::camera::x_print = vm.count("X") ? vm["X"].as<int>() : 0;
  ray::camera::y_print = vm.count("Y") ? vm["Y"].as<int>() : 0;
  ray::camera::animation = vm.count("interactive");
  ray::model m;

  m.build(obj);
  m.cmd(cmd);

  for(int i = 0; i < cmd.size(); i++) {
    ray::camera c(cmd.cam(cmd[i]->name()));
    std::ostringstream ostr;

    c.umin() = cmd[i]->minx();
    c.umax() = cmd[i]->maxx();
    c.vmin() = cmd[i]->miny();
    c.vmax() = cmd[i]->maxy();

    if(cmd[i]->type() == obj::objstream::view::wireframe) {
      ray::display disp(&m, &c);
      disp.exec();
    } else if(cmd[i]->type() == obj::objstream::view::shader){
      ray::display disp(&m, &c, false);
      disp.exec();
    } else {
      cv::Mat image(c.height(), c.width(), CV_8UC3);
      c.click(&m, image);
    }
  }

  return 0;
}

