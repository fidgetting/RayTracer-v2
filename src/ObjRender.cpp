/*
 * Rays.cpp
 *
 *  Created on: Mar 5, 2013
 *      Author: norton
 */

/* local includes */
#include <ObjView.hpp>

int main(int argc, char** argv) {
  Glib::RefPtr<Gtk::Application> app =
      Gtk::Application::create(argc, argv);

  ray::gui::ObjViewer view;

  return view.display(app);
}

