/*
 * ObjView.hpp
 *
 *  Created on: Mar 5, 2013
 *      Author: norton
 */

#pragma once

/* local includes */
#include <Model.hpp>

/* gtk includes */
#include <gtkmm.h>
#include <gdk/gdk.h>

namespace ray {
  namespace gui {

    class ObjViewer {
      public:

        ObjViewer();

        int display(Glib::RefPtr<Gtk::Application> app);

      private:

        void onOpen();
        void onQuit();

        void copyOut(const Matrix<Pixel>& in);

        bool onPress  (GdkEventButton* drag);
        bool onMouse  (GdkEventMotion* drag);
        bool onRelease(GdkEventButton* drag);

        Gtk::Window* window;
        Gtk::Image*  image;

        Glib::RefPtr<Gtk::Builder> builder;
        Glib::RefPtr<Gdk::Pixbuf>  imgbuffer;

        ray::Model  model;
        ray::Camera camera;

        bool isButtonPressed;
    };

  }

}

