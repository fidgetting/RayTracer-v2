/*
 * ObjView.cpp
 *
 *  Created on: Mar 5, 2013
 *      Author: norton
 */

/* local includes */
#include <ObjView.hpp>
#include <ObjectStream.hpp>
#include <render.hpp>

/* boost includes */
#include <boost/chrono.hpp>

/* gtk incldues */
#include <gtkmm.h>
#include <gtkmm/stock.h>
#include <gdkmm.h>

namespace sc = boost::chrono;

namespace ray {
  namespace gui {

    const uint16_t width  = 1024;
    const uint16_t height = 1024;

    ObjViewer::ObjViewer() :
        window(nullptr),
        image(nullptr),
        builder(Gtk::Builder::create_from_file(
            "src/gui/ObjView.xml")),
        imgbuffer(Gdk::Pixbuf::create(
            Gdk::COLORSPACE_RGB,
            false,
            8,
            width,
            height)),
        model(),
        camera(),
        isButtonPressed(false)
    {
      builder->get_widget("ObjView", window);
      builder->get_widget("Image",   image);

      Gtk::ImageMenuItem* open;
      Gtk::ImageMenuItem* quit;

      builder->get_widget("FileOpen", open);
      builder->get_widget("FileQuit", quit);

      open->signal_activate().connect(
          sigc::mem_fun(*this, &ObjViewer::onOpen));
      quit->signal_activate().connect(
          sigc::mem_fun(*this, &ObjViewer::onQuit));

      window->add_events(Gdk::POINTER_MOTION_MASK);
      window->add_events(Gdk::BUTTON_PRESS_MASK);
      window->add_events(Gdk::BUTTON_RELEASE_MASK);

      window->signal_button_press_event().connect(
          sigc::mem_fun(*this, &ObjViewer::onPress));
      window->signal_motion_notify_event().connect(
          sigc::mem_fun(*this, &ObjViewer::onMouse));
      window->signal_button_release_event().connect(
          sigc::mem_fun(*this, &ObjViewer::onRelease));

      image->set(imgbuffer);
      window->set_resizable(false);
    }

    int ObjViewer::display(Glib::RefPtr<Gtk::Application> app) {
      return app->run(*window);
    }

    void ObjViewer::onOpen() {
      Gtk::FileChooserDialog dialog("Please choose a file",
          Gtk::FILE_CHOOSER_ACTION_OPEN);
      dialog.set_transient_for(*window);

      dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
      dialog.add_button(Gtk::Stock::OK,     Gtk::RESPONSE_OK);

      int result = dialog.run();

      switch(result)
      {
        case Gtk::RESPONSE_OK:
        {
          Model::fromObjectStream(
              ObjectStream::loadObject(dialog.get_filename()),
              model,
              camera);
          copyOut(model.click(camera, height, width));

          break;
        }
        case Gtk::RESPONSE_CANCEL: break;
      }

    }

    void ObjViewer::onQuit() {
      window->hide();
    }

    bool ObjViewer::onPress(GdkEventButton* drag) {
      isButtonPressed = true;
      return true;
    }

    bool ObjViewer::onMouse(GdkEventMotion* drag) {
      if(isButtonPressed) {
        std::cout << "[" << drag->x << ", " << drag->y << "]" << std::flush;
      }
      return true;
    }

    bool ObjViewer::onRelease(GdkEventButton* drag) {
      isButtonPressed = false;

      auto begin = sc::high_resolution_clock::now();
      auto out   = model.click(camera, height, width);
      auto end   = sc::high_resolution_clock::now();

      copyOut(out);

      std::cout << "Render time:["
          << (sc::duration_cast<sc::milliseconds>(end - begin)).count()
          << "ms]" << std::endl;

      return true;
    }

    void ObjViewer::copyOut(const Matrix<Pixel>& in) {
      ray::Pixel* data = reinterpret_cast<ray::Pixel*>(imgbuffer->get_pixels());

      for(int i = 0; i < in.rows(); i++) {
        for(int j = 0; j < in.cols(); j++) {
          data[j + i * in.cols()] = in[i][j];
        }
      }
    }

  }

}

