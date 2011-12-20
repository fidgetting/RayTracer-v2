/*
 * Lexer.h
 *
 *  Created on: Sep 8, 2010
 *      Author: norton
 */

#ifndef QUEUE_TPP_INCLUDE
#define QUEUE_TPP_INCLUDE

#include <condition_variable>
#include <mutex>
#include <deque>

namespace ray {

  template<typename T>
  class concurrent_queue {
    public:

      const unsigned int max_size;

      typedef typename std::deque<T*>::iterator       iterator;
      typedef typename std::deque<T*>::const_iterator const_iterator;

      concurrent_queue() : max_size(10000), _alive(true) { }
      virtual ~concurrent_queue() { }

      inline iterator       begin()       { return _queue.begin(); }
      inline const_iterator begin() const { return _queue.begin(); }
      inline iterator         end()       { return _queue.end();   }
      inline const_iterator   end() const { return _queue.end();   }

      void push(T* t);

      void start();
      void stop();

      unsigned int size() const { return _queue.size(); }
      void worker();

    protected:

      std::condition_variable _wait;
      std::mutex              _lock;
      std::deque<T*>          _queue;
      bool                    _alive;
  };

  template<typename T>
  void concurrent_queue<T>::push(T* t) {
    std::unique_lock<std::mutex> ul(_lock);
    /*if(_queue.size() >= max_size)
      _wait.wait(ul);*/

    _queue.push_back(t);
  }

  template<typename T>
  void concurrent_queue<T>::start() {
    std::unique_lock<std::mutex> ul(_lock);
    _alive = true;
  }

  template<typename T>
  void concurrent_queue<T>::stop() {
    std::unique_lock<std::mutex> ul(_lock);
    _alive = false;
  }

  template<typename T>
  void concurrent_queue<T>::worker() {
    T* ret;

    while(_alive || _queue.size() != 0) {
      {
        std::unique_lock<std::mutex> ul(_lock);
        if(size() != 0) {
          ret = _queue.front();
          _queue.pop_front();

          //if(_queue.size() < (max_size / 2)) {
          //_wait.notify_all();
        } else {
          break;
        }
      }

      if(ret->operator()()) {
        std::unique_lock<std::mutex> ul(_lock);
        _queue.push_back(ret);
      } else {
        delete ret;
      }
    }
  }

}

#endif /* QUEUE_TPP_INCLUDE */
