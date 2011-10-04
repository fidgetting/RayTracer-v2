/*
 * Lexer.h
 *
 *  Created on: Sep 8, 2010
 *      Author: norton
 */

#ifndef QUEUE_TPP_INCLUDE
#define QUEUE_TPP_INCLUDE

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <deque>

namespace ray {

  template<typename T>
  class concurrent_queue {
    public:

      typedef typename std::deque<T*>::iterator       iterator;
      typedef typename std::deque<T*>::const_iterator const_iterator;

      concurrent_queue() { }
      virtual ~concurrent_queue() { }

      inline iterator       begin()       { return _queue.begin(); }
      inline const_iterator begin() const { return _queue.begin(); }
      inline iterator         end()       { return _queue.end();   }
      inline const_iterator   end() const { return _queue.end();   }

      void push(T* t);

      unsigned int size() const { return _queue.size(); }
      void worker();

    protected:

      boost::mutex       _lock;
      std::deque<T*>   _queue;
  };

  template<typename T>
  void concurrent_queue<T>::push(T* t) {
    boost::unique_lock<boost::mutex> ul(_lock);
    _queue.push_back(t);
  }

  template<typename T>
  void concurrent_queue<T>::worker() {
    T* ret;

    while(size() != 0) {
      {
        boost::unique_lock<boost::mutex> ul(_lock);
        if(size() != 0) {
          ret = _queue.front();
          _queue.pop_front();
        } else {
          break;
        }
      }

      if(ret->operator()()) {
        boost::unique_lock<boost::mutex> ul(_lock);
        _queue.push_back(ret);
      } else {
        delete ret;
      }
    }
  }

}

#endif /* QUEUE_TPP_INCLUDE */
