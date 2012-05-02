/*
 * threadpool.hpp
 *
 *  Created on: Mar 7, 2012
 *      Author: norton
 */

#pragma once

/* std includes */
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <deque>
#include <vector>

namespace ray {

  /**
   * TODO
   */
  class threadpool {
    public:

      threadpool(uint8_t nthreads, int32_t capacity = -1);
      virtual ~threadpool() { }

      void stop();
      void join();

      template<typename _Callable, typename... _Args>
      void put(_Callable&& __f, _Args&&... __args);

    protected:

      std::function<void(void)> _take();

      void _exec_thread(uint8_t id);

      std::mutex                                 _lock;
      std::condition_variable                    _wait;
      std::condition_variable                    _full;
      std::vector<std::shared_ptr<std::thread> > _threads;
      std::deque<std::function<void(void)>>      _queue;

      bool    _running;
      uint8_t _nthreads;
      int32_t _capacity;

      uint8_t _waiting;
      uint8_t _working;
  };

  /**
   * TODO
   *
   * @param __f
   * @param __args
   */
  template<typename _Callable, typename... _Args>
  void threadpool::put(_Callable&& __f, _Args&&... __args) {
    std::unique_lock<std::mutex> _l(_lock);

    while(_capacity > 0 && _queue.size() == uint32_t(_capacity)) {
      _full.wait(_l);
    }

    _queue.push_back(std::bind<void>(
            std::forward<_Callable>(__f),
            std::forward<_Args>(__args)...));

    if(_waiting == 0 && _threads.size() < _nthreads) {
      _threads.push_back(std::make_shared<std::thread>(
          &threadpool::_exec_thread, this, _threads.size()));
    }

    _wait.notify_one();
  }
}






