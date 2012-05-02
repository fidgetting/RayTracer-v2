/*
 * threadpool.cpp
 *
 *  Created on: Mar 7, 2012
 *      Author: norton
 */

#include <threadpool.hpp>

/**
 * Constructor simply set the head and tail to NULL and sets the number of
 * threads. This does not start the threads in the thread pool. For that one
 * must call threadpool::start() after the constructor has finished.
 *
 * @param threads  the number of threads in the thread pool
 */
ray::threadpool::threadpool(uint8_t threads, int32_t capacity) :
_lock(), _wait(), _full(), _threads(), _queue(), _running(true),
_nthreads(threads), _capacity(capacity), _waiting(0), _working(0) { }

/**
 * @brief Stops all threads in the threadpool and joins them.
 */
void
ray::threadpool::stop() {
  {
    std::unique_lock<std::mutex> _l(_lock);
    _running = false;
    _wait.notify_all();
  }

  for(std::shared_ptr<std::thread> curr : _threads) {
    curr->join();
  }
}

/**
 * @brief wait for the threadpool to finish all currently queued work.
 *
 * This function will block until the threadpool has finished all currently
 * queue worked. Once all work has finished this will call stop().
 */
void
ray::threadpool::join() {
  {
    std::unique_lock<std::mutex> _l(_lock);

    while(_queue.size() != 0) {
      _full.wait(_l);
    }
  }

  stop();
}

/**
 * TODO
 *
 * @return
 */
std::function<void(void)>
ray::threadpool::_take() {
  std::unique_lock<std::mutex> _l(_lock);
  std::function<void(void)> retval;

  while(_queue.size() == 0) {
    if(!_running)
      return NULL;

    _waiting++;
    _working--;

    _wait.wait(_l);

    _working++;
    _waiting--;

    if(!_running)
      return NULL;
  }

  retval = _queue.front();
  _queue.pop_front();
  _full.notify_all();
  return retval;
}

/**
 * TODO
 *
 * @param id
 */
void
ray::threadpool::_exec_thread(uint8_t id) {
  std::function<void(void)> curr;

  while(_running) {
    if((curr = _take()) != NULL) {
      curr();
      curr = NULL;
    }
  }
}


