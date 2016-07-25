/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */

#pragma once

#include <mutex>
#include <thread>
#include <sstream>
#include <string.h>
#include <condition_variable>

namespace surround360 {
/// An in-place, thread-aware producer consumer messaging class
///
/// This is an implementation of a thread aware producer/consumer
/// class that uses an in-place circular buffer.  In-place means that
/// the circular buffer directly stores the items messaged from the
/// producer to the consumer. As such, care must be taken to insure
/// that both head/tail access and head/tail advancement are mutually
/// protected from overwrite and wrap around.  We achieve this using
/// two condition variables that "signal" when the producer and
/// consumer can proceed.  The circular buffer state is protected
/// using a single mutex.  Strictly speaking only a head and tail
/// pointer are necessary and sufficient for proper operation of the
/// circular buffer, however a count is added to make the code easier
/// read and maintain. An additional "done" routine is provided so
/// that the producer can signal the consumer when the producer is
/// truly done producing items.
///
  template <typename T, int LENGTH>
    class ProducerConsumer {
  private :
    T items[LENGTH];
    std::mutex m;
    std::condition_variable dataAvailable;
    std::condition_variable spaceAvailable;
    int head;
    int tail;
    int count;
    bool fini;

  public:
    // Don't allow any assignment or copies
    ProducerConsumer(ProducerConsumer const&) = delete;
    ProducerConsumer& operator=(ProducerConsumer const&) = delete;

    ///  The only constructor is the default constructor
    ///
    /// We do this because the size of the circular buffer used to
    /// share data between the producer and consumer is statically
    /// allocated.
    ProducerConsumer()
      : head(0), tail(0), count(0), fini(false)
    {
      memset(items, 0, LENGTH * sizeof(T));
    }

    void setBuffers(void *ptr, size_t size)
    {
      uint64_t vaddr = reinterpret_cast<uint64_t>(ptr);
      for (int k = 0; k < LENGTH; k++) {
        items[k].imageBytes = reinterpret_cast<uint8_t *>(vaddr + k * size);
      }
    }

    /// Signal the consuemr that producer is done producing.
    ///
    /// Used by the producer to notify the consumer that when the queue
    /// empty it can exit.
    void done() {
      std::unique_lock<std::mutex> lk(m);
      fini = true;
      lk.unlock();
      dataAvailable.notify_one();
    }

    /// Access the head of the message queue.
    ///
    /// The routine will block if the queue is full. It used by the
    /// producer to copy data into the queue.
    ///
    /// @return Returns a poimnter to the head of the queue.
    T* getHead() {
      std::unique_lock<std::mutex> lk(m);
      spaceAvailable.wait(lk, [this]{return this->count < LENGTH;});
      lk.unlock();
      return &items[head];
    }

    /// Sends the message the value onto the queue.
    ///
    /// Thread safe queue state update.
    void advanceHead() {
      std::unique_lock<std::mutex> lk(m);
      head = (head + 1) % LENGTH;
      ++count;
      lk.unlock();
      dataAvailable.notify_one();
    }

    /// Accesses data at the tail of the message queue.
    ///
    /// This routine will block until there is something available. The
    /// normal usage is for the consumer to loop on the value of this
    /// pointer until it null indicating that the queue is empty and the
    /// producer is done producing.
    ///
    // @return Returns the pointer to the data or null if the queue is
    // empty and the producer has called done().
    T* getTail() {
      T* val = nullptr;
      std::unique_lock<std::mutex> lk(m);
      dataAvailable.wait(lk, [this]{return this->count > 0 || fini;});
      if (this->count > 0) {
        val = &items[tail];
      }
      lk.unlock();
      return val;
    }

    /// Advances the tail pointer indicating tail consumption.
    ///
    /// Used by the consumer to signal that it has infact
    /// consumed/used/copied the last getTail.
    void advanceTail() {
      std::unique_lock<std::mutex> lk(m);
      if (this->count > 0) {
        tail = (tail + 1) % LENGTH;
        --count;
      }
      lk.unlock();
      spaceAvailable.notify_one();
    }

    // Used to debug the state of the circular buffer
    std::string stateString() {
      std::stringstream ss;
      m.lock();
      ss << "[head=" << tail
         << " tail=" << head
         << " count=" << count
         << " fini=" << fini
         << "]";
      m.unlock();
      return ss.str();
    }
  };
}
