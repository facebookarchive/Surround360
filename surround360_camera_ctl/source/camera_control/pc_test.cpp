/**
 * Copyright (c) 2016-present, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE_camera_ctl file in the root directory of this subproject.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "ProducerConsumer.h"

#undef DEBUG_PUSH

using namespace std;
using namespace surround360;

#define BUFFER_SIZE    47
#define CONSUMER_COUNT 5
#define IO_SIZE        (BUFFER_SIZE * CONSUMER_COUNT * 200)

int output[IO_SIZE];

// Random time upto in 100's micro-second range This is to jitter the
//   producer and consumer as if they have indeterminate timing.
inline useconds_t randomSleep(const int s) {
  return usleep(random() & ((1 << s) - 1));
}


// Consumer thread function
void consumer(ProducerConsumer<int, BUFFER_SIZE>* cb) {
  // Keep popping values off until we're handed a null pointer
  // indicating that there is no more data to consume.
  int* ptr;
  while ((ptr = cb->getTail()) != NULL) {
    output[*ptr] = *ptr;
    cb->advanceTail();
    randomSleep(10);
  }
}


// Producer thread - should only be one of these for this test.
void producer(const int itemCount,
              const int consumerCount,
              ProducerConsumer<int, BUFFER_SIZE>* cb) {

  for (int i = 0; i < itemCount; ++i) {
    const int cid = i % consumerCount;
    int* ptr = cb[cid].getHead();
    *ptr = i;
    cb[cid].advanceHead();
#ifdef DEBUG_PUSH
    cout << "pushed[" << cid << "] = " << i
         << " " << cb[cid].stateString() << endl;
#endif
    randomSleep(3);
  }

  // Notify all the consumers when we're done
  for (int cid = 0; cid < consumerCount; ++cid) {
    cb[cid].done();
  }
}


int main(int argc, char* argv[]) {
  // Set the output to "known" marker value
  for (int i = 0; i < IO_SIZE; i++) {
    output[i] = 0xdeadbeef;
  }

  // Create all producer consumer queues
  ProducerConsumer<int, BUFFER_SIZE> cb[CONSUMER_COUNT];

  // Create and start the consumer threads
  thread* consumerThreads[CONSUMER_COUNT];

  for (int i = 0; i < CONSUMER_COUNT; ++i) {
    consumerThreads[i] = new thread(consumer, &cb[i]);
  }

  // Create the producer thread
  thread producerThread(producer, IO_SIZE, CONSUMER_COUNT, cb);

  // Wait for the threads to finish
  producerThread.join();
  for (int i = 0; i < CONSUMER_COUNT; ++i) {
    consumerThreads[i]->join();
  }

  int status = EXIT_SUCCESS;
  for (int i = 0; i < IO_SIZE; i++) {
    if (i !=  output[i]) {
      cerr << "Failed p-c test at " << i << " != "
           << output[i] << " == 0x" << hex << output[i] << dec << endl;
      status = EXIT_FAILURE;
    }
  }
  exit(status);
}
