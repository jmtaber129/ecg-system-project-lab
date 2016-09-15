#include "uart-queue.h"

#include <stdlib.h>
#include <string.h>

bool UartQueue::Empty() {
  bool return_val = (front_index_ == back_index_);
  return return_val;
}

bool UartQueue::Full() {
  return this->Size() == kBufferSize - 1;
}

int UartQueue::Size() {
  return (kBufferSize + front_index_ - back_index_) % kBufferSize;
}

char UartQueue::Front() {
  return circular_buffer_[front_index_];
}

void UartQueue::Push(const char* data) {
  int length = strlen(data);
  for (int i = 0; i < length; ++i) {
    Push(data[i]);
  }
}

void UartQueue::Push(char data) {
  circular_buffer_[back_index_] = data;
  if (++back_index_ >= kBufferSize) {
    back_index_ -= kBufferSize;
  }
}

void UartQueue::Pop() {
  circular_buffer_[front_index_] = '\0';
  ++front_index_;
  if (front_index_ >= kBufferSize) {
    front_index_ -= kBufferSize;
  }
}
