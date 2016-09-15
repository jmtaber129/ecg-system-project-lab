#ifndef ECG_SYSTEM_PROJECT_LAB_UART_QUEUE_H_
#define ECG_SYSTEM_PROJECT_LAB_UART_QUEUE_H_

// A queue of c-strings. Used to queue up strings to be transmitted via UART.
// NOTE: Because the implementation of the queue uses a buffer of a finite
// size, the user must check that the queue is not full prior to attempting a
// Push() operation.
class UartQueue {
  public:
    UartQueue() :
      front_index_(0),
      back_index_(0) {}

    // Returns true if the queue is empty.
    bool Empty();

    // Returns true if the queue is full.
    bool Full();

    // Returns the size of the queue based on the front and back indices.
    int Size();

    // Returns the first element in the queue.
    char Front();

    // Copies and inserts the string 'data' at the end of the queue.
    void Push(const char* data);

    void Push(char data);

    // Removes the first element in the queue.
    void Pop();

  private:
    // The size of the circular buffer.  Because the data being pushed is
    // streamed and popped almost immediately, this doesn't need to be too
    // large.
    // TODO(jmtaber129): Make this bigger for phase 2.
    static const unsigned int kBufferSize = 100;

    // Used to store c-strings in the queue.
    char circular_buffer_[kBufferSize];

    // The index of the front of the queue.
    unsigned int front_index_;

    // The index after the last element in the queue (e.g. if the last element
    // in the queue is at index 5, this field will be 6).
    unsigned int back_index_;
};

#endif // ECG_SYSTEM_PROJECT_LAB_UART_QUEUE_H_
