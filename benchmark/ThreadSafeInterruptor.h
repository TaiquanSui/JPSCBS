#ifndef THREAD_SAFE_INTERRUPTOR_H
#define THREAD_SAFE_INTERRUPTOR_H

#include <atomic>
#include <condition_variable>
#include <mutex>

class ThreadSafeInterruptor {
public:
    void interrupt() { interrupted.store(true); }
    bool is_interrupted() const { return interrupted.load(); }
    void reset() { interrupted.store(false); }

private:
    std::atomic<bool> interrupted{false};
};

#endif // THREAD_SAFE_INTERRUPTOR_H 