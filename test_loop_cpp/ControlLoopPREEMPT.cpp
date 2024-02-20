#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <queue>
#include <pthread.h>
#include <cstring>  // Include for strerror
#include <cerrno>   // Include for errno

void setThreadPriority() {
    sched_param sch;
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &sch);
    sch.sched_priority = 20; // Set the priority high, but not maximum to avoid starving other threads
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch)) {
        std::cout << "Failed to set Thread scheduling : " << strerror(errno) << std::endl; // Corrected usage of strerror
    }
}

void controlLoop() {
    setThreadPriority(); // Set high priority for this thread

    using namespace std::chrono;
    auto next = high_resolution_clock::now();
    milliseconds loopTime(10); // 10 ms loop time

    std::priority_queue<double, std::vector<double>, std::greater<double>> minHeap;

    int n_iterations = 100000;
    for (int i = 0; i < n_iterations; ++i) {
        auto start = high_resolution_clock::now();
        next += loopTime;

        // Simulate work here

        while (high_resolution_clock::now() < next) {
            // Active waiting (busy loop) to reduce latency
            std::this_thread::yield(); // Yield to other threads/processes if needed
        }

        auto end = high_resolution_clock::now();
        double duration = duration_cast<microseconds>(end - start).count() / 1000.0;

        if (minHeap.size() < 100) {
            minHeap.push(duration);
        } else if (duration > minHeap.top()) {
            minHeap.pop();
            minHeap.push(duration);
        }
    }

    std::vector<double> worstDurations;
    while (!minHeap.empty()) {
        worstDurations.push_back(minHeap.top());
        minHeap.pop();
    }

    std::reverse(worstDurations.begin(), worstDurations.end());


    std::cout << "Worst durations out of " << n_iterations << " iterations" << std::endl;
    for (size_t i = 0; i < worstDurations.size(); ++i) {
        std::cout << "Worst duration " << i + 1 << ": " << worstDurations[i] << " ms" << std::endl;
    }
}

int main() {
    std::thread controlThread(controlLoop);
    controlThread.join();
    return 0;
}
