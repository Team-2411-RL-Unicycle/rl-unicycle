#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
#include <vector>
#include <algorithm>

void controlLoop() {
    using namespace std::chrono;
    auto next = high_resolution_clock::now();
    milliseconds loopTime(10); // 10 ms loop time

    // A min-heap to store the longest 100 durations
    std::priority_queue<double, std::vector<double>, std::greater<double>> minHeap;

    int n_iterations = 1000;
    for (int i = 0; i < n_iterations; ++i) {  // Run the loop x times
        auto start = high_resolution_clock::now(); // Start time of loop iteration
        next += loopTime;

        // Simulate some work (This can be commented out or removed for real applications)
        std::cout << "Loop iteration: " << i << std::endl;

        std::this_thread::sleep_until(next);

        auto end = high_resolution_clock::now(); // End time of loop iteration
        double duration = duration_cast<microseconds>(end - start).count() / 1000.0; // Duration in milliseconds

        // Keep track of the 100 worst times
        if (minHeap.size() < 100) {
            minHeap.push(duration);
        } else if (duration > minHeap.top()) {
            minHeap.pop();
            minHeap.push(duration);
        }
    }

    // Print the 100 worst durations
    std::vector<double> worstDurations;
    while (!minHeap.empty()) {
        worstDurations.push_back(minHeap.top());
        minHeap.pop();
    }

    // Since we used a min-heap, the durations are in ascending order, so we reverse it to start with the longest duration
    std::reverse(worstDurations.begin(), worstDurations.end());

    std::cout << "Worst durations out of " << n_iterations << " iterations" << std::endl;
    for (size_t i = 0; i < worstDurations.size(); ++i) {
        std::cout << "Worst duration " << i + 1 << ": " << worstDurations[i] << " ms" << std::endl;
    }
}

int main() {
    std::thread controlThread(controlLoop);
    controlThread.join();  // Wait for the controlThread to finish
    return 0;
}
