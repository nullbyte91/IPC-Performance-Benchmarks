#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <chrono>
#include <numeric>

#define NS_TO_MS(ns) ((ns) / 1e6)

class TimeMeasurement {
public:
    std::vector<long long> time_measurements;

    template<typename T>
    auto measure_time(std::function<T()> operation) {
        auto start = std::chrono::high_resolution_clock::now();
        T result = operation();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

        //std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        time_measurements.push_back(duration.count());
        return result;
    }

    double mean_time(int num_messages) {
        if (num_messages <= 0 || num_messages > time_measurements.size()) {
            throw std::invalid_argument("Invalid number of messages for mean time calculation.");
        }

        double sum = std::accumulate(time_measurements.begin(), time_measurements.begin() + num_messages, 0.0);
        return sum / num_messages;
    }
};

// explicit specialization for T = void
template<>
auto TimeMeasurement::measure_time<void>(std::function<void()> operation) {
    auto start = std::chrono::high_resolution_clock::now();
    operation();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    //std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
    time_measurements.push_back(duration.count());
}
