// Throttler.cpp

#include "Throttler.h"
#include "Utils.h"

Throttler::Throttler() {}

void Throttler::executeTask(std::string taskName, std::chrono::duration<double> interval, std::function<void()> task) {
	auto lastExecuted = lastExecuted_[taskName];
	auto currentTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastExecuted);

	if (elapsedTime >= interval) {
		lastExecuted_[taskName] = currentTime;
		task();
	}
}

void Throttler::executeTaskWithSleep(std::string taskName, std::chrono::microseconds interval, std::function<void()> task) {
	if (lastExecuted__.find(taskName) == lastExecuted__.end()) {
		LARGE_INTEGER currentTime;
		QueryPerformanceCounter(&currentTime);
		lastExecuted__[taskName] = currentTime;
	}

	auto lastExecuted = lastExecuted__[taskName];

	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);

	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);

	auto elapsedMicroseconds = std::chrono::microseconds((currentTime.QuadPart - lastExecuted.QuadPart) * std::micro::den / frequency.QuadPart);

	if (elapsedMicroseconds < interval) {
		auto sleepMicroseconds = interval - elapsedMicroseconds;
		auto sleepTimePoint = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() + sleepMicroseconds);
		std::this_thread::sleep_until(sleepTimePoint);
	}

	QueryPerformanceCounter(&currentTime);
	lastExecuted__[taskName] = currentTime;
	task();
}
