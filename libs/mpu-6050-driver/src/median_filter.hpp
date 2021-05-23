// File: median_filter.hpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#ifndef MEDIAN_FILTER_HPP
#define MEDIAN_FILTER_HPP

#include <algorithm>

#include "circular_buffer.hpp"

template<size_t sz>
class MedianFilter {
public:
	using element_t = int16_t;
	MedianFilter()=default;
	~MedianFilter()=default;

	MedianFilter(const MedianFilter&)=delete;
	MedianFilter(const MedianFilter&&)=delete;
	MedianFilter& operator=(const MedianFilter&)=delete;
	MedianFilter& operator=(const MedianFilter&&)=delete;

	void update(element_t value) {
		_window.push(value);
	}
	element_t get_median() {
		auto data = _window.data();
		auto size = _window.size();
		std::sort(data.begin(), data.begin() + size);
		return data[size/2];
	}
private:
	CircularBuffer<element_t, sz> _window;
};

#endif
