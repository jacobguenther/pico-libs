// File: median_filter.hpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#ifndef MEDIAN_FILTER_HPP
#define MEDIAN_FILTER_HPP

#include <algorithm> // std::partial_sort
#include <cstring>   // memcpy

#include "pico/util/queue.h"


/*
Invariants: sz is odd
*/
template<size_t sz>
class MedianFilter {
public:
	using element_t = int16_t;
	MedianFilter()
		: _data{}
	{
		queue_init(&_window, sizeof(element_t), sz);
		assert(sz % 2 != 0);
	};
	~MedianFilter() {
		queue_free(&_window);
	};

	MedianFilter(const MedianFilter&)=delete;
	MedianFilter(const MedianFilter&&)=delete;
	MedianFilter& operator=(const MedianFilter&)=delete;
	MedianFilter& operator=(const MedianFilter&&)=delete;

	void update(element_t value) {
		if (queue_is_full(&_window)) {
			queue_try_remove(&_window, nullptr);
		}
		queue_try_add(&_window, &value);
	}
	element_t get_median() {
		// number of elements in queue [0, sz)
		const uint32_t size{queue_get_level(&_window)};
		// copy queue data into array of element_t
		memcpy(&_data, _window.data, _data_size_bytes);
		// middle point for median
		const uint32_t middle{size/2};
		// puts the median at middle
		std::nth_element(_data.begin(), _data.begin() + middle, _data.begin() + size);
		return _data[middle];
	}
private:
	queue_t _window;
	std::array<element_t, sz> _data;
	uint32_t _data_size_bytes{sizeof(element_t) * sz};
};

#endif
