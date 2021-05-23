// File: circular_buffer.hpp
// Author: Jacob Guenther
// Date Created: 15 May 2021
// License: AGPLv3

#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <array>

template<class T, size_t sz>
class CircularBuffer {
public:
	CircularBuffer() :
		_head{0},
		_tail{0},
		_is_full{false}
	{}
	~CircularBuffer()=default;

	CircularBuffer(const CircularBuffer&)=delete;
	CircularBuffer(const CircularBuffer&&)=delete;
	CircularBuffer& operator=(const CircularBuffer&)=delete;
	CircularBuffer& operator=(const CircularBuffer&&)=delete;

	void push(T item) {
		_buffer[_head] = item;
		_head = (_head + 1) % sz;
		if (_is_full) {
			_tail = (_tail + 1) % sz;
		}
		_is_full = _head == _tail;
	}
	std::array<T, sz> data() {
		return _buffer;
	}
	bool is_empty() const {
		return _tail == _head;
	}
	size_t capacity() const {
		return sz;
	}
	size_t size() const {
		if (_is_full) {
			return sz;
		} else if (_head >= _tail) {
			return _head - _tail;
		} else {
			return sz + _head - _tail;
		}
	}
private:
	std::array<T, sz> _buffer;
	size_t _head;
	size_t _tail;
	bool _is_full;
};

#endif
