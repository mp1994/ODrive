#pragma once
#include <cstring>

class CircularBuffer {
public:
    float* buf; 
	volatile uint32_t _count = 0;
    
private:
    size_t _size = 8;    // default size
	float _1over_size = (float) 1.0f/_size;

public:
	CircularBuffer() {};

    CircularBuffer(size_t s) : _size(s) {
        buf = (float*) malloc(_size * sizeof(float));
    }
    
    size_t size() {
        return _size;
    }
    
    void push(float x) {

        _count = (_count % _size);
        buf[_count] = (float) x;
        _count++;

    }
    
    float mean() {

		float avg = 0.0f;
        for( size_t i = 0; i < _size; i++ ) {
            avg += buf[i];
        }
        avg *= _1over_size; // multiply by float constant (faster than division)
        return avg;

    }
    
	float last() {
		return buf[_count];
	}

    void clear() {

        _count = 0;
        memset(buf, 0, _size);

    }

private:    
    
};
