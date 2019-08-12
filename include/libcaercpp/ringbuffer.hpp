#ifndef LIBCAER_RINGBUFFER_HPP_
#define LIBCAER_RINGBUFFER_HPP_

#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <vector>

// Alignment specification support (with defines for cache line alignment).
#if !defined(CACHELINE_SIZE)
#	define CACHELINE_SIZE 128 // Default (big enough for most processors), must be power of two!
#endif

namespace libcaer {
namespace ringbuffer {

template<typename T> class RingBuffer {
private:
	alignas(CACHELINE_SIZE) std::atomic_size_t putPos;
	alignas(CACHELINE_SIZE) std::atomic_size_t getPos;
	alignas(CACHELINE_SIZE) std::vector<std::atomic<T>> elements;
	const size_t sizeAdj;
	const T placeholder;

public:
	RingBuffer(size_t sz) : putPos(0), getPos(0), elements(sz), sizeAdj(sz - 1), placeholder() {
		// Force multiple of two size for performance.
		if ((sz == 0) || ((sz & sizeAdj) != 0)) {
			throw std::invalid_argument("Size must be a power of two.");
		}

		std::atomic_thread_fence(std::memory_order_release);
	}

	bool operator==(const RingBuffer &rhs) const noexcept {
		return (this == &rhs);
	}

	bool operator!=(const RingBuffer &rhs) const noexcept {
		return (!operator==(rhs));
	}

	void put(const T &elem) {
		if (elem == placeholder) {
			// Default constructed elements are disallowed (used as place-holders).
			throw std::invalid_argument("Default constructed elements are not allowed in the ringbuffer.");
		}

		const size_t putPosLocal = putPos.load(std::memory_order_acquire);

		const T curr = elements[putPosLocal].load(std::memory_order_acquire);

		// If the place where we want to put the new element is NULL, it's still
		// free and we can use it.
		if (curr == placeholder) {
			elements[putPosLocal].store(elem, std::memory_order_release);

			// Increase local put pointer.
			putPos.store(((putPosLocal + 1) & sizeAdj), std::memory_order_release);

			return;
		}

		// Else, buffer is full.
		throw std::out_of_range("Ringbuffer full.");
	}

	bool full() const noexcept {
		const T curr = elements[putPos.load(std::memory_order_acquire)].load(std::memory_order_acquire);

		// If the place where we want to put the new element is NULL, it's still
		// free and thus the buffer still has available space.
		if (curr == placeholder) {
			return (false);
		}

		// Else, buffer is full.
		return (true);
	}

	T get() {
		const size_t getPosLocal = getPos.load(std::memory_order_acquire);

		T curr = elements[getPosLocal].load(std::memory_order_acquire);

		// If the place where we want to get an element from is not NULL, there
		// is valid content there, which we return, and reset the place to NULL.
		if (curr != placeholder) {
			elements[getPosLocal].store(placeholder, std::memory_order_release);

			// Increase local get pointer.
			getPos.store(((getPosLocal + 1) & sizeAdj), std::memory_order_release);

			return (curr);
		}

		// Else, buffer is empty.
		throw std::out_of_range("Ringbuffer empty.");
	}

	T look() const {
		T curr = elements[getPos.load(std::memory_order_acquire)].load(std::memory_order_acquire);

		// If the place where we want to get an element from is not NULL, there
		// is valid content there, which we return, without removing it from the
		// ring buffer.
		if (curr != placeholder) {
			return (curr);
		}

		// Else, buffer is empty.
		throw std::out_of_range("Ringbuffer empty.");
	}

	bool empty() const noexcept {
		const T curr = elements[getPos.load(std::memory_order_acquire)].load(std::memory_order_acquire);

		// If the place where we want to get an element from is not NULL, there
		// is valid content there, which we return, without removing it from the
		// ring buffer.
		if (curr != placeholder) {
			return (false);
		}

		// Else, buffer is empty.
		return (true);
	}

	size_t capacity() const noexcept {
		return (elements.size());
	}

	/**
	 * Get approximate number of elements.
	 * Can be lagging behind or be imprecise when close to full/empty.
	 *
	 * @return approximate number of elements
	 */
	size_t size() const noexcept {
		const size_t putPosLocal = putPos.load(std::memory_order_acquire);
		const size_t getPosLocal = getPos.load(std::memory_order_acquire);

		// Standard way to get ringbuffer size.
		const size_t dist = ((putPosLocal - getPosLocal) & sizeAdj);

		// Since this is a concurrent ringbuffer, there are corner cases
		// where the reported size is incorrect, as it is possible for
		// one index to overtake the other. We try to disambiguate those
		// cases here, but cannot guarantee exact size always.
		if (dist <= 1 || dist >= sizeAdj) {
			if (elements[getPosLocal].load(std::memory_order_acquire) == placeholder) {
				return (0); // Empty or close to.
			}
			else {
				return (elements.size()); // Full or close to.
			}
		}

		return (dist);
	}
};

} // namespace ringbuffer
} // namespace libcaer

#endif /* LIBCAER_RINGBUFFER_HPP_ */
