#ifndef C11THREADS_POSIX_H_
#define C11THREADS_POSIX_H_

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#if defined(__WINDOWS__)
#	ifndef WIN32_LEAN_AND_MEAN
#		define WIN32_LEAN_AND_MEAN
#	endif
#	include <windows.h>
#	include <synchapi.h>

typedef void *thrd_t;
typedef HANDLE mtx_t;

#else
#	define _DARWIN_C_SOURCE 1
#	include <pthread.h>
#	undef _DARWIN_C_SOURCE

#	include <sched.h>
#	include <sys/resource.h>
#	include <sys/time.h>

typedef pthread_t thrd_t;
typedef pthread_mutex_t mtx_t;
#endif

#if !defined(__WINDOWS__) && !defined(__APPLE__)
#	include <sys/prctl.h>
#	include <unistd.h>
#endif

typedef int (*thrd_start_t)(void *);

enum {
	thrd_success  = 0,
	thrd_error    = 1,
	thrd_nomem    = 2,
	thrd_timedout = 3,
	thrd_busy     = 4,
};

enum {
	mtx_plain     = 0,
	mtx_timed     = 1,
	mtx_recursive = 2,
};

#define MAX_THREAD_NAME_LENGTH 15

static inline int thrd_create(thrd_t *thr, thrd_start_t func, void *arg) {
#if defined(__WINDOWS__)
	*thr = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) func, arg, 0, NULL);
	return (thrd_success);
#else
	// This is fine on most architectures.
#	pragma GCC diagnostic push
#	pragma GCC diagnostic ignored "-Wcast-function-type"
	int ret = pthread_create(thr, NULL, (void *(*) (void *) ) func, arg);
#	pragma GCC diagnostic pop

	switch (ret) {
		case 0:
			return (thrd_success);

		case EAGAIN:
			return (thrd_nomem);

		default:
			return (thrd_error);
	}
#endif
}

static inline int thrd_join(thrd_t thr, int *res) {
#if defined(__WINDOWS__)
	WaitForSingleObject(thr, INFINITE);

	if (res != NULL) {
		*res = 0;
	}
#else
	void *pthread_res;

	if (pthread_join(thr, &pthread_res) != 0) {
		return (thrd_error);
	}

	if (res != NULL) {
		*res = (int) (intptr_t) pthread_res;
	}
#endif
	return (thrd_success);
}

static inline int thrd_sleep(const int64_t usec) {
	if (usec == 0) {
		return (0);
	}

#if defined(__WINDOWS__)
	HANDLE timer;
	LARGE_INTEGER ft = {0};

	ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
	return (0);
#else
	struct timespec time_point = {.tv_sec = usec / 1000000LL, .tv_nsec = (usec % 1000000LL) * 1000LL};
	if (nanosleep(&time_point, NULL) == 0) {
		return (0); // Successful sleep.
	}

	if (errno == EINTR) {
		return (-1); // C11: a signal occurred.
	}

	return (-2); // C11: other negative value if an error occurred.
#endif
}

static inline void thrd_yield(void) {
	// windows doesn't have an alternative (at least I don't know one)
#if !defined(__WINDOWS__)
	sched_yield();
#endif
}

static inline int mtx_init(mtx_t *mutex, int type) {
#if defined(__WINDOWS__)
	*mutex = CreateMutex(NULL, FALSE, NULL);

	if (*mutex == NULL) {
		return (thrd_error);
	}
#else
	pthread_mutexattr_t attr;
	if (pthread_mutexattr_init(&attr) != 0) {
		return (thrd_error);
	}

	// TIMED and PLAIN
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);

	if (type & 0x02) {
		// RECURSIVE
		pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	}

	if (pthread_mutex_init(mutex, &attr) != 0) {
		pthread_mutexattr_destroy(&attr);
		return (thrd_error);
	}

	pthread_mutexattr_destroy(&attr);
#endif
	return (thrd_success);
}

static inline void mtx_destroy(mtx_t *mutex) {
#if defined(__WINDOWS__)
	CloseHandle(*mutex);
#else
	pthread_mutex_destroy(mutex);
#endif
}

static inline int mtx_lock(mtx_t *mutex) {
#if defined(__WINDOWS__)
	if (WaitForSingleObject(*mutex, INFINITE) == WAIT_ABANDONED) {
		return (thrd_error);
	}
#else
	if (pthread_mutex_lock(mutex) != 0) {
		return (thrd_error);
	}
#endif
	return (thrd_success);
}

static inline int mtx_unlock(mtx_t *mutex) {
#if defined(__WINDOWS__)
	if (!ReleaseMutex(*mutex)) {
		return (thrd_error);
	}
#else
	if (pthread_mutex_unlock(mutex) != 0) {
		return (thrd_error);
	}
#endif
	return (thrd_success);
}

// NON STANDARD!
static inline int thrd_set_name(const char *name) {
#if defined(__linux__)
	if (prctl(PR_SET_NAME, name) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#elif defined(__APPLE__)
	if (pthread_setname_np(name) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#else
	(void) (name); // UNUSED.

	return (thrd_error);
#endif
}

// NON STANDARD!
static inline int thrd_get_name(char *name, size_t maxNameLength) {
#if defined(__linux__)
	(void) (maxNameLength); // UNUSED ON LINUX.

	if (prctl(PR_GET_NAME, name) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#elif defined(__APPLE__)
	if (pthread_getname_np(pthread_self(), name, maxNameLength) != 0) {
		return (thrd_error);
	}

	return (thrd_success);
#else
	(void) (name);          // UNUSED.
	(void) (maxNameLength); // UNUSED.

	return (thrd_error);
#endif
}

#endif /* C11THREADS_POSIX_H_ */
