#ifndef PORTABLE_TIME_H_
#define PORTABLE_TIME_H_

#if defined(__APPLE__)
#	include <mach/clock.h>
#	include <mach/clock_types.h>
#	include <mach/mach.h>
#	include <mach/mach_host.h>
#	include <mach/mach_port.h>
#	include <mach/mach_time.h>
#	include <stdbool.h>
#	include <sys/time.h>
#	include <time.h>

static inline bool portable_clock_gettime_monotonic(struct timespec *monoTime) {
	kern_return_t kRet;
	clock_serv_t clockRef;
	mach_timespec_t machTime;

	mach_port_t host = mach_host_self();

	kRet = host_get_clock_service(host, SYSTEM_CLOCK, &clockRef);
	mach_port_deallocate(mach_task_self(), host);

	if (kRet != KERN_SUCCESS) {
		errno = EINVAL;
		return (false);
	}

	kRet = clock_get_time(clockRef, &machTime);
	mach_port_deallocate(mach_task_self(), clockRef);

	if (kRet != KERN_SUCCESS) {
		errno = EINVAL;
		return (false);
	}

	monoTime->tv_sec  = machTime.tv_sec;
	monoTime->tv_nsec = machTime.tv_nsec;

	return (true);
}

static inline bool portable_clock_gettime_realtime(struct timespec *realTime) {
	kern_return_t kRet;
	clock_serv_t clockRef;
	mach_timespec_t machTime;

	mach_port_t host = mach_host_self();

	kRet = host_get_clock_service(host, CALENDAR_CLOCK, &clockRef);
	mach_port_deallocate(mach_task_self(), host);

	if (kRet != KERN_SUCCESS) {
		errno = EINVAL;
		return (false);
	}

	kRet = clock_get_time(clockRef, &machTime);
	mach_port_deallocate(mach_task_self(), clockRef);

	if (kRet != KERN_SUCCESS) {
		errno = EINVAL;
		return (false);
	}

	realTime->tv_sec  = machTime.tv_sec;
	realTime->tv_nsec = machTime.tv_nsec;

	return (true);
}
#elif ((defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L) || (defined(_XOPEN_SOURCE) && _XOPEN_SOURCE >= 600) \
	   || (defined(_WIN32) && defined(__MINGW32__)))
#	include <stdbool.h>
#	include <time.h>

static inline bool portable_clock_gettime_monotonic(struct timespec *monoTime) {
	return (clock_gettime(CLOCK_MONOTONIC, monoTime) == 0);
}

static inline bool portable_clock_gettime_realtime(struct timespec *realTime) {
	return (clock_gettime(CLOCK_REALTIME, realTime) == 0);
}
#elif defined(__WINDOWS__)
#	include <stdint.h>
#	include <time.h>

static inline int clock_gettime(struct timespec *spec) {
	int64_t wintime;
	GetSystemTimeAsFileTime((FILETIME *) &wintime);
	wintime -= 116444736000000000i64;            // 1jan1601 to 1jan1970
	spec->tv_sec  = wintime / 10000000i64;       // seconds
	spec->tv_nsec = wintime % 10000000i64 * 100; // nano-seconds
	return 0;
}

static inline bool portable_clock_gettime_monotonic(struct timespec *monoTime) {
	return (clock_gettime(monoTime) == 0);
}

static inline bool portable_clock_gettime_realtime(struct timespec *realTime) {
	return (clock_gettime(realTime) == 0);
}

#else
#	error "No portable way of getting absolute monotonic time found."
#endif

#endif /* PORTABLE_TIME_H_ */
