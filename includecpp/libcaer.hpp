#ifndef LIBCAER_HPP_
#define LIBCAER_HPP_

#include <libcaer/libcaer.h>
#include <type_traits>

namespace libcaer {
namespace log {
	enum class logLevel {
		EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7
	};

	void logLevelSet(logLevel l) {
		caerLogLevelSet(static_cast<typename std::underlying_type<logLevel>::type>(l));
	}

	logLevel logLevelGet() {
		return (static_cast<logLevel>(caerLogLevelGet()));
	}

	void fileDescriptorsSet(int fd1, int fd2) {
		caerLogFileDescriptorsSet(fd1, fd2);
	}

	void log(logLevel l, const char *subSystem, const char *format, ...) {
		va_list argumentList;
		va_start(argumentList, format);
		caerLogVA(static_cast<typename std::underlying_type<logLevel>::type>(l), subSystem, format, argumentList);
		va_end(argumentList);
	}

	void logVA(logLevel l, const char *subSystem, const char *format, va_list args) {
		caerLogVA(static_cast<typename std::underlying_type<logLevel>::type>(l), subSystem, format, args);
	}
}
}

#endif /* LIBCAER_HPP_ */
