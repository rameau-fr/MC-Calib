/**
 * @file logger.h
 * @breif Global logger borrowed from https://github.com/gklingler/simpleLogger
 */

#ifndef simpleLogger_h__
#define simpleLogger_h__

#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/trivial.hpp>

// the logs are also written to LOGFILE
#define LOGFILE "logfile.log"

// just log messages with severity >= SEVERITY_THRESHOLD are written
#define SEVERITY_THRESHOLD logging::trivial::info

// register a global logger
BOOST_LOG_GLOBAL_LOGGER(logger, boost::log::sources::severity_logger_mt<
                                    boost::log::trivial::severity_level>)

// just a helper macro used by the macros below - don't use it in your code
#define LOG_AT_SEVERITY(severity)                                              \
  BOOST_LOG_SEV(logger::get(), boost::log::trivial::severity)

// ===== log macros =====
#define LOG_TRACE LOG_AT_SEVERITY(trace)
#define LOG_DEBUG LOG_AT_SEVERITY(debug)
#define LOG_INFO LOG_AT_SEVERITY(info)
#define LOG_WARNING LOG_AT_SEVERITY(warning)
#define LOG_ERROR LOG_AT_SEVERITY(error)
#define LOG_FATAL LOG_AT_SEVERITY(fatal)

#endif