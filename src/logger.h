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
#define LOG(severity)                                                          \
  BOOST_LOG_SEV(logger::get(), boost::log::trivial::severity)

// ===== log macros =====
#define LOG_TRACE LOG(trace)
#define LOG_DEBUG LOG(debug)
#define LOG_INFO LOG(info)
#define LOG_WARNING LOG(warning)
#define LOG_ERROR LOG(error)
#define LOG_FATAL LOG(fatal)

#endif