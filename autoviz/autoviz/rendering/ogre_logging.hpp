/******************************************************************************
 * Adapted from rviz_rendering (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_logging.hpp
 * @brief Ogre log routing macros and @c Ogre::LogManager configuration.
 *
 * Provides @c AUTOVIZ_OGRE_LOG_* macros that forward to optional handlers, plus
 * @ref OgreOgreLogging to configure file / stdout logging before Root creation
 * (mirrors rviz_rendering::OgreLogging).
 *
 * @see RenderSystem
 * @see setOgreLoggingHandlers()
 */

#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>

namespace autoviz {
namespace rendering {

/**
 * @brief Callback type for a single Ogre log line.
 *
 * @param message Log text.
 * @param file_name Source file from the macro.
 * @param line_number Source line from the macro.
 */
using OgreLoggingHandler = std::function<void(const std::string& message,
                                              const std::string& file_name,
                                              size_t line_number)>;

/**
 * @brief Installs process-wide handlers for Autoviz Ogre log macros.
 *
 * @param debug_handler Debug-level sink (may be empty).
 * @param info_handler Info-level sink.
 * @param warning_handler Warning-level sink.
 * @param error_handler Error-level sink.
 */
void setOgreLoggingHandlers(OgreLoggingHandler debug_handler,
                            OgreLoggingHandler info_handler,
                            OgreLoggingHandler warning_handler,
                            OgreLoggingHandler error_handler);

/**
 * @brief Forwards a debug message to the registered handler.
 * @param message Log text.
 * @param file_name Source file.
 * @param line_number Source line.
 */
void ogreLogDebug(const std::string& message, const std::string& file_name,
                  size_t line_number);

/**
 * @brief Forwards an info message to the registered handler.
 * @param message Log text.
 * @param file_name Source file.
 * @param line_number Source line.
 */
void ogreLogInfo(const std::string& message, const std::string& file_name,
                 size_t line_number);

/**
 * @brief Forwards a warning message to the registered handler.
 * @param message Log text.
 * @param file_name Source file.
 * @param line_number Source line.
 */
void ogreLogWarning(const std::string& message, const std::string& file_name,
                    size_t line_number);

/**
 * @brief Forwards an error message to the registered handler.
 * @param message Log text.
 * @param file_name Source file.
 * @param line_number Source line.
 */
void ogreLogError(const std::string& message, const std::string& file_name,
                  size_t line_number);

/**
 * @def AUTOVIZ_OGRE_LOG_DEBUG
 * @brief Logs a debug string with file/line via @ref ogreLogDebug().
 */
#define AUTOVIZ_OGRE_LOG_DEBUG(msg) \
  do { autoviz::rendering::ogreLogDebug(msg, __FILE__, __LINE__); } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_INFO
 * @brief Logs an info string with file/line via @ref ogreLogInfo().
 */
#define AUTOVIZ_OGRE_LOG_INFO(msg) \
  do { autoviz::rendering::ogreLogInfo(msg, __FILE__, __LINE__); } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_WARNING
 * @brief Logs a warning string with file/line via @ref ogreLogWarning().
 */
#define AUTOVIZ_OGRE_LOG_WARNING(msg) \
  do { autoviz::rendering::ogreLogWarning(msg, __FILE__, __LINE__); } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_ERROR
 * @brief Logs an error string with file/line via @ref ogreLogError().
 */
#define AUTOVIZ_OGRE_LOG_ERROR(msg) \
  do { autoviz::rendering::ogreLogError(msg, __FILE__, __LINE__); } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_DEBUG_STREAM
 * @brief Stream-style debug log: @c AUTOVIZ_OGRE_LOG_DEBUG_STREAM("a" << x).
 */
#define AUTOVIZ_OGRE_LOG_DEBUG_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogDebug(__ss.str(), __FILE__, __LINE__); \
  } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_INFO_STREAM
 * @brief Stream-style info log.
 */
#define AUTOVIZ_OGRE_LOG_INFO_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogInfo(__ss.str(), __FILE__, __LINE__); \
  } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_WARNING_STREAM
 * @brief Stream-style warning log.
 */
#define AUTOVIZ_OGRE_LOG_WARNING_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogWarning(__ss.str(), __FILE__, __LINE__); \
  } while (0)

/**
 * @def AUTOVIZ_OGRE_LOG_ERROR_STREAM
 * @brief Stream-style error log.
 */
#define AUTOVIZ_OGRE_LOG_ERROR_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogError(__ss.str(), __FILE__, __LINE__); \
  } while (0)

/**
 * @class OgreOgreLogging
 * @brief rviz_rendering::OgreLogging — configure @c Ogre::LogManager before Root.
 *
 * Call @ref useLogFile(), @ref useLogFileAndStandardOut(), or @ref noLog() then
 * @ref configureLogging() prior to @ref RenderSystem::ensureInitialized().
 */
class OgreOgreLogging {
 public:
  /**
   * @brief Returns the singleton instance.
   * @return Non-null logging configurator.
   */
  static OgreOgreLogging* instance();

  /**
   * @brief Prefer file-only logging.
   * @param filename Log file path (default @c "Ogre.log").
   */
  void useLogFile(const std::string& filename = "Ogre.log");

  /**
   * @brief Prefer file + standard output logging.
   * @param filename Log file path (default @c "Ogre.log").
   */
  void useLogFileAndStandardOut(const std::string& filename = "Ogre.log");

  /**
   * @brief Disable Ogre log output.
   */
  void noLog();

  /**
   * @brief Applies the selected preference to @c Ogre::LogManager.
   */
  void configureLogging();

 private:
  OgreOgreLogging();
  ~OgreOgreLogging();

  static OgreOgreLogging* instance_;

  /**
   * @enum Preference
   * @brief Desired log destination before configure.
   */
  enum Preference {
    kStandardOut,  /**< Stdout (and optionally file via other setters). */
    kFileLogging,  /**< File logging. */
    kNoLogging     /**< Disabled. */
  };

  Preference preference_ = kNoLogging;
  std::string filename_ = "Ogre.log";
  struct Private;
  std::unique_ptr<Private> data_;
};

}  // namespace rendering
}  // namespace autoviz

