/******************************************************************************
 * Adapted from rviz_rendering (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <functional>
#include <memory>
#include <sstream>
#include <string>

namespace autoviz {
namespace rendering {

using OgreLoggingHandler = std::function<void(const std::string& message,
                                              const std::string& file_name,
                                              size_t line_number)>;

void setOgreLoggingHandlers(OgreLoggingHandler debug_handler,
                            OgreLoggingHandler info_handler,
                            OgreLoggingHandler warning_handler,
                            OgreLoggingHandler error_handler);

void ogreLogDebug(const std::string& message, const std::string& file_name,
                  size_t line_number);
void ogreLogInfo(const std::string& message, const std::string& file_name,
                 size_t line_number);
void ogreLogWarning(const std::string& message, const std::string& file_name,
                    size_t line_number);
void ogreLogError(const std::string& message, const std::string& file_name,
                  size_t line_number);

#define AUTOVIZ_OGRE_LOG_DEBUG(msg) \
  do { autoviz::rendering::ogreLogDebug(msg, __FILE__, __LINE__); } while (0)
#define AUTOVIZ_OGRE_LOG_INFO(msg) \
  do { autoviz::rendering::ogreLogInfo(msg, __FILE__, __LINE__); } while (0)
#define AUTOVIZ_OGRE_LOG_WARNING(msg) \
  do { autoviz::rendering::ogreLogWarning(msg, __FILE__, __LINE__); } while (0)
#define AUTOVIZ_OGRE_LOG_ERROR(msg) \
  do { autoviz::rendering::ogreLogError(msg, __FILE__, __LINE__); } while (0)

#define AUTOVIZ_OGRE_LOG_DEBUG_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogDebug(__ss.str(), __FILE__, __LINE__); \
  } while (0)
#define AUTOVIZ_OGRE_LOG_INFO_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogInfo(__ss.str(), __FILE__, __LINE__); \
  } while (0)
#define AUTOVIZ_OGRE_LOG_WARNING_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogWarning(__ss.str(), __FILE__, __LINE__); \
  } while (0)
#define AUTOVIZ_OGRE_LOG_ERROR_STREAM(args) \
  do { \
    std::stringstream __ss; \
    __ss << args; \
    autoviz::rendering::ogreLogError(__ss.str(), __FILE__, __LINE__); \
  } while (0)

/** rviz_rendering::OgreLogging — configure Ogre::LogManager before Root. */
class OgreOgreLogging {
 public:
  static OgreOgreLogging* instance();

  void useLogFile(const std::string& filename = "Ogre.log");
  void useLogFileAndStandardOut(const std::string& filename = "Ogre.log");
  void noLog();
  void configureLogging();

 private:
  OgreOgreLogging();
  ~OgreOgreLogging();

  static OgreOgreLogging* instance_;
  enum Preference { kStandardOut, kFileLogging, kNoLogging };
  Preference preference_ = kNoLogging;
  std::string filename_ = "Ogre.log";
  struct Private;
  std::unique_ptr<Private> data_;
};

}  // namespace rendering
}  // namespace autoviz

#endif
