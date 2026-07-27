/******************************************************************************
 * Adapted from rviz_rendering (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/ogre_logging.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <cstdio>
#include <mutex>

#include <OgreLog.h>
#include <OgreLogManager.h>

namespace autoviz {
namespace rendering {
namespace {

std::mutex g_logging_mutex;
OgreLoggingHandler g_debug_handler = [](const std::string& message,
                                        const std::string& file_name,
                                        size_t line_number) {
  printf("[autoviz_ogre:debug] %s, at %s:%zu\n", message.c_str(), file_name.c_str(),
         line_number);
};
OgreLoggingHandler g_info_handler = [](const std::string& message,
                                       const std::string& file_name,
                                       size_t line_number) {
  printf("[autoviz_ogre:info] %s, at %s:%zu\n", message.c_str(), file_name.c_str(),
         line_number);
};
OgreLoggingHandler g_warning_handler = [](const std::string& message,
                                          const std::string& file_name,
                                          size_t line_number) {
  fprintf(stderr, "[autoviz_ogre:warning] %s, at %s:%zu\n", message.c_str(),
          file_name.c_str(), line_number);
};
OgreLoggingHandler g_error_handler = [](const std::string& message,
                                        const std::string& file_name,
                                        size_t line_number) {
  fprintf(stderr, "[autoviz_ogre:error] %s, at %s:%zu\n", message.c_str(),
          file_name.c_str(), line_number);
};

class CustomOgreLogListener : public Ogre::LogListener {
 public:
  CustomOgreLogListener() : min_lml_(Ogre::LML_CRITICAL) {}
  void messageLogged(const Ogre::String& message, Ogre::LogMessageLevel lml,
                     bool maskDebug, const Ogre::String& logName,
                     bool& skipThisMessage) override {
    (void)maskDebug;
    (void)logName;
    if (skipThisMessage || lml < min_lml_) {
      return;
    }
    switch (lml) {
      case Ogre::LogMessageLevel::LML_TRIVIAL:
        AUTOVIZ_OGRE_LOG_DEBUG(message.c_str());
        break;
      case Ogre::LogMessageLevel::LML_NORMAL:
        AUTOVIZ_OGRE_LOG_INFO(message.c_str());
        break;
      case Ogre::LogMessageLevel::LML_WARNING:
        AUTOVIZ_OGRE_LOG_WARNING(message.c_str());
        break;
      case Ogre::LogMessageLevel::LML_CRITICAL:
        AUTOVIZ_OGRE_LOG_ERROR(message.c_str());
        break;
      default:
        AUTOVIZ_OGRE_LOG_ERROR_STREAM("unknown Ogre log message level: " << lml);
    }
  }
  Ogre::LogMessageLevel min_lml_;
};

}  // namespace

void setOgreLoggingHandlers(OgreLoggingHandler debug_handler,
                            OgreLoggingHandler info_handler,
                            OgreLoggingHandler warning_handler,
                            OgreLoggingHandler error_handler) {
  std::lock_guard<std::mutex> lock(g_logging_mutex);
  g_debug_handler = std::move(debug_handler);
  g_info_handler = std::move(info_handler);
  g_warning_handler = std::move(warning_handler);
  g_error_handler = std::move(error_handler);
}

void ogreLogDebug(const std::string& message, const std::string& file_name,
                  size_t line_number) {
  std::lock_guard<std::mutex> lock(g_logging_mutex);
  g_debug_handler(message, file_name, line_number);
}

void ogreLogInfo(const std::string& message, const std::string& file_name,
                 size_t line_number) {
  std::lock_guard<std::mutex> lock(g_logging_mutex);
  g_info_handler(message, file_name, line_number);
}

void ogreLogWarning(const std::string& message, const std::string& file_name,
                    size_t line_number) {
  std::lock_guard<std::mutex> lock(g_logging_mutex);
  g_warning_handler(message, file_name, line_number);
}

void ogreLogError(const std::string& message, const std::string& file_name,
                  size_t line_number) {
  std::lock_guard<std::mutex> lock(g_logging_mutex);
  g_error_handler(message, file_name, line_number);
}

struct OgreOgreLogging::Private {
  CustomOgreLogListener listener;
};

OgreOgreLogging* OgreOgreLogging::instance_ = nullptr;

OgreOgreLogging* OgreOgreLogging::instance() {
  if (instance_ == nullptr) {
    instance_ = new OgreOgreLogging();
  }
  return instance_;
}

OgreOgreLogging::OgreOgreLogging() : data_(std::make_unique<Private>()) {}

OgreOgreLogging::~OgreOgreLogging() {
  if (Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr()) {
    delete log_manager;
  }
  instance_ = nullptr;
}

void OgreOgreLogging::useLogFile(const std::string& filename) {
  preference_ = kFileLogging;
  filename_ = filename;
}

void OgreOgreLogging::useLogFileAndStandardOut(const std::string& filename) {
  preference_ = kStandardOut;
  filename_ = filename;
}

void OgreOgreLogging::noLog() { preference_ = kNoLogging; }

void OgreOgreLogging::configureLogging() {
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (log_manager == nullptr) {
    log_manager = new Ogre::LogManager();
  }
  Ogre::Log* log = log_manager->createLog(
      filename_, false, false, preference_ == kNoLogging);
  log->addListener(&data_->listener);
  if (preference_ == kStandardOut) {
    data_->listener.min_lml_ = Ogre::LML_NORMAL;
  }
}

}  // namespace rendering
}  // namespace autoviz

#endif
