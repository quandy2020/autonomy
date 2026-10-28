/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#ifndef ARGUS_APP_ORB_VOCABULARY_CACHE_HPP_
#define ARGUS_APP_ORB_VOCABULARY_CACHE_HPP_

#include <string>

namespace argus {
namespace app {

// Returns path unchanged for a plain vocabulary. A .tar.gz/.tgz archive is
// safely expanded into a process-independent temporary cache.
std::string PrepareOrbVocabulary(const std::string& path);

}  // namespace app
}  // namespace argus

#endif  // ARGUS_APP_ORB_VOCABULARY_CACHE_HPP_
