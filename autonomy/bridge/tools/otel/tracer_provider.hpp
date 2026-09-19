/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file tracer_provider.hpp
 * @brief OpenTelemetry tracer provider interface + noop default.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace tools {
namespace otel {

/**
 * @brief Minimal span handle (no-op capable).
 */
class Span
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Span)
    virtual ~Span() = default;
    virtual void SetStatusOk() {}
    virtual void SetStatusError(const std::string& /*message*/) {}
};

/**
 * @brief Tracer provider abstraction.
 */
class TracerProvider
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TracerProvider)
    virtual ~TracerProvider() = default;

    virtual std::unique_ptr<Span> StartSpan(const std::string& /*name*/) {
        return std::make_unique<Span>();
    }
};

/**
 * @brief Always-no-op provider (default).
 */
class NoopTracerProvider : public TracerProvider
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(NoopTracerProvider)
};

}  // namespace otel
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
