/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <thread>
#include <utility>

#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/data/data_visitor.hpp"
#include "autolink/event/perf_event_cache.hpp"

namespace autolink {
namespace croutine {

class RoutineFactory
{
public:
    using VoidFunc = std::function<void()>;
    using CreateRoutineFunc = std::function<VoidFunc()>;
    // We can use routine_func directly.
    CreateRoutineFunc create_routine;
    inline std::shared_ptr<data::DataVisitorBase> GetDataVisitor() const {
        return data_visitor_;
    }
    inline void SetDataVisitor(
        const std::shared_ptr<data::DataVisitorBase>& dv) {
        data_visitor_ = dv;
    }

private:
    std::shared_ptr<data::DataVisitorBase> data_visitor_ = nullptr;
};

template <typename M0, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0>>& dv) {
    RoutineFactory factory;
    factory.SetDataVisitor(dv);
    // Copy the function object to ensure it's properly stored
    std::decay_t<F> func = std::forward<F>(f);
    factory.create_routine = [func = std::move(func), dv]() {
        return [func, dv]() {
            std::shared_ptr<M0> msg;
            for (;;) {
                auto routine = CRoutine::GetCurrentRoutine();
                if (routine == nullptr) {
                    std::this_thread::yield();
                    continue;
                }
                routine->set_state(RoutineState::DATA_WAIT);
                if (dv && dv->TryFetch(msg) && msg) {
                    func(msg);
                    routine->set_state(RoutineState::READY);
                    CRoutine::Yield();
                } else {
                    CRoutine::Yield();
                }
            }
        };
    };
    return factory;
}

template <typename M0, typename M1, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1>>& dv) {
    RoutineFactory factory;
    factory.SetDataVisitor(dv);
    factory.create_routine = [f = std::forward<F>(f), dv]() {
        return [f, dv]() {
            std::shared_ptr<M0> msg0;
            std::shared_ptr<M1> msg1;
            for (;;) {
                auto routine = CRoutine::GetCurrentRoutine();
                if (routine == nullptr) {
                    std::this_thread::yield();
                    continue;
                }
                routine->set_state(RoutineState::DATA_WAIT);
                if (dv->TryFetch(msg0, msg1)) {
                    f(msg0, msg1);
                    routine->set_state(RoutineState::READY);
                    CRoutine::Yield();
                } else {
                    CRoutine::Yield();
                }
            }
        };
    };
    return factory;
}

template <typename M0, typename M1, typename M2, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1, M2>>& dv) {
    RoutineFactory factory;
    factory.SetDataVisitor(dv);
    factory.create_routine = [f = std::forward<F>(f), dv]() {
        return [f, dv]() {
            std::shared_ptr<M0> msg0;
            std::shared_ptr<M1> msg1;
            std::shared_ptr<M2> msg2;
            for (;;) {
                auto routine = CRoutine::GetCurrentRoutine();
                if (routine == nullptr) {
                    std::this_thread::yield();
                    continue;
                }
                routine->set_state(RoutineState::DATA_WAIT);
                if (dv->TryFetch(msg0, msg1, msg2)) {
                    f(msg0, msg1, msg2);
                    routine->set_state(RoutineState::READY);
                    CRoutine::Yield();
                } else {
                    CRoutine::Yield();
                }
            }
        };
    };
    return factory;
}

template <typename M0, typename M1, typename M2, typename M3, typename F>
RoutineFactory CreateRoutineFactory(
    F&& f, const std::shared_ptr<data::DataVisitor<M0, M1, M2, M3>>& dv) {
    RoutineFactory factory;
    factory.SetDataVisitor(dv);
    factory.create_routine = [f = std::forward<F>(f), dv]() {
        return [f, dv]() {
            std::shared_ptr<M0> msg0;
            std::shared_ptr<M1> msg1;
            std::shared_ptr<M2> msg2;
            std::shared_ptr<M3> msg3;
            for (;;) {
                auto routine = CRoutine::GetCurrentRoutine();
                if (routine == nullptr) {
                    std::this_thread::yield();
                    continue;
                }
                routine->set_state(RoutineState::DATA_WAIT);
                if (dv->TryFetch(msg0, msg1, msg2, msg3)) {
                    f(msg0, msg1, msg2, msg3);
                    routine->set_state(RoutineState::READY);
                    CRoutine::Yield();
                } else {
                    CRoutine::Yield();
                }
            }
        };
    };
    return factory;
}

template <typename Function>
RoutineFactory CreateRoutineFactory(Function&& f) {
    RoutineFactory factory;
    factory.create_routine = [f = std::forward<Function&&>(f)]() { return f; };
    return factory;
}

}  // namespace croutine
}  // namespace autolink