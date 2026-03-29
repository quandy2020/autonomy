/*
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

#include <map>
#include <queue>

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

//! Priority queue for generic elements with integer priorities (e.g. squared
//! distances).
/** A priority queue that uses buckets to group elements with the same priority.
 *  The individual buckets are unsorted, which increases efficiency if these
 * groups are large. The priorities are typically squared Euclidean distances
 * (integers).
 */
template <typename T>
class BucketPrioQueue
{
public:
    /**
     * Standard constructor. When called for the first time it creates a look up
     * table that maps square distances to bucket numbers, which might take some
     * time...
     */
    BucketPrioQueue();

    void clear() {
        buckets.clear();
        count = 0;
        nextPop = buckets.end();
    }

    /**
     * Check if the queue is empty
     * @return True if the queue is empty, false otherwise
     */
    bool empty();

    /**
     * Push an element with a priority
     * @param prio The priority of the element
     * @param t The element to push
     */
    void push(int prio, T t);

    /**
     * Pop the element with the lowest priority
     * @return The element with the lowest priority
     */
    T pop();

    int size() {
        return count;
    }
    int getNumBuckets() {
        return buckets.size();
    }

    int getTopPriority() {
        return nextPop->first;
    }

private:
    int count;

    typedef std::map<int, std::queue<T> > BucketType;
    BucketType buckets;
    typename BucketType::iterator nextPop;
};

template <class T>
BucketPrioQueue<T>::BucketPrioQueue() {
    clear();
}

template <class T>
bool BucketPrioQueue<T>::empty() {
    return (count == 0);
}

template <class T>
void BucketPrioQueue<T>::push(int prio, T t) {
    buckets[prio].push(t);
    if (nextPop == buckets.end() || prio < nextPop->first)
        nextPop = buckets.find(prio);
    count++;
}

template <class T>
T BucketPrioQueue<T>::pop() {
    while (nextPop != buckets.end() && nextPop->second.empty())
        ++nextPop;

    T p = nextPop->second.front();
    nextPop->second.pop();
    if (nextPop->second.empty()) {
        typename BucketType::iterator it = nextPop;
        nextPop++;
        buckets.erase(it);
    }
    count--;
    return p;
}

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy