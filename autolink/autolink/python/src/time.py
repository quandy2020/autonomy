#!/usr/bin/env python3

#***************************************************************************
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#***************************************************************************

# -*- coding: utf-8 -*-
"""Module for init environment."""

try:
    from ._loader import load_wrapper_module
except ImportError:
    from _loader import load_wrapper_module

_AUTOLINK = load_wrapper_module("_autolink_wrapper")
_AUTOLINK_TIME = load_wrapper_module("_autolink_time_wrapper")


class Duration(object):

    """
    Class for autolink Duration wrapper.
    """

    def __init__(self, other):
        if isinstance(other, int):
            self.nanoseconds_ = other
        elif isinstance(other, float):
            self.nanoseconds_ = other * 1000000000
        elif isinstance(other, Duration):
            self.nanoseconds_ = other.nanoseconds_
        self.duration_ = _AUTOLINK_TIME.new_PyDuration(int(self.nanoseconds_))

    def __del__(self):
        _AUTOLINK_TIME.delete_PyDuration(self.duration_)

    def sleep(self):
        """
        sleep for the amount of time specified by the duration.
        """
        _AUTOLINK_TIME.PyDuration_sleep(self.duration_)

    def __str__(self):
        return str(self.nanoseconds_)

    def to_sec(self):
        """
        convert to second.
        """
        return float(self.nanoseconds_) / 1000000000

    def to_nsec(self):
        """
        convert to nanosecond.
        """
        return self.nanoseconds_

    def iszero(self):
        return self.nanoseconds_ == 0

    def __add__(self, other):
        return Duration(self.nanoseconds_ + other.nanoseconds_)

    def __radd__(self, other):
        return Duration(self.nanoseconds_ + other.nanoseconds_)

    def __sub__(self, other):
        return Duration(self.nanoseconds_ - other.nanoseconds_)

    def __lt__(self, other):
        return self.nanoseconds_ < other.nanoseconds_

    def __gt__(self, other):
        return self.nanoseconds_ > other.nanoseconds_

    def __le__(self, other):
        return self.nanoseconds_ <= other.nanoseconds_

    def __ge__(self, other):
        return self.nanoseconds_ >= other.nanoseconds_

    def __eq__(self, other):
        return self.nanoseconds_ == other.nanoseconds_

    def __ne__(self, other):
        return self.nanoseconds_ != other.nanoseconds_


class Time(object):

    """
    Class for autolink time wrapper.
    """

    ##
    # @brief Constructor, creates a Time.
    #
    # @param other float means seconds unit.
    # int means nanoseconds.
    def __init__(self, other):
        nanoseconds = 0
        if isinstance(other, int):
            nanoseconds = other
        elif isinstance(other, float):
            nanoseconds = other * 1000000000
        elif isinstance(other, Time):
            nanoseconds = other.to_nsec()

        self.time = _AUTOLINK_TIME.new_PyTime(int(nanoseconds))
        if self.time is None:
            raise RuntimeError("Failed to create PyTime object")

    def __del__(self):
        if hasattr(self, 'time') and self.time is not None:
            _AUTOLINK_TIME.delete_PyTime(self.time)

    def __str__(self):
        return str(self.to_nsec())

    def iszero(self):
        return self.to_nsec() == 0

    @staticmethod
    def now():
        """
        return current time.
        """
        # print _CYBER_TIME.PyTime_now()
        # print type(_CYBER_TIME.PyTime_now())
        time_now = Time(_AUTOLINK_TIME.PyTime_now())
        return time_now

    @staticmethod
    def mono_time():
        mono_time = Time(_AUTOLINK_TIME.PyTime_mono_time())
        return mono_time

    def to_sec(self):
        """
        convert to second.
        """
        if self.time is None:
            return 0.0
        return _AUTOLINK_TIME.PyTime_to_sec(self.time)

    def to_nsec(self):
        """
        convert to nanosecond.
        """
        if self.time is None:
            return 0
        return _AUTOLINK_TIME.PyTime_to_nsec(self.time)

    def sleep_until(self, cyber_time):
        """
        sleep until time.
        """
        if isinstance(time, Time):
            return _AUTOLINK_TIME.PyTime_sleep_until(self.time,
                                                  cyber_time.to_nsec())
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, Time):
            return Duration(self.to_nsec() - other.to_nsec())
        else:
            isinstance(other, Duration)
            return Time(self.to_nsec() - other.to_nsec())

    def __add__(self, other):
        return Time(self.to_nsec() + other.to_nsec())

    def __radd__(self, other):
        return Time(self.to_nsec() + other.to_nsec())

    def __lt__(self, other):
        return self.to_nsec() < other.to_nsec()

    def __gt__(self, other):
        return self.to_nsec() > other.to_nsec()

    def __le__(self, other):
        return self.to_nsec() <= other.to_nsec()

    def __ge__(self, other):
        return self.to_nsec() >= other.to_nsec()

    def __eq__(self, other):
        return self.to_nsec() == other.to_nsec()

    def __ne__(self, other):
        return self.to_nsec() != other.to_nsec()


class Rate(object):

    """
    Class for autolink Rate wrapper. Help run loops at a desired frequency.
    """

    ##
    # @brief Constructor, creates a Rate.
    #
    # @param other float means frequency the desired rate to run at in Hz.
    # int means the expected_cycle_time.
    def __init__(self, other):
        if isinstance(other, int):
            self.rate_ = _AUTOLINK_TIME.new_PyRate(other)
        elif isinstance(other, float):
            # Convert frequency (Hz) to period (nanoseconds)
            # e.g., 1.0 Hz -> 1 second = 1e9 nanoseconds
            period_ns = int(1.0 / other * 1e9)
            self.rate_ = _AUTOLINK_TIME.new_PyRate(period_ns)
        elif isinstance(other, Duration):
            self.rate_ = _AUTOLINK_TIME.new_PyRate(other.to_nsec())

    def __del__(self):
        _AUTOLINK_TIME.delete_PyRate(self.rate_)

    def __str__(self):
        return "cycle_time = %s, exp_cycle_time = %s" % (str(self.get_cycle_time()), str(self.get_expected_cycle_time()))

    def sleep(self):
        """
        Sleeps for any leftover time in a cycle.
        """
        _AUTOLINK_TIME.PyRate_sleep(self.rate_)

    def reset(self):
        """
        Sets the start time for the rate to now.
        """
        _AUTOLINK_TIME.PyRate_PyRate_reset(self.rate_)

    def get_cycle_time(self):
        """
        Get the actual run time of a cycle from start to sleep.
        """
        return Duration(_AUTOLINK_TIME.PyRate_get_cycle_time(self.rate_))

    def get_expected_cycle_time(self):
        """
        Get the expected cycle time.
        """
        return Duration(_AUTOLINK_TIME.PyRate_get_expected_cycle_time(self.rate_))