/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//modified to be a light version of the ros time class.

#ifndef LIGHT_TIME
#define LIGHT_TIME


#include <cmath>
#include <stdint.h>
#include <lightDuration.h>

  /*********************************************************************
   ** Functions
   *********************************************************************/

   void normalizeSecNSec(uint64_t& sec, uint64_t& nsec);
   void normalizeSecNSec(uint32_t& sec, uint32_t& nsec);
   void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);
   void ros_walltime(uint32_t& sec, uint32_t& nsec);
   void ros_steadytime(uint32_t& sec, uint32_t& nsec);

  /*********************************************************************
   ** Time Classes
   *********************************************************************/

  /**
   * \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
   * This should not need to be used directly.
   */
  template<class T, class D>
  class LightTime
  {
  public:
    uint32_t sec, nsec;

    LightTime() : sec(0), nsec(0) { }
    LightTime(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
    {
      normalizeSecNSec(sec, nsec);
    }
    explicit LightTime(double t) { fromSec(t); }
    D operator-(const T &rhs) const;
    T operator+(const D &rhs) const;
    T operator-(const D &rhs) const;
    T& operator+=(const D &rhs);
    T& operator-=(const D &rhs);
    bool operator==(const T &rhs) const;
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const;
    bool operator<(const T &rhs) const;
    bool operator>=(const T &rhs) const;
    bool operator<=(const T &rhs) const;

    double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
    T& fromSec(double t) {
      int64_t sec64 = (int64_t)floor(t);
//      if (sec64 < 0 || sec64 > UINT32_MAX)
//        std::cout<<"Time is out of dual 32-bit range";
      sec = (uint32_t)sec64;
      nsec = (uint32_t)round((t-sec) * 1e9);
      // avoid rounding errors
      sec += (nsec / 1000000000ul);
      nsec %= 1000000000ul;
      return *static_cast<T*>(this);
    }

    uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
    T& fromNSec(uint64_t t);

    inline bool isZero() const { return sec == 0 && nsec == 0; }
    inline bool is_zero() const { return isZero(); }

  };

  /**
   * \brief Time representation.  May either represent wall clock time or ROS clock time.
   *
   * ros::TimeBase provides most of its functionality.
   */
  class  Time : public LightTime<Time, LightDuration<Time>>
  {
  public:
    Time()
      : LightTime<Time, LightDuration<Time>>()
    {}

    Time(uint32_t _sec, uint32_t _nsec)
      : LightTime<Time, LightDuration<Time>>(_sec, _nsec)
    {}

    explicit Time(double t) { fromSec(t); }

    /**
     * \brief Retrieve the current time.  If ROS clock time is in use, this returns the time according to the
     * ROS clock.  Otherwise returns the current wall clock time.
     */

  };





#endif // LIGHT_TIME

