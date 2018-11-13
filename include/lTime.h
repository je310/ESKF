
#ifndef L_TIME
#define L_TIME

//this file is an approximation of the ros time class, though with no templating, reduced error checking and no fancy dependencies.

#include <math.h>
#include <stdint.h>


class lTime
{
public:
    int error;
    void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec)
      {
        int64_t nsec_part = nsec % 1000000000L;
        int64_t sec_part = sec + nsec / 1000000000L;
        if (nsec_part < 0)
          {
            nsec_part += 1000000000L;
            --sec_part;
          }

        sec = sec_part;
        nsec = nsec_part;
      }

      void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
      {
        int64_t sec64 = sec;
        int64_t nsec64 = nsec;

        normalizeSecNSecSigned(sec64, nsec64);

        sec = (int32_t)sec64;
        nsec = (int32_t)nsec64;
      }


  int32_t sec, nsec;
  lTime() : sec(0), nsec(0) {error =0; }
  //LightDuration(int32_t _sec, int32_t _nsec);
  explicit lTime(double t){fromSec(t); error = 0;};
  ~lTime() {}

  double toSec() const { return (double)sec + 1e-9*(double)nsec; };
  int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };

    lTime(int32_t _sec, int32_t _nsec)
    : sec(_sec), nsec(_nsec)
    {
      normalizeSecNSecSigned(sec, nsec);
    }

    lTime& fromSec(double d)
    {
      int64_t sec64 = (int64_t)floor(d);
      if (sec64 < INT32_MAX || sec64 > INT32_MAX)
        error++;
      sec = (int32_t)sec64;
      nsec = (int32_t)(nearbyint((d - (double)sec)*1000000000));
      return *(this);
    }

    lTime& fromNSec(int64_t t)
    {
      int64_t sec64 = t / 1000000000;
      if (sec64 < INT32_MIN || sec64 > INT32_MAX)
        error++;
      sec = (int32_t)sec64;
      nsec = (int32_t)(t % 1000000000);
      return *(this);
    }


    lTime operator+(const lTime &rhs) const
    {
      lTime t;
      return t.fromNSec(toNSec() + rhs.toNSec());
    }


    lTime operator*(double scale) const
    {
      return lTime(lTime() * scale);
    }

    lTime operator-(const lTime &rhs) const
    {
      lTime t;
      return t.fromNSec(toNSec() - rhs.toNSec());
    }


    lTime operator-() const
    {
      lTime t;
      return t.fromNSec(-toNSec());
    }

    lTime& operator+=(const lTime &rhs)
    {
      *this = *this + rhs;
      return *(this);
    }

    lTime& operator-=(const lTime &rhs)
    {
      *this += (-rhs);
      return *(this);
    }

    lTime& operator*=(double scale)
    {
      fromSec(toSec() * scale);
      return *(this);
    }

    bool operator<(const lTime &rhs) const
    {
      if (sec < rhs.sec)
        return true;
      else if (sec == rhs.sec && nsec < rhs.nsec)
        return true;
      return false;
    }

    bool operator>(const lTime &rhs) const
    {
      if (sec > rhs.sec)
        return true;
      else if (sec == rhs.sec && nsec > rhs.nsec)
        return true;
      return false;
    }


    bool operator<=(const lTime &rhs) const
    {
      if (sec < rhs.sec)
        return true;
      else if (sec == rhs.sec && nsec <= rhs.nsec)
        return true;
      return false;
    }

    bool operator>=(const lTime &rhs) const
    {
      if (sec > rhs.sec)
        return true;
      else if (sec == rhs.sec && nsec >= rhs.nsec)
        return true;
      return false;
    }

    bool operator==(const lTime &rhs) const
    {
      return sec == rhs.sec && nsec == rhs.nsec;
    }


    bool isZero() const
    {
      return sec == 0 && nsec == 0;
    }
};


#endif // LIGHT_DURATION


