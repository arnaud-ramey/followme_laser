#ifndef TIMER_H_
#define TIMER_H_

/*!
* \file timer.h
*
* Useful functions for time measures
*
* \date 10/11/2010
* \author Arnaud Ramey, Víctor González (vgonzale@ing.uc3m.es)
*/

// c includes
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
class Timer {
public:
  typedef double Time;
  static const Time NOTIME = -1;
  Timer() { reset(); }
  virtual inline void reset() {
    gettimeofday(&start, NULL);
  }
  //! get the time since ctor or last reset (milliseconds)
  virtual inline Time getTimeSeconds() const {
    struct timeval end;
    gettimeofday(&end, NULL);
    return (Time) (// seconds
                   (end.tv_sec - start.tv_sec)
                   +
                   // useconds
                   (end.tv_usec - start.tv_usec)
                   / 1E6);
  }
private:
  struct timeval start;
}; // end class Timer
al inline Time getTimeSeconds() const {
    return getTimeMilliseconds() / 1000.f;
  }

  //! get the time since ctor or last reset (milliseconds)
  virtual inline Time time() const {
    return getTimeMilliseconds();
  }

  //! print time needed for a task identified by its string
  virtual inline void printTime(const char* msg) {
    printf("Time for %s : %g ms.\n", msg, getTimeMilliseconds());
  }

  //! print time needed for a task identified by its string
  virtual inline void printTime_factor(const char* msg, const int times) {
    printf("Time for %s (%i times) : %g ms.\n",
                msg, times, getTimeMilliseconds() / times);
  }

private:
  struct timeval start;
};

#endif /*TIMER_H_*/

