/*!
  \file        timer.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/3/26

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A minimalistic stopwatch
 */
#ifndef TIMER_H
#define TIMER_H

// c includes
#include <sys/time.h>
#include <stdio.h>

class Timer {
public:
  typedef float Time;
  const static Time NOTIME = -1;

  Timer() {
    reset();
  }

  ~Timer() {
  }

  //! reset time to 0
  virtual inline void reset() {
    gettimeofday(&start, NULL);
    //usleep(2000);
  }

  //! get the time since ctor or last reset (milliseconds)
  virtual inline Time getTimeMilliseconds() const {
    struct timeval end;
    gettimeofday(&end, NULL);
    return (Time) (// seconds
                   (end.tv_sec - start.tv_sec)
                   * 1000 +
                   // useconds
                   (end.tv_usec - start.tv_usec)
                   / 1000.f);
  }

  //! get the time since ctor or last reset (seconds)
  virtual inline Time getTimeSeconds() const {
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

#endif // TIMER_H

