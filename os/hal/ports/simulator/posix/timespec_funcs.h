#pragma once

#include <sys/time.h>

/*
 * from bsd <sys/time.h>
 */
#define timespecclear(tvp)  ((tvp)->tv_sec = (tvp)->tv_nsec = 0)
#define timespecisset(tvp)  ((tvp)->tv_sec || (tvp)->tv_nsec)
#define timespeccmp(tvp, uvp, cmp)                  \
    (((tvp)->tv_sec == (uvp)->tv_sec) ?             \
        ((tvp)->tv_nsec cmp (uvp)->tv_nsec) :           \
        ((tvp)->tv_sec cmp (uvp)->tv_sec))

#define timespecadd(tsp, usp, vsp)                  \
    do {                                \
        (vsp)->tv_sec = (tsp)->tv_sec + (usp)->tv_sec;      \
        (vsp)->tv_nsec = (tsp)->tv_nsec + (usp)->tv_nsec;   \
        if ((vsp)->tv_nsec >= 1000000000L) {            \
            (vsp)->tv_sec++;                \
            (vsp)->tv_nsec -= 1000000000L;          \
        }                           \
    } while (0)
#define timespecsub(tsp, usp, vsp)                  \
    do {                                \
        (vsp)->tv_sec = (tsp)->tv_sec - (usp)->tv_sec;      \
        (vsp)->tv_nsec = (tsp)->tv_nsec - (usp)->tv_nsec;   \
        if ((vsp)->tv_nsec < 0) {               \
            (vsp)->tv_sec--;                \
            (vsp)->tv_nsec += 1000000000L;          \
        }                           \
    } while (0)
