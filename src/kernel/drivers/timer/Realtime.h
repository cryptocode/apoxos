/**
 * Apox Operating System
 * Copyright (c) 2005-2006, cryptocode
 */

#ifndef _APOX_REALTIME_H_
#define _APOX_REALTIME_H_

#include <kernel/libc/std.h>
#include <kernel/unicode/Unistring.h>

/**
 * Wall time from CMOS
 */
class Realtime
{
    public:

        static int seconds;
        static int minutes;
        static int hours;
        static int day;
        static int month;
        static int year;
        static int century;

        /** \brief Read real time from CMOS */
        static void read();

        /** String representation */
        static String& toString();

    private:

        static String textRepresentation;
};

#endif // _APOX_REALTIME_H_
