/**
 * Apox Operating System
 * Copyright (c) 2005-2006, cryptocode
 */

#include <kernel/drivers/timer/Realtime.h>
#include <kernel/io/Ports.h>
#include <kernel/util/Bcd.h>

// Initialize monostate object
int Realtime::seconds=0;
int Realtime::minutes=0;
int Realtime::hours=0;
int Realtime::day=0;
int Realtime::month=0;
int Realtime::year=0;
int Realtime::century=0;
String Realtime::textRepresentation;

/** Read wall time from CMOS */
void Realtime::read()
{
    // The CMOS chip works like this: first output to port 0x70 what CMOS register
    // you'd like to read, then actually read it from port 0x71. The values are BCD
    // encoded and must be converted to integers. There should be a delay between
    // output and input, but that is handled by the in/out functions by default.
    //
    // CMOS register indices of interest:
    //
    //  0x00 Seconds
    //  0x02 Minutes
    //  0x04 Hours
    //  0x07 Day of month
    //  0x08 Month
    //  0x09 Year
    //  0x32 Century

    // No interrupts when doing CMOS programming! Also, NMI's should be off.
    flag_t flags;
    disableLocalIRQ(&flags);

    out8(0x70, 0x00);
    Realtime::seconds = bcdDecode(in8(0x71));

    out8(0x70, 0x02);
    Realtime::minutes = bcdDecode(in8(0x71));

    out8(0x70, 0x04);
    Realtime::hours = bcdDecode(in8(0x71));

    out8(0x70, 0x07);
    Realtime::day = bcdDecode(in8(0x71));

    out8(0x70, 0x08);
    Realtime::month = bcdDecode(in8(0x71));

    out8(0x70, 0x09);
    Realtime::year = bcdDecode(in8(0x71));

    out8(0x70, 0x32);
    Realtime::century = bcdDecode(in8(0x71));

    // Restore state
    out8(0x70, 0x0D);

    // Restore interrupts
    restoreLocalIRQ(flags);

    //
    // Update text representation
    //
    textRepresentation = "";

    // DD.MM.CCYY HH:MM:SS
    if(Realtime::day < 10)
        textRepresentation << 0UL;
    textRepresentation << Realtime::day << ".";

    if(Realtime::month < 10)
        textRepresentation << 0UL;
    textRepresentation << Realtime::month << "." << Realtime::century;

    if(Realtime::year < 10)
        textRepresentation += 0UL;
    textRepresentation << Realtime::year << " ";

    if(Realtime::hours < 10)
        textRepresentation += 0UL;
    textRepresentation << Realtime::hours << ":";

    if(Realtime::minutes < 10)
        textRepresentation += 0UL;

    String colon(":");
    textRepresentation << Realtime::minutes << colon;

    if(Realtime::seconds < 10)
        textRepresentation += 0UL;
    textRepresentation << Realtime::seconds;
}

/**
 * Get text representation of wall time. The date and time is re-read from CMOS.
 *
 * \todo Localized version
 */
String& Realtime::toString()
{
    read();

    return textRepresentation;
}
