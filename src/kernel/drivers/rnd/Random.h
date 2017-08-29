/**
 * The Apox Operating System
 * Copyright (c) 2006, crypto
 */

#ifndef _APOX_RANDOMCHANNEL_H_
#define _APOX_RANDOMCHANNEL_H_

#include <kernel/io/Channel.h>
#include <kernel/adt/Random.h>
#include <stdlib.h>

/**
 * A random output channel. This is attached to /sys/random in the
 * global namespace.
 */
class RandomChannel : public InputChannel
{
    public:

        /** RandomChannel is not clonable, so this always returns null */
        virtual AbstractChannel* clone()
        {
            return null;
        }

        /** RandomChannel, despite its name, does not support random
         * access to the channel. */
        virtual bool isRandom()
        {
            return false;
        }

        /** Delivers random characters */
        virtual int read()
        {
            return rand() & 0xFF;
        }

        /** Delivers random 32-bit characters */
        virtual uint32 read32()
        {
            return rand();
        }

        /** @see AbsractChannel#read */
        virtual ssize_t read(uint8* bytes, size_t length, bool block = true)
        {
            for(int i=0; i < length; i++)
            {
                bytes[i] = (uint8) read();
            }

            return (ssize_t)length;
        }

};

#endif // _APOX_RANDOMCHANNEL_H_
