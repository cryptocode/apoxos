/*
 * Apox Operating System
 * Copyright (c) 2007, cryptocode
 */

#ifndef _APOX_FS_FAT_FATFILECHANNEL_H_
#define _APOX_FS_FAT_FATFILECHANNEL_H_

#include <kernel/libc/std.h>
#include <kernel/io/Channel.h>

class FATNode;

/**
 * FAT file channel
 */
class FATFileChannel : public Channel
{
    public:

        /** C'tor */
        FATFileChannel(FATNode* node);

        /** D'tor */
        ~FATFileChannel();

        /**
         * @see Channel#clone
         */
        virtual AbstractChannel* clone();

        /**
         * @see Channel#read
         */
        virtual ssize_t read(uint8* bytes, size_t length, bool block);


        /**
         * @see Channel#read
         */
        virtual int read();


        /**
         * @see Channel#write
         */
        virtual ssize_t write(const uint8* bytes, size_t length, bool block);

        /**
         * @see Channel#write
         */
        virtual int write(int c);

    private:

        FATNode* node;
};

#endif // _APOX_FS_FAT_FATFILECHANNEL_H_
