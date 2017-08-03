/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * PCNet32 descriptor ring structure
 */

#ifndef _APOX_PCNET32DESCRIPTOR_H_
#define _APOX_PCNET32DESCRIPTOR_H_

#include <kernel/Object.h>

namespace PCNet32
{

/** Length of each TX descriptor buffer */
#define TX_BUF_LEN              2048U

/** Length of each RX descriptor buffer */
#define RX_BUF_LEN              2048U

/** A 32-bit-mode descriptor is 16 bytes */
class Descriptor
{
    public:

        /** C'tor */
        Descriptor()
        {
            physBuffer = 0;
            dword2 = 0;
            dword3 = 0;
            reserved = 0;

            // Bits 12-15 must be 1's
            dword2 = 0xF << 12;
        }

        int getErrors()
        {
            return 0;
        }

        inline bool isErrors()
        {
            return (dword2 & (1 << 30)) == 1;
        }

        /**
         * Returns true if the drivers owns the descriptor (either
         * automatically or by calling aquire)
         */
        bool isAquired()
        {
            return (dword2 & (1 << 31)) == 0;
        }

        /** Clears the OWN bit to let the driver own the descriptor */
        void aquire()
        {
            dword2 &= ~(1 << 31);
        }

        /** Sets the OWN bit to let the chip own the descriptor */
        void release()
        {
            dword2 |= (1 << 31);
        }

        /**
         * Sets the BCNT field (the method converts to negative two's complement
         * format as required by the chip)
         */
        inline void setBufferSize(int size)
        {
            dword2 = Bits::setRange(dword2, 0, 11, (uint32) (-size & 0xFFF));
            //dword2 = Convert::host2le((uint32) (-size & 0xFFF));
        }

        /** Print debug information */
        void dumpInfo()
        {
            out << "[RX] BUFFER: " << H(physBuffer) << eol;
            out << "[RX] DWORD2: " << H(dword2) << eol;
            out << "[RX] DWORD3: " << H(dword3) << eol;
        }

        uint32 getBuffer()
        {
            return physBuffer;
        }

        uint32 getDword2()
        {
            return dword2;
        }

        void setDword2(uint32 dw2)
        {
            dword2 = dw2;
        }

        uint32 getDword3()
        {
            return dword3;
        }

        void setDword3(uint32 dw3)
        {
            dword3 = dw3;
        }

    protected:

        /** Descriptor buffer; 32-bit physical with no alignment requirements. */
        uint32 physBuffer;
        uint32 dword2;
        uint32 dword3;
        uint32 reserved;
};


/** Receive descriptor */
class RxDescriptor : public Descriptor
{
    public:

        /** Initialize descriptor */
        void initialize(PCNet32Driver* driver)
        {
            physBuffer = (uint32)driver->toPhysRXBuffer(this);
            setBufferSize(RX_BUF_LEN);

            // Initially, we do not own the controller (1=chip owns)
            dword2 |= (1 << 31);
        }

        /**
         * Length of received message (packet) in bytes.
         * Only valid when ERR is cleared and ENP is set.
         */
        int getMessageLen()
        {
            return Convert::le2host((uint32)dword3 & 0xFFF);
        }

};

/** Transmit descriptor */
class TxDescriptor : public Descriptor
{
    public:

        /** Initalize descriptor */
        void initialize(PCNet32Driver* driver)
        {
            physBuffer = (uint32)driver->toPhysTXBuffer(this);
            setBufferSize(TX_BUF_LEN);

            // Set ADD_FCS to generate frame checksum; the chip never changes this.
            dword2 |= (1 << 29);
        }

        /** Transmit the buffer */
        inline void transmit(Buffer& buf, PCNet32Driver* driver)
        {
            transmit(buf.data(), buf.size(), driver);
        }

        /** Transmit the buffer */
        inline void transmit(uint8* buffer, int len, PCNet32Driver* driver)
        {
            // Copy data
            memcpy((void*)driver->toVirtual(physBuffer), buffer, len);

            // Set byte count
            dword2 |= Convert::host2le((uint32) -len & 0xFFF);

            // Set STP, ENP
            dword2 |= (1<<24 | 1<<25);

            // Let the chip OWN the descriptor
            release();
        }
};

} // namespace

#endif // _APOX_PCNET32DESCRIPTOR_H_
