/*
 * Apox Operating System
 * Copyright (c) 2007, cryptocode
 */

#ifndef _APOX_MOUSEDRIVER_H_
#define _APOX_MOUSEDRIVER_H_

#include <kernel/drivers/Device.h>
#include <kernel/io/Ports.h>
#include <kernel/threads/Thread.h>

/** Represents a mouse event */
class MouseEvent
{
    public:

        enum
        {
            MB_LEFT = 1,
            MB_RIGHT = 2,
            MB_MIDDLE = 4,
        };

        int32 deltaX;
        int32 deltaY;
        int mouseState;
};

/** Mouse listener interface */
class MouseListener
{
    public:

        static void addMouseListener(MouseListener* listener);
        static void removeMouseListener(MouseListener* listener);

        virtual bool onMouseEvent(MouseEvent* evt) { return true; }
        virtual bool onMouseDown(int x, int y) { return true; }
        virtual bool onMouseUp(int x, int y) { return true; }
        virtual bool onMouseMove(bool isMouseDown, int x, int y) { return true; }
};

/**
 * Generic PS/2 mouse driver
 */
class MouseDriver : public DeviceDriver, public PortIO
{
    public:

        class Receiver : public Threadable, public Waitable
        {
            public:

                /** C'tor */
                Receiver(MouseDriver* driver)
                {
                    this->driver = driver;
                }

                /** @see Threadable#run */
                virtual void run(void* arg);

            private:

                MouseDriver* driver;
        };

        Receiver receiver;

        /** Get driver for the device */
        static MouseDriver* initialize();

        /** @see DeviceDriver#onInterrupt */
        virtual int onInterrupt(ContextState* ctx);

    private:

        error_t waitCtrl(int waitMask);
        error_t waitRead();
        uint8 readData();
        void writeAux(uint8 cmd);
        void writeCommand(uint8 cmd);

        MouseDriver();
        virtual ~MouseDriver();

        /** Probe and initialize */
        error_t probe();

        /** The device we're driving */
        Device* dev;
};

#endif // _APOX_MOUSEDRIVER_H_
