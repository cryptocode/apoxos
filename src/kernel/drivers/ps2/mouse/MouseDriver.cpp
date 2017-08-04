/*
 * Apox Operating System
 * Copyright (c) 2007, cryptocode
 *
 * Mouse driver
 */

#include <kernel/drivers/ps2/mouse/MouseDriver.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/init/irq.h>
#include <kernel/adt/List.h>
#include <arch/Delay.h>

namespace
{
    /** List of mouse listeners */
    List<MouseListener*> listeners;

    /** Internal driver name */
    #define DRIVERNAME              "PS/2 Compatible Mouse"

    /* Ports are relative to 0x60 */
    #define PS2_PORTBASE            0x60
    #define PS2_PORT_DATA           0x0
    #define PS2_PORT_CTRL           0x4

    #define PS2_CTRL_WRITE_CMD      0x60
    #define PS2_CTRL_WRITE_AUX      0xD4

    // data words
    #define PS2_CMD_DEV_INIT        0x43
    #define PS2_CMD_ENABLE_MOUSE    0xF4
    #define PS2_CMD_DISABLE_MOUSE   0xF5
    #define PS2_CMD_RESET_MOUSE     0xFF

    #define PS2_RES_ACK             0xFA
    #define PS2_RES_RESEND          0xFE

    #define SIG_THREAD_EXIT         2
}

/**
 * The purpose of the read thread is to:
 *
 *  1) Reduce interrupt latency by doing as little as possible (just a wake-up)
 *  2) Do mouse packet handling in thread context so we can sleep, etc
 *
 * @see Threadable#run */
void MouseDriver::Receiver::run(void* arg)
{
    for(;;)
    {
        // Wait 'till someone calls notify() on us
        wait();

        if(getSignal() != SIG_THREAD_EXIT)
        {
            List<void*>& messages = getMessages();

            for(;;)
            {
                AutoSpin lck(messages);
                MouseEvent* evt = (MouseEvent*)messages.pop();

                if(evt == null)
                    break;

                foreach_ref(listeners)
                {
                    listeners.currentItem()->onMouseEvent(evt);
                }

                delete evt;

                lck.unlock();
            }
        }
    }
}

void MouseListener::addMouseListener(MouseListener* listener)
{
    AutoSpin lck(listeners);
    listeners.appendItem(listener);
}

void MouseListener::removeMouseListener(MouseListener* listener)
{
    AutoSpin lck(listeners);
    listeners.remove(listener);
}

MouseDriver::MouseDriver() : receiver(this)
{
    dev = null;
}

MouseDriver::~MouseDriver()
{
}

/**
 * Get driver for the device; NULL if no device is found (in which case; check
 * lastError)
 */
MouseDriver* MouseDriver::initialize()
{
    String name(DRIVERNAME);

    MouseDriver* driver = new MouseDriver();
    driver->dev = new Device(driver, name);
    Thread::create(&driver->receiver);

    // Probe and register
    if(driver->probe() == E_OK)
    {
        DeviceManager::addDevice(driver->dev);
        installIRQHandler(IRQ_SYSTEM_PS2MOUSE, driver);
        printf("Success\n");
    }
    else
    {
        delete driver;
        driver = null;
    }

    return driver;
}

/** Probe and initialize */
error_t MouseDriver::probe()
{
    error_t res = E_OK;

    this->setIOBase(PS2_PORTBASE);

    // Empty out-buffer
    if(in8(PS2_PORT_CTRL) & 0x1)
    {
        in8(PS2_PORT_DATA);
    }

    // Initialize device
    writeCommand(PS2_CMD_DEV_INIT);
    // A delay is required after init, or some systems will lock up
    Delay::ms(10);
    // Enable
    writeAux(PS2_CMD_ENABLE_MOUSE);

    if(readData() != PS2_RES_ACK)
    {
       out << "No PS/2 mouse found" << eol;
       res = Thread::setLastError(E_DEVICE_NOT_FOUND);
    }
    else
    {
        printf("PS/2 mouse controller found\n");
    }

    return res;
}


MouseEvent mp;

/** @see DeviceDriver#onInterrupt */
int MouseDriver::onInterrupt(ContextState* ctx)
{
    static volatile int irqCount = 0;

    // We get three interrupts per event
    uint8 data = readData();

    //out << "," << H(data) << eos;

    irqCount++;

    switch(irqCount)
    {
        // STATUS
        case 1:
        {
            mp.mouseState = data;

            // Ignore ack's and words where bit#3 is not 1 (is should always be 1)
            if(mp.mouseState == PS2_RES_ACK)
            {
                irqCount--;
                return IRQRES_NORMAL;
            }

            break;
        }

        // DELTA X
        case 2:
        {
            mp.deltaX = data;
            break;
        }

        // DELTA Y
        case 3:
        {
             mp.deltaY = data;
             break;
        }

        default:
        {
            kprintf("MOUSE: irq count is bogus");
            assert(false);
        }
    };

    // Got a packet?
    if(irqCount == 3)
    {
        irqCount = 0;

        // Negative delta X?
        if(mp.mouseState & 0x10)
        {
            mp.deltaX = 0xFFFFFF00  | (uint8)mp.deltaX;
        }

        // Negative delta Y?
        if(mp.mouseState & 0x20)
        {
            mp.deltaY = (0xFFFFFF00  | (uint8)mp.deltaY);
        }

        mp.deltaY = -mp.deltaY;

        // Set button states; we remove the upper 5 bits (which are overflow and sign bits)
        // Note that the MB_LEFT, etc enums corresponds to the bit values in mouseState.
        mp.mouseState = mp.mouseState & 0x7;

        List<void*>& messages = receiver.getMessages();
        AutoSpin lck(messages);
        MouseEvent* evt = (MouseEvent*)messages.peek();
        lck.unlock();

        if(evt)
        {
            *evt = mp;
        }
        else
        {
            evt = new MouseEvent();
            *evt = mp;
            receiver.notifyMessage(evt);
        }
    }

    return IRQRES_NORMAL;
}

/** Wait up to 100 milliseconds */
error_t MouseDriver::waitRead()
{
    int i=0;
    for(i=0; i < 100000; i++)
    {
        if((in8(PS2_PORT_CTRL) & 0x1) == 1)
            break;

        Delay::us(1);
    }

    return i < 100000 ? OK : E_TIMEOUT;
}


/** Wait up to 100 milliseconds */
error_t MouseDriver::waitCtrl(int waitMask)
{
    int i=0;
    for(i=0; i < 100000; i++)
    {
        if((in8(PS2_PORT_CTRL) & waitMask) == 0)
            break;

        Delay::us(1);
    }

    return i < 100000 ? OK : E_TIMEOUT;
}

/** Write PS/2 command */
void MouseDriver::writeCommand(uint8 cmd)
{
    // in/out buffer full bits must be 0
    waitCtrl(0x03);
    out8(PS2_PORT_CTRL, PS2_CTRL_WRITE_CMD);

    // in-buffer full bit must be 0
    waitCtrl(0x02);
    out8(PS2_PORT_DATA, cmd);
}

/** Write PS/2 aux command */
void MouseDriver::writeAux(uint8 cmd)
{
    // in/out buffer full bits must be 0
    waitCtrl(0x03);
    out8(PS2_PORT_CTRL, PS2_CTRL_WRITE_AUX);

    // in-buffer full bit must be 0
    waitCtrl(0x02);
    out8(PS2_PORT_DATA, cmd);
}

/** Read PS/2 data */
uint8 MouseDriver::readData()
{
    waitRead();
    return in8(PS2_PORT_DATA);
}
