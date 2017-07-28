/*
 * The Apox Operating System
 * Copyright (c) 2005-2006, cryptocode
 *
 * Programming the MF II keyboard boils down to reading and writing to a couple
 * of IO ports, in addition to handling keyboard interrupts.
 *
 * 0x60 Read  = from kb into output buffer
 * 0x60 Write = to kb from input buffer
 * 0x64 Read  = read from keyboard status register
 * 0x64 Write = update control register
 *
 */

#include <stdio.h>
#include <ctype.h>
#include <kernel/drivers/keyboard/DriverKeyboard.h>
#include <kernel/drivers/video/Video.h>
#include <kernel/drivers/video/GraphicsDriver.h>
#include <kernel/threads/ThreadScheduler.h>
#include <kernel/vmm/malloc.h>
#include <kernel/Errors.h>
#include <kernel/drivers/DeviceManager.h>
#include <kernel/io/Channel.h>

namespace
{
    #define KB_STATUS_OUTBUFF       1
    #define KB_STATUS_INBUFF        2

    // This is a.k.a. the self test command
    #define KB_STATUS_SYSFLAG       4
    #define KB_STATUS_CMD_DATA      8
    #define KB_STATUS_KBLOCK        16
    #define KB_STATUS_AUXBUFF       32
    #define KB_STATUS_TIMEOUT       64
    // PS/2 only
    #define KB_STATUS_PARITY        128

    #define LED_NUMLOCK             1
    #define LED_SCROLLOCK           2
    #define LED_CAPSLOCK            4

    /** List of keyboard event listeners. */
    List<KeyListener*> listeners;

    /** The one and only keyboard driver instance */
    Keyboard* driver;

    /**
     * Blocks until the kbd input buffer is empty
     */
    void kbdWait()
    {
        // wait until port 0x64 returns second bit set => input buffer empty
        while ((in8(0x64) & KB_STATUS_INBUFF) != 0)
        {
            /*no-op*/
        }
    }

    /**
     * Returns true if the scan code is a possible gray key (such as the
     * arrow keys)
     */
    bool isGray(uint8 scancode)
    {
        bool res = false;

        switch(scancode)
        {
            case KBD_SCAN_KEYPAD7_HOME:
            case KBD_SCAN_KEYPAD8_UP:
            case KBD_SCAN_KEYPAD9_PGUP:
            case KBD_SCAN_KEYPAD4_LEFT:
            case KBD_SCAN_KEYPAD6_RIGHT:
            case KBD_SCAN_KEYPAD1_END:
            case KBD_SCAN_KEYPAD2_DOWN:
            case KBD_SCAN_KEYPAD3_PGDN:
            case KBD_SCAN_KEYPAD0_INS:
            case KBD_SCAN_KEYPAD_DEL:
            {
                res = true;
            }

        };

        return res;
    }

}

void KeyListener::addKeyListener(KeyListener* listener)
{
    listeners.appendItem(listener);
}

void KeyListener::removeKeyListener(KeyListener* listener)
{
    listeners.remove(listener);
}

/* See declaration */
void Keyboard::Receiver::run(void* arg)
{
    for(;;)
    {
        // Wait 'till someone calls notify() on us
        wait();

        List<void*>& messages = getMessages();

        for(;;)
        {
            AutoSpin lck(messages);
            KeyEvent* evt = (KeyEvent*)messages.pop();

            if(evt == null)
                break;

            // TODO: only send to "focal" app
            foreach_ref(listeners)
            {
                listeners.currentItem()->onKeyEvent(evt);
            }

            delete evt;
            lck.unlock();
        }
    }
}

/**
 * Initialize the keyboard driver
 */
void Keyboard::initialize()
{
    driver = new Keyboard();

    driver->focusedComponent = null;
    driver->eventQueue = queueCreate();
    driver->lastScancode = KBD_SCAN_INVALID;

    // Register device
    String name("Keyboard");
    Device* dev = new Device(driver, name);
    DeviceManager::addDevice(dev);

    kassert(driver != null);

    // Scancode -> unicode mapping fixups
    int elementCount = sizeof(kbdBaseUnicodeMapping[0]) / sizeof *(kbdBaseUnicodeMapping[0]);

    for(int j=0; j < 3; j++)
    {
        for(int i=0; i < elementCount; i++)
        {
            // We place extended scan code mappings in upper half of the table
            if(i == 128 + 0x35 /* keypad-/ */)
            {
                kbdBaseUnicodeMapping[j][i] = '/';
            }
            else if(i == 128 + 0x1c /* keypad-Enter */)
            {
                kbdBaseUnicodeMapping[j][i] = '\n';
            }
            else if(kbdBaseUnicodeMapping[j][i] == 0)
            {
                // Replace 0 (how C initializes arrays) with -1 (meaning no Unicode mapping)
                kbdBaseUnicodeMapping[j][i] = -1;
            }
        }
    }

    // NO_no keymap fix-ups
    kbdBaseUnicodeMapping[KBD_MAP_UNSHIFTED][86] = '<';
    kbdBaseUnicodeMapping[KBD_MAP_SHIFTED][86] = '>';

    // Start off with clear LED's
    clearLED();

    // We're ready to receive keyboard interrupts
    installIRQHandler(IRQ_SYSTEM_KEYBOARD, driver);

    // Make a stdin channel
    stdkbd = new KeyboardChannel();
    __apox_set_stdio_in(stdkbd);
}

/**
 *  Set focal component
 *  @returns Previous focus, or NULL
 */
Component* Keyboard::setFocus(Component* component)
{
    Component* prev = driver->focusedComponent;
    driver->focusedComponent = component;
    return prev;
}

/** Returns current focal compoenent, or NULL */
Component* Keyboard::getFocus()
{
    return driver->focusedComponent;
}

int Keyboard::onInterrupt(ContextState* ctx)
{
    // Make sure we're called through an interrupt gate
    kassert(irqEnabled() == FALSE);

    // Get scan code
    unsigned char scancode  = in8(0x60);
    unsigned char scanvalue = scancode & ~KBD_BREAK_MASK;

    // In order to process the key event, require a focused component
    if(focusedComponent != null && scancode != KBD_SCAN_INVALID)
    {
        // Create key event
        KeyEvent* evt = new KeyEvent();
        evt->type = (scancode & KBD_BREAK_MASK) ? KEY_UP : KEY_DOWN;

        //
        // Escapes
        //
        if(lastScancode == KBD_SCAN_ESCAPED)
        {
            // Gray modifier?
            if(isGray(scanvalue))
            {
                state |= KEYSTATE_GRAY;
            }
            else
            {
                state &= ~KEYSTATE_GRAY;
            }

            // Right modifer?
            if(scanvalue == KBD_SCAN_CTRL || scanvalue == KBD_SCAN_ALT)
            {
                state |= KEYSTATE_RIGHT;
            }
            else
            {
                state &= ~KEYSTATE_RIGHT;
            }
        }

        //
        // Shift state
        //
        if(scanvalue == KBD_SCAN_SHIFT_LEFT ||
           scanvalue == KBD_SCAN_SHIFT_RIGHT)
        {
            if(evt->type == KEY_DOWN)
            {
                state |= KEYSTATE_SHIFT;
            }
            else
            {
                // TODO: do not do this if caps lock is set
                state &= ~KEYSTATE_SHIFT;
            }

            if(lastScancode == KBD_SCAN_SHIFT_RIGHT)
            {
                state |= KEYSTATE_RIGHT;
            }
        }

        // Caps lock toggles shift state
        if(scanvalue == KBD_SCAN_CAPSLOCK)
        {
            if(evt->type == KEY_DOWN)
            {
                // Toggle states with caps lock
                state ^= KEYSTATE_SHIFT;

                // Set LED accordingly
                if(state & KEYSTATE_SHIFT)
                {
                    // FIXME: set only the caps led
                    setLED();
                }
                else
                {
                    // FIXME:
                    clearLED();
                }
            }
        }

        //
        // Alt state
        //
        if(scanvalue == KBD_SCAN_ALT)
        {
            if(evt->type == KEY_DOWN)
            {
                state |= KEYSTATE_ALT;
            }
            else
            {
                state &= ~KEYSTATE_ALT;
            }
        }

        //
        // Ctrl state
        //
        if(scanvalue == KBD_SCAN_CTRL)
        {
            if(evt->type == KEY_DOWN)
            {
                state |= KEYSTATE_CTRL;
            }
            else
            {
                state &= ~KEYSTATE_CTRL;
            }
        }

        // Select map index (assume unshifted subtable initially)
        int mapTableIndex = 0;

        // Use shift map?
        if(state & KEYSTATE_SHIFT)
        {
            mapTableIndex = KEYSTATE_SHIFT;
        }
        // Use AltGr map?
        else if(state & KEYSTATE_ALT &&
                state & KEYSTATE_RIGHT)
        {
            mapTableIndex = KEYSTATE_ALT;
        }

        /** Ctrl+Alt+Del reboots (yup, with no questions asked) */
        if(scanvalue == KBD_SCAN_KEYPAD_DEL &&
           state & KEYSTATE_ALT &&
           state & KEYSTATE_CTRL)
        {
            reboot();
        }

        /** Shift+Alt+Del gets out of graphics mode */
        if(scanvalue == KBD_SCAN_KEYPAD_DEL &&
           state & KEYSTATE_ALT &&
           state & KEYSTATE_SHIFT)
        {
            #ifdef BUILD_APOX_BUILDMODE_KERNEL
            if(GraphicsDriver::getCurrent())
                GraphicsDriver::getCurrent()->enable(false);
            #endif
        }

        if(kbdBaseUnicodeMapping[mapTableIndex][scanvalue] != -1)
            evt->character = unicodeToISO8859(kbdBaseUnicodeMapping[mapTableIndex][scanvalue]);
        else
            evt->character = -1;

        evt->unichar = kbdBaseUnicodeMapping[mapTableIndex][scanvalue];
        evt->scanvalue  = scanvalue;
        evt->state = state;

        // Remember last event
        memcpy(&lastEvent, evt, sizeof(KeyEvent));

        kassert(focusedComponent->kbdStream != null);
        kassert(focusedComponent->kbdStream->keyEvents != null);

        // Now there's one more key event available...
        queuePut(focusedComponent->kbdStream->keyEvents,
                 queueNewItem(evt));

        lastScancode = scancode;

        // Wake up any waiters
        focusedComponent->kbdStream->semEvents->release(false /* do not reschedule */);
    }

    return IRQRES_NORMAL;
}

/** Awaits any keyboard event. On timeout, NULL is returned (caller should check lastError) */
KeyEvent* getKey(long timeout/*=-1*/)
{
    Thread* currentThread = Thread::current();
    checkptr(currentThread);

    KeyEvent* evt = null;

    if(currentThread->component->kbdStream->semEvents->aquire(timeout) != E_TIMEOUT)
    {
        QueueEntryPtr node = queueGet(currentThread->component->kbdStream->keyEvents);
        if(node)
        {
            // Caller frees the returned event ptr
            evt = (KeyEvent*)node->object;

            delete node;
        }
    }

    return evt;
}

/**
 * Reads a printable character. Returns -SYSERR_NO_FOCAL_COMPONENT if an error occurs
 * and -SYSERR_TIMEOUT if it times out.
 *
 * @note Calling this method eats any UP events
 * @param timeout See Semaphore#aquire for more information.
 * @param Returns character or negated SYSERR
 */
int readchar(long timeout /* = -1*/)
{
    int res = -E_NO_FOCAL_COMPONENT;
    boolean isKeyDown = FALSE;

    Thread* currentThread = Thread::current();

    while(!isKeyDown || res == -1)
    {
        kassert(currentThread != null && currentThread->component != null &&
                currentThread->component->kbdStream != null &&
                currentThread->component->kbdStream->semEvents != null);

        // Wait until someone puts a key event on current threads' component input queue
        res = -currentThread->component->kbdStream->semEvents->aquire(timeout);

        if(res != E_OK)
        {
            break;
        }

        QueueEntryPtr node = queueGet(currentThread->component->kbdStream->keyEvents);

        // No focal component, probably...
        if(node == null)
        {
            kassert(false);
            break;
        }

        KeyEvent* evt = (KeyEvent*)node->object;
        checkptr(evt);

        res = evt->character;
        isKeyDown = (evt->type == KEY_DOWN);

        delete evt;
        delete node;
    }


    return res;
}

/**
 * Blocks until the kbd controllers is ready
 */
static void ctrlWait()
{
    while ((in8(0x64) & 1) != 0)
    {
        /*no-op*/
    }
}

void setLED()
{
    kbdWait();
    out8(0x60, (int) 0xED);
    in8(0x60); // await ack

    int led = 0;

    if(driver->state & KEYSTATE_SHIFT)
        led |= LED_CAPSLOCK;

    kbdWait();
    out8(0x60, (int) led);
    in8(0x60); // await ack
    kbdWait();
}

void clearLED()
{
    kbdWait();
    out8(0x60, (int) 0xED);
    in8(0x60); // await ack

    kbdWait();
    out8(0x60, (int) 0); // set all off :-)
    in8(0x60); // await ack
    kbdWait();
}

/**
 * Reboot the computer
 */
void reboot()
{
    CPU::disableNMI();
    CPU::localIrqOff();

    // Try 8042 kbd controller reset
    while ((in8(0x64) & KB_STATUS_INBUFF) != 0)
    {
        /*no-op*/
    }
    out8(0x64, 0xFE);

    // Try pci chipset reset
    out8(0xCF9, 0x02);
    out8(0xCF9, 0x06);

    // Try ps2 fast reset
    out8(0x92, 0x01);
}
