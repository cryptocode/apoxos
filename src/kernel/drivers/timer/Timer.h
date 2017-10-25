/*
 * Apox Operating System
 * Copyright (c) 2005,2006,2007,2008 cryptocode
 */

#ifndef _APOX_TIMER_H_
#define _APOX_TIMER_H_

#include <kernel/init/irq.h>
#include <kernel/libc/std.h>
#include <kernel/adt/Queue.h>
#include <kernel/adt/BinTree.h>
#include <kernel/drivers/Device.h>
#include <kernel/vmm/ObjectCache.h>

/**
 * Initial heart beat.
 */
#define INITIAL_HZ  1000

struct Alarm;

/** System timer driver*/
class Timer : public DeviceDriver, public Allocator
{
public:

    Timer()
    {}

    /**
     * Create and initialize the timer
     */
    static Timer* initialize();

    /** Get timer singleton */
    static Timer* getInstance();

    /**
     * Convert milliseconds to timer ticks
     *
     * @param ms Milliseconds
     * @return Number of timer ticks in the specified amount of time
     */
    static inline uint64 msToTicks(uint64 ms)
    {
        return ms;
    }

    /**
     *  Get 32-bit tick count since boot.
     *
     *  Depending on HZ, this may overflow in a matter of days.
     */
    static uint32 getTickCount32();

    /**
     *  Get 64-bit tick count since boot.
     */
    static uint64 getTickCount64();

    /**
     * Timer interrupt handler
     * @see DeviceDriver#onInterrupt
     */
    virtual int onInterrupt(ContextState* ctx);

    // List of alarms, sorted on time
    QueuePtr alarmQueue;

    /** \brief End-of-queue marker; used when traversing and re-inserting items */
    QueueEntryPtr eoqMarker;

    /**
     * Add a timer
     *
     * @param period Time in millisecond
     * @param onAlarm Function to call when time is due
     * @return Timer.
     */
    static Alarm* addTimer(uint64_t period, void (*onAlarm)(void*), void* param);

    /**
     * Cancels the specific timer.
     *
     * Note that this does not quarantee that the timer will not be fired, since the timer
     * subsystem may be in the process of sending the alarm when this call is made.
     */
    static void cancelTimer(struct Alarm* timer);

public:

    /**
     * @see Allocator#allocateObject
     */
    void* allocateObject(void* param, long flags);

    /**
     * @see Allocator#deallocateObject
     */
    void deallocateObject(void* obj, long flags);

private:

    /**
     * Change heartbeat by reprogramming the timer
     */
    static void setHertz(int hertz);

    /**
     * Cache of nodes
     */
    TypedObjectCache<BinTreeNode<Alarm*> > nodeCache;

    /**
     * Cache of Alarm objects
     */
    TypedObjectCache<Alarm> alarmCache;

    /**
     * Alarms; the next due timer is available in O(1) using this
     * ADT.
     */
    BinTree<Alarm*> alarms;

    /** Timer singleton */
    static Timer* instance;
};

/** Alarm object */
struct Alarm
{
    /** The due tick value */
    uint64 dueTick;

    /**
     *  Alarm callback function. If this is null, the alarm is
     *  considered cancelled.
     */
    void (*onAlarm)(void*);

    /** Argument to alarm callback **/
    void* param;

    /** Cancels the timer */
    void cancel()
    {
        Timer::getInstance()->cancelTimer(this);
    }
};

#endif // _APOX_TIMER_H_
