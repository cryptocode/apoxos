/*
 * Apox Operating System
 * Copyright (c) 2006, cryptocode
 *
 * $Id$
 */

#include <kernel/drivers/nic/NIC.h>

/** C'tor */
NetworkInterface::NetworkInterface()
{
    mtu = 0;
}

/** C'tor */
NetworkInterface::NetworkInterface(IPAddress& ipAddress)
{
    // Make a copy of the IP address and add it to the list of assigned ip's. This
    // list can be extended later on.
    ipAddresses.add(new IPAddress(ipAddress));
}

/** Get list of IP addresses assigned to this interface. Mutable. */
List<IPAddress*>& NetworkInterface::getIPAddresses()
{
    return ipAddresses;
}

/** Gets the MTU of the interface */
int NetworkInterface::getMTU()
{
    return mtu;
}

/** Sets the MTU of the interface */
void NetworkInterface::setMTU(int mtu)
{
    this->mtu = mtu;
}
