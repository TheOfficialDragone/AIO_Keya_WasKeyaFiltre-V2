void udpNtrip()
{
#ifdef ARDUINO_TEENSY41
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  unsigned int packetLength = Eth_udpNtrip.parsePacket();
  
  if (packetLength > 0)
  {
    int got = Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, sizeof(Eth_NTRIP_packetBuffer));
    if (got > 0) SerialGPS->write(Eth_NTRIP_packetBuffer, got);
  }
 #endif
}
