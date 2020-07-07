- Before compiling, change `ioLibrary_Driver/Ethernet/wizchip_conf.h` to select the correct chip
- `sudo arp-scan --interface=enp2s0 192.168.2.0/24` # replace enp2s0 with your own ethernet interface
  This should show
  ```
  WARNING: Could not obtain IP address for interface enp2s0. Using 0.0.0.0 for
  the source address, which may not be what you want.
  Either configure enp2s0 with an IP address, or manually specify the address
  with the --arpspa option.
  Interface: enp2s0, type: EN10MB, MAC: <redacted>, IPv4: (none)
  Starting arp-scan 1.9.7 with 256 hosts (https://github.com/royhills/arp-scan)
  192.168.2.165   00:08:dc:ab:cd:ef       Wiznet

  1 packets received by filter, 0 packets dropped by kernel
  Ending arp-scan 1.9.7: 256 hosts scanned in 2.018 seconds (126.86 hosts/sec). 1 responded
  ```
- `sudo ip addr add 192.168.2.1 dev enp2s0`
- `sudo ip route add 192.168.2.165/32 via 192.168.2.1`

- go to gdb, right after the socket is opened (RetargetInit), connect to it by `telnet 192.168.2.165 5000`
  - using newlib in ubuntu20.04 package (`/usr/lib/arm-none-eabi/newlib/`) leads to weird problem
      - retarget-tcp.c:73 in `_write` errno=EBADF will always hit and the error isn't reset if the connection isn't established before a `printf`, code to wait on connection is added to address this
      - building newlib-3.1.0 works

