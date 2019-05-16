
The purpose of this repository is verify CyloneTCP.  This requires CycloneTCP 1.9.2.  It should be in CycloneTCP_SSL_Crypto_Open_1_9_2/demo/microchip/pic32mz_ef_starter_kit/snmp_agent_demo

I am using a PIC32MZ2048EFH144 on a EK starter kit.  Generally speaking, there's some issues that cause debug.h trickle problems through the code due to redefinitions between HTONS and other things in <stdio.h>.  I was not given the privilege of time to fix things properly, so I just added #ifdef PUNIXDEBUG to files around TRACE macros.  

Changes that were required:
I updated the Makefile-local-default.mk to have a selectable path.
PREFIX_IDE=/Applications/microchip/mplabx/v4.01
PREFIX_XC32=/Applications/microchip/xc32/v1.44


The file pic32mz_ef_starter_kit.h was copied into the src directory from third_party/microchip/boards/pic32mz_ef_starter_kit/pic32mz_ef_starter_kit.h

The file endian.c was not available in CycloneTCP 1.9.2; however, cpu_endian.c seemed to be the replacement.  Dependencies were updated in the Makefile-default.mk file.

drivers/pic32mz_eth.c was not available, and the Makefile was updated to be drivers/mac/pic32mz_eth_driver.c


UPDATE:  just made a null debug.h to solve the issues below with #include <stdint.h> in it.  The debug.c was including debug.h and getting it from somewhere.
Something was not correctly including HTONS macros correctly, but instead of correcting it and including cpu_endian.c, I just did the following modifications in arp.c, igmp.c, ipv6_misc.c, mld.c, tcp_misc.c

#ifdef _CPU_BIG_ENDIAN
#define HTONS(value) (value)
#define HTONL(value) (value)
#define htons(value) ((uint16_t) (value))
#define htonl(value) ((uint32_t) (value))
#else
#define SWAPINT16(x) ( \
   (((uint16_t)(x) & 0x00FFU) << 8) | \
   (((uint16_t)(x) & 0xFF00U) >> 8))
#define SWAPINT32(x) ( \
   (((uint32_t)(x) & 0x000000FFUL) << 24) | \
   (((uint32_t)(x) & 0x0000FF00UL) << 8) | \
   (((uint32_t)(x) & 0x00FF0000UL) >> 8) | \
   (((uint32_t)(x) & 0xFF000000UL) >> 24))
#define HTONS(value) SWAPINT16(value)
#define HTONL(value) SWAPINT32(value)
#define htons(value) swapInt16((uint16_t) (value))
#define htonl(value) swapInt32((uint32_t) (value))

#endif

CycloneTCP specific:

Removed ../../../../../cyclone_tcp/snmp/snmp_common.c  and ../../../../../cyclone_tcp/snmp/snmp_usm.c references from Makefile-default.mk as per instructions in https://www.oryx-embedded.com/pdf/CycloneTCP_Migration_Guide.pdf
and updated new files as per page 8.  

The type int32_t is used everywhere, but is not a standard type.  I used the BSD style command to replace those in the CycloneTCP library:
LC_ALL=C find ./ -type f -exec sed -i '' -e 's/int_t/int32_t/g' {} \;
LC_ALL=C find ./ -type f -exec sed -i '' -e 's/uint_t/uint32_t/g' {} \;

../../../../../cyclone_tcp/core/bsd_socket.c removed the debug.h include




