# Custom bootloader
A custom bootloader for flashing, erasing, running code from any address and many other things.

## Supported commands:
- BL_GET_VER : 0x51
- BL_GET_CID : 0x53
- BL_GET_RDP_STATUS : 0x54
- BL_GO_TO_ADDR : 0x55
- BL_FLASH_ERASE : 0x56
- BL_MEM_WRITE : 0x57
- BL_MEM_READ : 0x59
- BL_READ_SECTOR_STATUS : 0x5A

There is a host application that can be used to correctly run the commands

The Board used for testing is https://github.com/mcauser/BLACK_F407ZG

To run bootloader K0 should be pressed during reset.