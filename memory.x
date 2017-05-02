MEMORY
{
  FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 2048K
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 192K
  CCMRAM (rw)      : ORIGIN = 0x10000000, LENGTH = 64K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);