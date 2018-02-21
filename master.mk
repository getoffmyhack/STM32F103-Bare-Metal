################################################################################
#####
##### Toolchain paths
#####
##### These paths need to point to the location of your tools installation.
#####
##### BUILDTOOLPATH : The location where the ARM-GCC binaries are located
##### FLASHTOOLPATH : The location where the tools used for flashing are located,
#####				  st-flash, stm32flash, stm32loader, openocd, etc.
#####

BUILDTOOLPATH	= /usr/local/arm-none-eabi/bin
FLASHTOOLPATH	= /opt/local/bin

################################################################################
#####
##### CMSIS / Device Directory Layout
#####
##### Paths and locations of the CMSIS drivers from ST.
##### 
##### See https://github.com/getoffmyhack/STM32-CMSIS

CMSISBASE 		= ../CMSIS
CMSISINC 		= $(CMSISBASE)/include

# Device specific, change to either IF-THEN or makefile includes
DEVICEINC 		= $(CMSISBASE)/Device/STM32F1xx/include
DEVICESRC	 	= $(CMSISBASE)/Device/STM32F1xx/src

DEVICESTARTUP 	= startup_stm32f103xb.s
DEVICELINKER 	= $(CMSISBASE)/Device/STM32F1xx/linker/STM32F103XB_FLASH.ld

################################################################################
#####
##### Local Project Directory Layout
#####

SRCDIR = src
BINDIR = bin
OBJDIR = obj
INCDIR = include

################################################################################
#####
##### C and ASM source files
#####

SRC = $(wildcard $(SRCDIR)/*.c) $(wildcard $(DEVICESRC)/*.c)
ASM = $(wildcard $(SRCDIR)/*.s) $(DEVICESRC)/$(DEVICESTARTUP)

################################################################################
#####
##### include files
#####

INCLUDE  = -I$(INCDIR)
INCLUDE += -I$(CMSISINC)
INCLUDE += -I$(DEVICEINC)

################################################################################
#####
##### compiler flags
#####

CFLAGS   = -std=c99 
CFLAGS	+= -Wall 
CFLAGS	+= -fno-common 
CFLAGS	+= -mthumb 
CFLAGS	+= -mcpu=$(ARMCPU) 
CFLAGS	+= -D$(STM32MCU) 
CFLAGS	+= -g 
CFLAGS	+= -Wa,-ahlms=$(addprefix $(OBJDIR)/,$(notdir $(<:.c=.lst)))
CFLAGS	+= $(INCLUDE)

################################################################################
#####
##### linker flags
#####

LDFLAGS  = -T$(DEVICELINKER)
LDFLAGS	+= -mthumb 
LDFLAGS	+= -mcpu=$(ARMCPU)
#LDFLAGS	+= -specs=rdimon.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += --specs=nano.specs
LDFLAGS += -lc
#LDFLAGS += -lrdimon

################################################################################
#####
##### assembler flags
#####

ASFLAGS += -mcpu=$(ARMCPU)

################################################################################
#####
##### tools
#####

CC 				= $(BUILDTOOLPATH)/arm-none-eabi-gcc
AS 				= $(BUILDTOOLPATH)/arm-none-eabi-as
AR 				= $(BUILDTOOLPATH)/arm-none-eabi-ar
LD 				= $(BUILDTOOLPATH)/arm-none-eabi-ld
GDB 			= $(BUILDTOOLPATH)/arm-none-eabi-gdb
GDBPY 			= $(BUILDTOOLPATH)/arm-none-eabi-gdb-py
OBJCOPY 		= $(BUILDTOOLPATH)/arm-none-eabi-objcopy
SIZE 			= $(BUILDTOOLPATH)/arm-none-eabi-size
OBJDUMP 		= $(BUILDTOOLPATH)/arm-none-eabi-objdump
SWDFLASH 		= $(FLASHTOOLPATH)/st-flash
SERFLASH 		= $(FLASHTOOLPATH)/stm32flash
RM 				= rm -rf

## Build process

OBJ := $(addprefix $(OBJDIR)/,$(notdir $(SRC:.c=.o)))
OBJ += $(addprefix $(OBJDIR)/,$(notdir $(ASM:.s=.o)))

all: $(BINDIR)/$(PROJECT).bin

swdflash: $(BINDIR)/$(PROJECT).bin
	$(SWDFLASH) write $(BINDIR)/$(PROJECT).bin 0x8000000

serflash: $(BINDIR)/$(PROJECT).bin
	@read -p "Press --RESET-- with Boot0 = 1 and press <CR>" dummy;
	$(SERFLASH) -b 230400 -w $(BINDIR)/$(PROJECT).bin -g 0 /dev/tty.usbserial-*

debug: $(BINDIR)/$(PROJECT).bin
	$(GDBPY) --command $(BASEPATH)/gdb/debug.gdb $(BINDIR)/$(PROJECT).elf

macros:
	$(CC) $(GCFLAGS) -dM -E - < /dev/null

clean:
	$(RM) $(BINDIR)
	$(RM) $(OBJDIR)

################################################################################
#####
##### Compile / assemble project files
#####

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<

################################################################################
#####
##### Compile / assemble CMSIS and Device files
#####

$(OBJDIR)/%.o: $(DEVICESRC)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(DEVICESRC)/%.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<

################################################################################
#####
##### Link binaries
#####

$(BINDIR)/$(PROJECT).hex: $(BINDIR)/$(PROJECT).elf
	$(OBJCOPY) -R .stack -O ihex $(BINDIR)/$(PROJECT).elf $(BINDIR)/$(PROJECT).hex

$(BINDIR)/$(PROJECT).bin: $(BINDIR)/$(PROJECT).elf
	$(OBJCOPY) -R .stack -O binary $(BINDIR)/$(PROJECT).elf $(BINDIR)/$(PROJECT).bin

$(BINDIR)/$(PROJECT).elf: $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(OBJ) $(LDFLAGS) -o $(BINDIR)/$(PROJECT).elf
	$(OBJDUMP) -D $(BINDIR)/$(PROJECT).elf > $(BINDIR)/$(PROJECT).lst
	$(SIZE) $(BINDIR)/$(PROJECT).elf	

