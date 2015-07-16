################################################################################
# Makefile for the LightCCB_STM32F429_Firmware project.
#
# Created by Duong Vuong.
#
# For use on the Light_CCB board, uses the GCC toolchain for ARM. 
################################################################################

################################################################################
#
# Compile target definition
#
################################################################################
# Derive the project name from the current folder.
ifeq ($(TARGET),)
TARGET        = $(OBJDIR)/SPI_Nucleo
endif
# Set up stack and heap sizes & FPU Usage.
USE_HARD_FPU   = 1
HEAP_SIZE      = 8192
STACK_SIZE     = 1024
MCU  		   = cortex-m4
BOARD		   = NUCLEO
# Start of default section.
# List all default C defines here, like _DEBUG=1. -D is automatically added.
DCDEFS+=USE_STDPERIPH_DRIVER

DCDEFS+=STM32F401xx

DCDEFS+=USE_FULL_ASSERT
DCDEFS+=HSE_VALUE=8000000
DCDEFS+=__HEAP_SIZE=$(HEAP_SIZE)
DCDEFS+=__STACK_SIZE=$(STACK_SIZE)
# List all default ASM defines here, like _DEBUG=1 -D is automatically added.
DADEFS+= 
# List all default directories to look for include files here
DINCDIR+= $(PERIPH_DRIVER)/inc
DINCDIR+= inc
DINCDIR+= spi
DINCDIR+= led
DINCDIR+= i2c
DINCDIR+= usart
DINCDIR+= fifo
DINCDIR+= Libraries/CMSIS/Include
DINCDIR+= Libraries/FreeRTOS/Source/include
DINCDIR+= Libraries/FreeRTOS/Source/portable/GCC/ARM_CM4F
# List the default directory to look for the libraries here
DLIBDIR =
# List all default libraries here
DLIBS = 

################################################################################

################################################################################
#
# Source directories
#	Directory source code of LightCCB firmware
#
################################################################################
# Relative path to LightCCB-FW root.
ifeq ($(LIGHTCCB_FW),)
LIGHTCCB_FW=
endif
# Standard Peripheral Driver Path
PERIPH_DRIVER=Libraries/STM32F4xx_StdPeriph_Driver
# Compiled Output Path
OBJDIR = Output
LSTDIR = Lst

#
# Create output directories
#
$(shell mkdir -p $(OBJDIR)/$(LSTDIR) 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Libraries/Startup 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/led 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/spi 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/i2c 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/fifo 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/usart 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(PERIPH_DRIVER)/src 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/src 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Libraries/FreeRTOS/Source 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Libraries/FreeRTOS/Source/portable/GCC/ARM_CM4F 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Libraries/FreeRTOS/Source/portable/MemMang 2>/dev/null)
# Include the dependency files
$(wildcard $(OBJDIR)/*.d)

################################################################################
#
#	Specifies compiler settings for LightCCB board
#
################################################################################

#
# Linker Scripts.
#
LDSCRIPT = Libraries/ldscripts/stm32f401xx_flash.ld
#
# ASM source files here
#
ASRC+= Libraries/Startup/startup_stm32f401xx.s
#
# C source files here
#




# List of all the board related files.
SRC+= $(PERIPH_DRIVER)/src/misc.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_gpio.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_i2c.c
SRC+= src/stm32f4xx_it.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_rcc.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_usart.c
SRC+= src/system_stm32f4xx.c
SRC+= src/main.c
SRC+= led/led.c
SRC+= spi/spi.c
SRC+= i2c/i2c.c
SRC+= fifo/fifo.c
SRC+= usart/usart.c
SRC+= Libraries/FreeRTOS/Source/tasks.c
SRC+= Libraries/FreeRTOS/Source/queue.c
SRC+= Libraries/FreeRTOS/Source/list.c
SRC+= Libraries/FreeRTOS/Source/timers.c
SRC+= Libraries/FreeRTOS/Source/portable/MemMang/heap_1.c
SRC+= Libraries/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_usart.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_flash.c
SRC+= $(PERIPH_DRIVER)/src/stm32f4xx_spi.c

################################################################################

################################################################################
#
#	Compiler settings for ARM cortex-m4
#
################################################################################
## Compiler programs.

TRGT = /opt/toolchains/eabi/arm-2014.05/bin/arm-none-eabi-

# Program names.
CC		= $(TRGT)gcc
CP		= $(TRGT)objcopy
AS		= $(TRGT)gcc -x assembler-with-cpp
OD		= $(TRGT)objdump
SZ  	= $(TRGT)size
NM  	= $(TRGT)nm
OPENOCD = openocd
DOXYGEN	= doxygen

# Compile Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
# 
OPT = 3

################################################################################

################################################################################
#
#	Flags
#		Linker, compiler, assembler flags for LightCCB board..
#
################################################################################

#
# Define FPU settings here
#
ifeq ($(USE_HARD_FPU), 0)
# Specifying ‘soft’ causes GCC to generate output containing library calls for floating-point operations
FPU+= -mfloat-abi=soft
else
# ‘Hard’ allows generation of floating-point instructions and uses FPU-specific calling conventions.
FPU+=-mfloat-abi=hard 
# Specifies what floating-point hardware (or hardware emulation) is available on the target. 
FPU+=-mfpu=fpv4-sp-d16
# __FPU_USED sets registry to enable FPU.
FPU+=-D__FPU_USED=1
endif

#
# Add prefixe to defines
#
INCDIR  = $(patsubst %,-I%,$(sort $(DINCDIR) $(UINCDIR)))
LIBDIR  = $(patsubst %,-L%,$(sort $(DLIBDIR) $(ULIBDIR)))

#
# Add -D prefix to defines.
#
CDEFS=$(patsubst %,-D%,$(sort $(DCDEFS) $(UCDEFS)))
ADEFS=$(patsubst %,-D%,$(sort $(DADEFS) $(UADEFS)))

#
# Combine Libraries. 
#
LIBS   +=$(DLIBS)
LIBS   +=$(ULIBS)

#
# MCU Flags
#
MCUFLAGS+=-mthumb
MCUFLAGS+=-mcpu=$(MCU)
MCUFLAGS+=$(FPU)

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler
#  -adhlns:   create listing
#----------------------------------------------------

#
# Assembly Flags
#
ASFLAGS+=$(MCUFLAGS) 
ASFLAGS+=$(ADEFS)
ASFLAGS+=-Wa,-amhls=$(OBJDIR)/$(LSTDIR)/$(*F).lst
ASFLAGS+=-O$(OPT)

#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#----------------------------------------------------

#
# C Flags
#
CPFLAGS+=$(MCUFLAGS)
CPFLAGS+=$(CDEFS)
CPFLAGS+=-O$(OPT)
CPFLAGS+=-Wall
CPFLAGS+=-std=gnu99
CPFLAGS+=-gdwarf-2
#CPFLAGS+=-Wextra
#CPFLAGS+=-Wpointer-arith
#CPFLAGS+=-Wstrict-prototypes
#CPFLAGS+=-Winline
#CPFLAGS+=-Wunreachable-code
#CPFLAGS+=-Wundef
CPFLAGS+=-Wstrict-prototypes
CPFLAGS+=-ffunction-sections
CPFLAGS+=-fdata-sections
CPFLAGS+=-Wa,-adhlns=$(OBJDIR)/$(LSTDIR)/$(*F).lst
# Optimize use of the single-precision FPU
#
CPFLAGS+=-fsingle-precision-constant

CPFLAGS+=-MD
CPFLAGS+=-MP
CPFLAGS+=-MF
CPFLAGS+=$(OBJDIR)/$(LSTDIR)/$(*F).d

#---------------- Linker Options --------------------
#  -Wl,...:     tell GCC to pass this to linker
#    -Map:      create map file
#    --cref:    add cross reference to  map file
#----------------------------------------------------

#
# Liner Flags
#
LDFLAGS+=$(MCUFLAGS)
#LDFLAGS+=-nostartfiles
LDFLAGS+=-T$(LDSCRIPT)
LDFLAGS+=-Wl,-Map=$(TARGET).map,--cref,--gc-sections,--no-warn-mismatch
LDFLAGS+=$(LIBDIR)

################################################################################

################################################################################
#
# Paths
#	Paths for LightCCB board
#
################################################################################
## Flashing Program & Flags
FLASH      = ~/stlink/st-flash
FLASHFLAGS+= write $(TARGET).bin 0x08000000

################################################################################

################################################################################
#
# Makefile targets.
#	Targets for LightCCB board
#
################################################################################

#
# Get objects 
#
OBJS	+= $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SRC))))
OBJS	+= $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(ASRC))))

# Make all
all: gccversion build showsize

# Build process
build: elf hex lss sym bin

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym

# Creating Doxygen documentation
doxygen:
	@echo
	@echo Creating Doxygen documentation
	@$(DOXYGEN)
	
# Display compiler version information
gccversion: 
	@$(CC) --version

# Show the final program size
showsize: elf
	@echo
	@$(SZ) $(TARGET).elf 2>/dev/null

# Compile: create object files from C source files
$(OBJDIR)/%.o : %.c Makefile
	@echo
	@echo Compiling C: $<
	$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@

# Assemble: create object files from assembler source files
$(OBJDIR)/%.o : %.s
	@echo
	@echo Assembling: $<
	$(AS) -c $(ASFLAGS) $< -o $@
	
# Link: create ELF output file from object files
.SECONDARY: $(TARGET).elf
.PRECIOUS:  $(OBJS)
$(TARGET).elf: $(OBJS)
	@echo
	@echo Linking: $@
	$(CC) $^ $(LDFLAGS) $(LIBS) -o $@

# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo Creating hex file: $@
	$(CP) -O ihex $< $@

# Create bin file :
%.bin: %.elf
	@echo
	@echo Creating bin file: $@
	$(CP) -O binary $< $@

# Flash hex file to the board.
flash: $(TARGET).hex
	@echo Flashing $(TARGET)
	$(FLASH) $(FLASHFLAGS)

clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)

# Create extended listing file from ELF output file
%.lss: %.elf
	@echo
	@echo Creating Extended Listing: $@
	$(OD) -h -S -z $< > $@

# Create a symbol table from ELF output file
%.sym: %.elf
	@echo
	@echo Creating Symbol Table: $@
	$(NM) -n $< > $@

# Listing of phony targets
.PHONY: all build flash clean \
        doxygen elf lss sym \
        showsize gccversion
################################################################################