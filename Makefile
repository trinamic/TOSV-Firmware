# To rebuild project do "make clean" then "make all".

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.)
OUTDIR = $(RUN_MODE)

# is Bootloader support desired?
ifeq ($(FORMAT),hex)
	BOOT_LOADER?=TMCM-BL
else ifeq ($(FORMAT),binary)
	BOOT_LOADER?=TMCM-NO-BL
endif

# MCU name, submodel
# - MCU used for compiler-option (-mcpu)
# - SUBMDL used for linker-script name (-T) and passed as define
# - TARGET used for the output filename
# - ASRC (Make them always end in a capital .S. Files ending in a lowercase .s
#         will not be considered source files but generated files (assembler
#         output from the compiler), and will be deleted upon "make clean"!)
# - CPU_INC_DIR include path for the selected cpu

ifeq ($(CPU),F103)
	MCU = cortex-m3
	SUBMDL = STM32F103xB
	TARGET = TMC4671_TMC6100_TOSV_REF
	CPU_INC_DIR = cpu/STM32F103
	ASRC =  $(CPU_INC_DIR)/cortexm3_macro.s

	# set linker-script name depending on selected run-mode
	ifeq ($(BOOT_LOADER),TMCM-BL)
		LDFLAGS =-T $(CPU_INC_DIR)/stm32-tmcm.ld
	else
		LDFLAGS =-T $(CPU_INC_DIR)/stm32.ld
	endif
else ifeq ($(CPU),F205)
	MCU = cortex-m3
	SUBMDL = STM32F205RB
	TARGET = Startrampe-TOSV
	CPU_INC_DIR = cpu/STM32F205
	ASRC =  $(CPU_INC_DIR)/startup_stm32f2xx.S
	
	# set linker-script name depending on selected run-mode
	ifeq ($(BOOT_LOADER),TMCM-BL)
		LDFLAGS =-T $(CPU_INC_DIR)/stm32f2xx-tmcm_256k.ld
	else
		LDFLAGS =-T $(CPU_INC_DIR)/stm32f2xx_256k.ld
	endif
endif

# determine output file
ifeq ($(FORMAT),hex)
	TARGET_FILE = $(OUTDIR)/$(TARGET).hex 
else ifeq ($(FORMAT),binary)
	TARGET_FILE = $(OUTDIR)/$(TARGET).bin 
endif

CHIP = $(SUBMDL)

# Toolchain prefix (i.e arm-elf- -> arm-elf-gcc.exe)
TCHAIN_PREFIX = arm-none-eabi-
REMOVE_CMD = cs-rm

# YES enables -mthumb option to flags for source-files listed 
# in SRC and CPPSRC and -mthumb-interwork option for all source
USE_THUMB_MODE = YES

# RUN_MODE is passed as define and used for the linker-script filename,
# the user has to implement the necessary operations for 
# the used mode(s) (i.e. no copy of .data, remapping)
RUN_MODE = ROM_RUN

# Exception vectors in ROM:
VECTOR_TABLE_LOCATION = VECT_TAB_ROM

# list C source files here

# cpu specific C files

ifeq ($(CPU),F103)

	# cpu
	SRC += $(CPU_INC_DIR)/stm32f10x_bkp.c
	SRC += $(CPU_INC_DIR)/stm32f10x_can.c
	SRC += $(CPU_INC_DIR)/stm32f10x_flash.c
	SRC += $(CPU_INC_DIR)/stm32f10x_exti.c
	SRC += $(CPU_INC_DIR)/stm32f10x_adc.c
	SRC += $(CPU_INC_DIR)/stm32f10x_dma.c
	SRC += $(CPU_INC_DIR)/stm32f10x_gpio.c
	SRC += $(CPU_INC_DIR)/stm32f10x_nvic.c
	SRC += $(CPU_INC_DIR)/stm32f10x_pwr.c
	SRC += $(CPU_INC_DIR)/stm32f10x_rcc.c
	SRC += $(CPU_INC_DIR)/stm32f10x_tim.c
	SRC += $(CPU_INC_DIR)/stm32f10x_usart.c
	SRC += $(CPU_INC_DIR)/stm32f10x_spi.c
	SRC += $(CPU_INC_DIR)/stm32f10x_systick.c
	SRC += $(CPU_INC_DIR)/stm32f10x_it.c
	SRC += $(CPU_INC_DIR)/stm32f10x_vector.c
	
	# modules
	SRC += hal/modules/TMC4671-TMC6100-TOSV-REF_v1.0.c

else ifeq ($(CPU),F205)

	# cpu
	SRC += $(CPU_INC_DIR)/system_stm32f2xx.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_it.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_adc.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_can.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_dma.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_dac.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_exti.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_flash.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_gpio.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_i2c.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_iwdg.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_rcc.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_rtc.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_spi.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_syscfg.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_tim.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_usart.c
	SRC += $(CPU_INC_DIR)/stm32f2xx_wwdg.c
	SRC += $(CPU_INC_DIR)/misc.c
	SRC += $(CPU_INC_DIR)/usbd_core.c
	SRC += $(CPU_INC_DIR)/usbd_cdc_core.c
	SRC += $(CPU_INC_DIR)/usbd_ioreq.c
	SRC += $(CPU_INC_DIR)/usbd_req.c
	SRC += $(CPU_INC_DIR)/usb_core.c
	SRC += $(CPU_INC_DIR)/usb_dcd.c
	SRC += $(CPU_INC_DIR)/usb_dcd_int.c
	
	# modules
	SRC += hal/modules/Startrampe-TOSV_v1.0.c
	
endif

# hal parts
SRC += hal/system/Cpu.c
SRC += hal/system/SysTick.c
SRC += hal/system/Debug.c
SRC += hal/system/SystemInfo.c
SRC += hal/comm/Eeprom.c
SRC += hal/comm/SPI.c
SRC += hal/comm/UART.c
SRC += hal/comm/USB.c
SRC += hal/Flags.c

# general motor control and interfacing
SRC += TMCL.c
SRC += BLDC.c
SRC += main.c

# the Trinamic Open Source Ventilator module
SRC += TOSV.c

# TMC_API
SRC += TMC-API/tmc/helpers/Functions.c
SRC += TMC-API/tmc/ramp/LinearRamp.c
SRC	+= TMC-API/tmc/ic/TMC4671/TMC4671.c
SRC	+= TMC-API/tmc/ic/TMC6200/TMC6200.c

# List C source files here which must be compiled in ARM-Mode (no -mthumb).
# use file-extension c for "c-only"-files
## just for testing, timer.c could be compiled in thumb-mode too
SRCARM = 

# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
CPPSRC = 

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension .cpp for C++-files (not .C)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM = 

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM = 

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
#EXTRAINCDIRS  = $(CPU_INC_DIR)
EXTRAINCDIRS += ./hal
EXTRAINCDIRS += ./tmc
EXTRAINCDIRS  += ./TMC-API
EXTRAINCDIRS  += ./TMC-API/tmc/helpers

# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS = 

# Extra libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a: 
#    EXTRA_LIBS = xyz abc efsl
# for newlib-lpc (file: libnewlibc-lpc.a):
#    EXTRA_LIBS = newlib-lpc
EXTRA_LIBS =

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 2

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA, 
#  ihex to create a load-image in Intel hex format i.e. for lpc21isp
ifeq ($(BOOT_LOADER),TMCM-BL)
LOADFORMAT = ihex
else
LOADFORMAT = binary
endif

# Debugging format.
DEBUG = dwarf-2

# Place project-specific -D (define) and/or -U options for C here.
ifeq ($(BOOT_LOADER), TMCM-BL)
CDEFS += -DBOOTLOADER
endif

# Place project-specific -D and/or -U options for 
# Assembler with preprocessor here.
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

ifdef VECTOR_TABLE_LOCATION
CDEFS += -D$(VECTOR_TABLE_LOCATION)
ADEFS += -D$(VECTOR_TABLE_LOCATION)
endif

CDEFS += -D$(RUN_MODE) -D$(CHIP)
ADEFS += -D$(RUN_MODE) -D$(CHIP)

# Compiler flags.
ifeq ($(USE_THUMB_MODE),YES)
THUMB    = -mthumb
### no for CM3 THUMB_IW = -mthumb-interwork
else 
THUMB    = 
THUMB_IW = 
endif

# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS =  -g$(DEBUG)
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU) $(THUMB_IW) 
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without longcall:
##CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrapper is used 
##CFLAGS += -mapcs-frame 
##CFLAGS += -fomit-frame-pointer
#CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wall -Wextra
CFLAGS += -Wimplicit -Wcast-align -Wpointer-arith
CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
#CFLAGS += -pedantic
CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d

# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-elf-g++)
CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS = 

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS  = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -Wl,--gc-sections,-Map=$(OUTDIR)/$(TARGET).map,-cref
LDFLAGS += -u,Reset_Handler
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += -lc
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))
LDFLAGS += $(MATH_LIB)
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += -lc -lgcc 
LDFLAGS += --specs=nosys.specs  --specs=nano.specs

# syscalls meaning:
#--specs=nosys.specs: no syscalls available (no syscalls.c needed)
#--specs=nano.specs: uses smaller Newlibc version

# Define programs and commands.
SHELL   = sh
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
REMOVE  = $(REMOVE_CMD) -f

# Define Messages
MSG_ERRORS_NONE = Errors: none
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_COMPILING = "**** Compiling C :"
MSG_COMPILING_ARM = "**** Compiling C (ARM-only):"
MSG_COMPILINGCPP = "Compiling C++ :"
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
#MSG_ASSEMBLING = "**** Assembling:"
MSG_ASSEMBLING_ARM = "****Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_LPC21_RESETREMINDER = You may have to bring the target in bootloader-mode now.
MSG_ASMFROMC = "Creating asm-File from C-Source:"
MSG_ASMFROMC_ARM = "Creating asm-File from C-Source (ARM-only):"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

elf: $(OUTDIR)/$(TARGET).elf
lss: $(OUTDIR)/$(TARGET).lss 
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin

# Default target.
all: begin gccversion build sizeafter end

ifeq ($(LOADFORMAT),ihex)
build: elf hex lss sym
else
ifeq ($(LOADFORMAT),binary)
build: elf bin lss sym
else 
ifeq ($(LOADFORMAT),both)
build: elf hex bin lss sym
else 
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif

begin:
	@echo -
	@echo ----- Parameter passed to Makefile -----
	@echo FORMAT = $(FORMAT)
	@echo CPU = $(CPU)
	@echo FLASH_SIZE = $(FLASH_SIZE)
	@echo ----- Intern used parameter -----
	@echo BOOT_LOADER = $(BOOT_LOADER)
	@echo MCU = $(MCU)
	@echo SUBMDL = $(SUBMDL)
	@echo TARGET_FILE = $(TARGET_FILE)
	@echo CPU_INC_DIR = $(CPU_INC_DIR)
	@echo ASRC = $(ASRC)
	@echo LDFLAGS = $(LDFLAGS)
	@echo CDEFS = $(CDEFS)

end:
	@echo $(MSG_END)

# Display sizes of sections.
ELFSIZE = $(SIZE) -A  $(OUTDIR)/$(TARGET).elf

sizeafter:
	@echo $(MSG_SIZE_AFTER)
	$(ELFSIZE)
	
# Display compiler version information.
gccversion : 
	@echo ----- gcc version -----
	@$(CC) --version
	@echo ---All objects:---
	@echo $(ALLOBJ)
	@echo ---

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O ihex $< $@
	
# Create final output file (.bin) from ELF output file.
%.bin: %.elf
	@echo $(MSG_LOAD_FILE) $@
	$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C -r $< > $@
#	$(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ)
	@echo .
	@echo ----- Linking: $@ -----
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)

# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo .
	@echo ----- Assembling $$< "->" $$@ -----
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_ASSEMBLING_ARM) $$< "->" $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo .
	@echo ----- Compiling $$< "->" $$@ -----
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)) ) )

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP) $$< "->" $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP_ARM) $$< "->" $$@
	$(CC) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src)))) 

# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC) $< to $@
	$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTDIR)/*.o
	$(REMOVE) $(OUTDIR)/*.lst
	$(REMOVE) $(OUTDIR)/dep/*.d
	$(REMOVE) $(OUTDIR)/*.map
	$(REMOVE) $(OUTDIR)/*.sym
	$(REMOVE) $(OUTDIR)/*.lss
	$(REMOVE) $(OUTDIR)/*.elf
	$(REMOVE) $(ALLOBJ)
	$(REMOVE) $(LSTFILES)
	$(REMOVE) $(DEPFILES)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(CPPSRC:.cpp=.s)
	$(REMOVE) $(CPPSRCARM:.cpp=.s)
	
# Create output files directory
$(shell mkdir $(OUTDIR) 2>NUL)

# Include the dependency files.
-include $(shell mkdir $(OUTDIR)/dep 2>NUL) $(wildcard $(OUTDIR)/dep/*)

# Listing of phony targets.
.PHONY : all begin finish end sizeafter gccversion \
build elf hex bin lss sym clean clean_list program
