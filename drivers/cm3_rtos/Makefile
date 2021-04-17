#! gmake

#
# Copyright (C) 2014 François Beaulier
# 
# This file is part of canfestival, a library implementing the canopen
# stack
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
# 

CC = arm-none-eabi-gcc
PROG_CFLAGS =  -fPIC

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

LIB_ROOT = ../../

INCLUDES = -I../../include \
		   -I../../include/cm3_rtos \
           -I$(LIB_ROOT)/libopencm3/include/ \
		   -I./rtos \
		   -I.

# For medium density F103           
DEFINES = -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

# For Connectivity line F105, external clock with 8Mhz crystal
#DEFINES = -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000ul

CFLAGS = $(PROG_CFLAGS) $(ARCH_FLAGS) $(DEFINES)

OBJS = cm3.o 

all: driver

driver: $(OBJS)

libcandriver.a: $(OBJS)
	@echo " "
	@echo "*********************************************"
	@echo "**Building libcandriver for arm cortex-M3
	@echo "*********************************************"
	ar rc $@ $(OBJS)
	ranlib $@

%.o: %.c
	@echo " "
	@echo "*********************************************"
	@echo "**Compiling $< -> $@"
	@echo "*********************************************"
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

clean:
	rm -f $(OBJS)
	rm -f $(LIB_ROOT)bin/libcanfestival.a
	rm -f $(LIB_ROOT)bin/libcandriver.a

mrproper: clean
