#!/bin/bash
####################################################################################
# flash.sh
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
####################################################################################

USAGE="$0 <nuttx-path>"
echo "Assumptions:"
echo ""
echo "  - Windows 7"
echo "  - LPCXpresso 4.6 installed at /cygdrive/c/nxp/lpcxpresso_3.6"
echo "  - AXF image built with Code Red"
echo "  - LPC17xx"
echo ""
echo "You will need to edit this is any of the above are false"

# This is the default install location for binaries on Linux
#BINDIR=/usr/local/LPCXpresso/bin/dfu-util

# This is the default install location for binaries on Windows (note that this
# path could change with the Code Red version number
BINDIR=/cygdrive/c/nxp/lpcxpresso_3.6/bin

# This is the default install location for DFUAPP.exe on Windows
DFUAPP="$BINDIR/DFUAPP.exe"

# ROM image for resetting LPC-Link
# ROM=LPCXpressoWIN.enc # WinUSB
# ROM=LPCXpressoFS.enc # Win2000
# ROM=LPCXpressoFS.enc # WinXP
# ROM=LPCXpressoFS.enc # Win2003
# ROM=LPCXpressoHS.enc # WinVista
ROM=LPCXpressoHS.enc # Win7

ROMPATH=`cygpath -w "$BINDIR/$ROM"`

# FLASHUTIL="$BINDIR/crt_emu_lpc11_13" # for LPC11xx or LPC13xx parts)
FLASHUTIL="$BINDIR/crt_emu_cm3_nxp"  # for LPC17xx parts
# FLASHUTIL="$BINDIR/crt_emu_a7_nxp"   # for LPC21/22/23/24 parts)
# FLASHUTIL="$BINDIR/crt_emu_a9_nxp"   # for LPC31/32 and LPC29xx parts)
# FLASHUTIL="$BINDIR/crt_emu_cm3_lmi"  # for TI Stellaris LM3S parts

if [ ! -x "$FLASHUTIL" ]; then
	echo "No executable file at ${FLASHUTIL}"
	exit 1
fi

# unset WIRE          # for Red Probe+, Red Probe, RDB1768v1, or TI Stellaris evaluation boards
# WIRE="-wire=hi"     # for RDB1768v2 without upgraded firmware)
# WIRE="-wire=winusb" # for RDB1768v2 with upgraded firmware)
# WIRE="-wire=winusb" # for LPC-Link on Windows XP)
WIRE="-wire=hid"      # for LPC-Link on Windows Vista/ Windows 7)

TARGET=LPC1768
#TARGET=NXP_dir_part_LPC17

# The nuttx directory must be provided as an argument

NUTTX=$1
if [ -z "${NUTTX}" ]; then
	echo "Missing argument"
	echo $USAGE
	exit 1
fi

if [ ! -d "${NUTTX}" ]; then
	echo "Directory ${NUTTX} does not exist"
	echo $USAGE
	exit 1
fi

# The binary to download:

if [ ! -f "${NUTTX}/nuttx.axf" ]; then
	if [ -f "${NUTTX}/nuttx" ]; then
		echo "Renaming ${NUTTX}/nuttx to ${NUTTX}/nuttx.axf"
		mv ${NUTTX}/nuttx ${NUTTX}/nuttx.axf
	fi
else
	if [ -f "${NUTTX}/nuttx" ]; then
		echo "Both ${NUTTX}/nuttx ${NUTTX}/nuttx.axf exist.."
		echo "  Deleting ${NUTTX}/nuttx.axf"
		rm -f ${NUTTX}/nuttx.axf
		echo "Renaming ${NUTTX}/nuttx to ${NUTTX}/nuttx.axf"
		mv ${NUTTX}/nuttx ${NUTTX}/nuttx.axf
	fi
fi

# First of all boot the LPC-Link using the script:

#${DFUAPP} /f ${ROMPATH} /tl 250 dfuapp.log

# Then program the FLASH

#${FLASHUTIL} ${WIRE} -p${TARGET} -flash-load="${NUTTX}/nuttx.axf"
${FLASHUTIL} -p${TARGET} -flash-load="${NUTTX}/nuttx.axf"

