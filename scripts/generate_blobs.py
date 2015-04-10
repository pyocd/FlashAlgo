"""
FlashAlgo
Copyright (c) 2011-2015 ARM Limited

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This script takes the path to file(s) created by an image conversion tool
(fromelf or arm-elf-objcopy) as input. These files (bin and text) are used
to create flash programming blobs (instruction arrays) which are then
loaded into the target MCU RAM. Generates files compatible with C programs 
and python programs (DAPLink Interface Firmware and pyDAPFlash)
"""
from struct import unpack
from os.path import join
import sys

# TODO
# - create template for c and py files
# - use template rather than hardcoded
# FIXED LENGTH - remove and these (shrink offset to 4 for bkpt only)
ALGO_OFFSET = 0x20
ALGO_START = 0x20000000

class FlashInfo(object):
    def __init__(self, path):
        with open(path, "rb") as f:
            # Read Device Information struct (defined in FlashOS.H, declared in FlashDev.c).
            self.version  = unpack("H", f.read(2))[0]
            self.devName  = f.read(128).split(b'\0',1)[0]
            self.devType  = unpack("H", f.read(2))[0]
            self.devAddr  = unpack("L", f.read(4))[0]
            self.szDev    = unpack("L", f.read(4))[0]
            self.szPage   = unpack("L", f.read(4))[0]
            skipped = f.read(4)
            self.valEmpty = unpack("B", f.read(1))[0]
            skipped = f.read(3)
            self.toProg   = unpack("L", f.read(4))[0]
            self.toErase  = unpack("L", f.read(4))[0]
            self.sectSize = []
            self.sectAddr = []
            while 1:
                addr = unpack("L", f.read(4))[0]
                size = unpack("L", f.read(4))[0]
                if addr == 0xffffffff:
                    break
                elif size == 0xffffffff:
                    break
                else:
                    self.sectSize.append(size)
                    self.sectAddr.append(addr)

    def printInfo(self):
        print "Extracted device information:"
        print "----------------------------"
        print "Version:        0x%04x" % (self.version)
        print "Device Name:    %s" % (self.devName)
        print "Device Type:    %u" % (self.devType)
        print "Flash Start:    0x%08x" % (self.devAddr)
        print "Flash Size:     0x%08x" % (self.szDev)
        print "Prog Page Size: %u" % (self.szPage)
        print "valEmpty:       0x%02x" % (self.valEmpty)
        print "Timeout Prog:   %u" % (self.toProg)
        print "Timeout Erase:  %u" % (self.toErase)
        for i in range(len(self.sectSize)):
            print "Sectors[%d]: { 0x%08x, 0x%08x }" % (i, self.sectSize[i], self.sectAddr[i])


def generate_c_blob(string):
    ALGO_ELF_PATH_NAME = string
    
    ALGO_PATH = string
    DEV_DSCR_PATH = join(ALGO_PATH, "DevDscr")
    PRG_CODE_PATH = join(ALGO_PATH, "PrgCode")
    ALGO_SYM_PATH = join(ALGO_PATH, "symbols")
    # need some work here to name and locate to a collective folder
    CBLOB_PATH = join(ALGO_PATH, "flash_algo.txt")

    flash_info = FlashInfo(DEV_DSCR_PATH)
    flash_info.printInfo()

    with open(PRG_CODE_PATH, "rb") as f1, open(CBLOB_PATH, mode="w+") as res:
        # Flash Algorithm - these instructions are the ALGO_OFFSET
        res.write("""
const uint32_t flash_algorithm_blob[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    """);

        nb_bytes = ALGO_OFFSET
        prg_data = ''

        bytes_read = f1.read(1024)
        while bytes_read:
            bytes_read = unpack(str(len(bytes_read)/4) + 'I', bytes_read)
            for i in range(len(bytes_read)):
                res.write(("0x%08x" % bytes_read[i]) + ", ")
                nb_bytes += 4
                if (nb_bytes % 0x20) == 0:
                    res.write("\n    ") # % nb_bytes)
            bytes_read = f1.read(1024)
        
        res.write("\n};\n")
                
        # Address of the functions within the flash algorithm
        with open(ALGO_SYM_PATH, "rb") as f2:
            res.write("""
    static const TARGET_FLASH flash_algorithm_struct = {
    """)
            for line in list(f2):
                t = line.strip().split()
                if len(t) < 5: continue
                name, loc, sec = t[1], t[2], t[4]
                
                if name in ['Init', 'UnInit', 'EraseChip', 'EraseSector', 'ProgramPage']:
                    addr = ALGO_START + ALGO_OFFSET + int(loc, 16)
                    res.write("    0x%08X, // %s\n" % (addr,  name))

                if name == '$d.realdata':
                    if sec == '2':
                        prg_data = int(loc, 16)

            res.write("    // breakpoint = RAM start + 1\n")
            res.write("    // RSB : base address is address of Execution Region PrgData in map file\n")
            res.write("    //       to access global/static data\n")
            res.write("    // RSP : Initial stack pointer\n")
            res.write("    {\n")
            res.write("        0x%08X, // breakpoint instruction address\n" % (ALGO_START+1))
            res.write("        0x%08X + 0x%X + 0x%X,  // static base register value (image start + header + static base offset)\n" % (ALGO_START, ALGO_OFFSET, prg_data))
            res.write("        0x%08X // initial stack pointer\n" % (ALGO_START+2048))
            res.write("    },\n\n")
            res.write("    0x%08X, // flash_buffer, any valid RAM location with > 512 bytes of working room and proper alignment\n" % (ALGO_START+2048+256))
            res.write("    0x%08X, // algo_start, start of RAM\n" % ALGO_START)
            res.write("    sizeof(flash_algorithm_blob), // algo_size, size of array above\n")
            res.write("    flash_algorithm, // image, flash algo instruction array\n")
            res.write("    512              // ram_to_flash_bytes_to_be_written\n")
            res.write("};\n\n")

    return


def generate_py_blob(string):
    return

if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print "usage: >python gen_algo.py <abs_path_bin_algo_info>"
        sys.exit()
    
    generate_c_blob(sys.argv[1])
    generate_py_blob(sys.argv[1])

