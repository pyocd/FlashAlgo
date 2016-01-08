'''
FlashAlgo
Copyright (c) 2011-2015 ARM Limited

Licensed under the Apache License, Version 2.0 (the 'License');
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an 'AS IS' BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This script takes the path to file(s) created by an image conversion tool
(fromelf or arm-elf-objcopy) as input. These files (bin and text) are used
to create flash programming blobs (instruction arrays) which are then
loaded into the target MCU RAM. Generates files compatible with C programs 
and python programs (DAPLink Interface Firmware and pyDAPFlash)
'''
from struct import unpack
from os.path import join
from jinja2 import Template, StrictUndefined
import sys, os, collections

# TODO
# FIXED LENGTH - remove and these (shrink offset to 4 for bkpt only)
# defines better documented
ALGO_OFFSET = 0x20
BLOB_START = 0x20000000
BLOB_HEADER = '0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,'
SP = BLOB_START + 2048
PROG_PAGE_SIZE = 512

class FlashInfo(object):
    def __init__(self, path):
        with open(path, 'rb') as f:
            # Read Device Information struct (defined in FlashOS.H, declared in FlashDev.c).
            self.version  = unpack('H', f.read(2))[0]
            self.devName  = f.read(128).split(b'\0',1)[0]
            self.devType  = unpack('H', f.read(2))[0]
            self.devAddr  = unpack('L', f.read(4))[0]
            self.szDev    = unpack('L', f.read(4))[0]
            self.szPage   = unpack('L', f.read(4))[0]
            skipped = f.read(4)
            self.valEmpty = unpack('B', f.read(1))[0]
            skipped = f.read(3)
            self.toProg   = unpack('L', f.read(4))[0]
            self.toErase  = unpack('L', f.read(4))[0]
            self.sectSize = []
            self.sectAddr = []
            while 1:
                addr = unpack('L', f.read(4))[0]
                size = unpack('L', f.read(4))[0]
                if addr == 0xffffffff:
                    break

                elif size == 0xffffffff:
                    break

                else:
                    self.sectSize.append(size)
                    self.sectAddr.append(addr)

    def printInfo(self):
        print 'Extracted device information:'
        print '----------------------------'
        print 'Version:        0x%04x' % (self.version)
        print 'Device Name:    %s' % (self.devName)
        print 'Device Type:    %u' % (self.devType)
        print 'Flash Start:    0x%08x' % (self.devAddr)
        print 'Flash Size:     0x%08x' % (self.szDev)
        print 'Prog Page Size: %u' % (self.szPage)
        print 'valEmpty:       0x%02x' % (self.valEmpty)
        print 'Timeout Prog:   %u' % (self.toProg)
        print 'Timeout Erase:  %u' % (self.toErase)
        for i in range(len(self.sectSize)):
            print 'Sectors[%d]: { 0x%08x, 0x%08x }' % (i, self.sectSize[i], self.sectAddr[i])


def generate_blob(template_path_file, ext, data):
    output = data['dir'] + '\\' + data['name'] + '_prog_blob.' + ext
    
    ''' Fills data to the project template, using jinja2. '''
    template_path = template_path_file
    template_text = open(template_path).read()
    template = Template(template_text)
    target_text = template.render(data)

    open(output, 'w').write(target_text)
    return


def decode_axf(string):
    ELF_PATH = string
    DEV_DSCR_PATH = join(ELF_PATH, 'DevDscr')
    PRG_CODE_PATH = join(ELF_PATH, 'PrgCode')
    ALGO_SYM_PATH = join(ELF_PATH, 'symbols')
    
    # print some info about the build
    flash_info = FlashInfo(DEV_DSCR_PATH)
    flash_info.printInfo()
    
    # prepare data to write to the template
    dic = {}
    dic['name'] = ELF_PATH.split('\\')[-1]
    dic['dir'] = ELF_PATH
    dic['prog_header'] = BLOB_HEADER
    dic['header_size'] = '0x%08x' % ALGO_OFFSET
    dic['entry'] = '0x%08x' % BLOB_START
    dic['prog_page_size'] = '0x%08x' % PROG_PAGE_SIZE
    dic['stack_pointer'] = '0x%08x' % SP
    dic['mem'] = ''
    dic['func'] = {}
    dic['static_base'] = ''

    with open(PRG_CODE_PATH, 'rb') as f1:
        nb_bytes = ALGO_OFFSET
        bytes_read = f1.read(1024)
        while bytes_read:
            bytes_read = unpack(str(len(bytes_read)/4) + 'I', bytes_read)
            for i in range(len(bytes_read)):
                dic['mem'] += '0x%08x' % bytes_read[i] + ', '
                nb_bytes += 4
                if (nb_bytes % 0x20) == 0:
                    dic['mem'] += '\n    '

            bytes_read = f1.read(1024)
                
        # Address of the functions within the flash algorithm
        with open(ALGO_SYM_PATH, 'rb') as f2:
            for line in list(f2):
                t = line.strip().split()
                if len(t) < 5: 
                    continue
                
                name, loc, sec = t[1], t[2], t[4]
                if name in ['Init', 'UnInit', 'EraseChip', 'EraseSector', 'ProgramPage', 'Verify']:
                    addr = BLOB_START + ALGO_OFFSET + int(loc, 16)
                    dic['func'].update({'%s' % name : '0x%08X' % addr})

                if name == '$d.realdata':
                    if sec == '2':
                        dic['static_base'] = '0x%08x' % int(loc, 16)

    # order the flash programming functions - known order 
    #dic['func'] = collections.OrderedDict(sorted(dic['func'].items()))
    return dic


if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print 'usage: >python gen_algo.py <abs_path_bin_algo_info>'
        sys.exit()
    
    data = decode_axf(sys.argv[1])
    generate_blob(os.path.dirname(os.path.realpath(__file__)) + '\\' + 'c_blob.tmpl', 'h', data)
    generate_blob(os.path.dirname(os.path.realpath(__file__)) + '\\' + 'py_blob.tmpl', 'py', data)

