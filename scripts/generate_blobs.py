#!/usr/bin/env python
# Copyright (c) 2011-2017 Arm Limited
# Copyright (c) 2021 Chris Reed
# 
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# 
# This script takes the path to file(s) created by an image conversion tool
# (fromelf or arm-elf-objcopy) as input. These files (bin and text) are used
# to create flash programming blobs (instruction arrays) which are then
# loaded into the target MCU RAM. Generates files compatible with C programs
# and python programs (DAPLink Interface Firmware and pyDAPFlash)

import os
import argparse
from flash_algo import PackFlashAlgo

# Header with a BKPT instruction.
BLOB_HEADER = '0xE00ABE00, '
HEADER_SIZE = 0x4

STACK_SIZE = 0x200

TEMPLATES = [
    ("c_blob.tmpl", "c_blob.c"),
    ("py_blob_orig.tmpl", "py_blob_orig.py"),
    ("py_blob.tmpl", "py_blob.py"),
    ("c_blob_mbed.tmpl", "c_blob_mbed.c")
]

def str_to_num(val):
    return int(val,0)  #convert string to number and automatically handle hex conversion

def main():
    parser = argparse.ArgumentParser(description="Blob generator")
    parser.add_argument("elf_path", help="Elf, axf, or flm to extract "
                        "flash algo from")
    parser.add_argument("--blob_start", default=0x20000000, type=str_to_num, help="Starting "
                        "address of the flash blob. Used only for DAPLink.")
    args = parser.parse_args()

    with open(args.elf_path, "rb") as file_handle:
        algo = PackFlashAlgo(file_handle.read())

    print(algo.flash_info)

    template_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'templates')
    output_dir = os.path.dirname(args.elf_path)

    # Allocate stack after algo and its rw data, rounded up.
    SP = args.blob_start + HEADER_SIZE + algo.rw_start + algo.rw_size + STACK_SIZE
    SP = (SP + 0x100 - 1) // 0x100 * 0x100

    data_dict = {
        'name': os.path.splitext(os.path.split(args.elf_path)[-1])[0],
        'prog_header': BLOB_HEADER,
        'header_size': HEADER_SIZE,
        'entry': args.blob_start,
        'stack_pointer': SP,
    }

    for tmpl, name in TEMPLATES:
        template_path = os.path.join(template_dir, tmpl)
        output_path = os.path.join(output_dir, name)
        algo.process_template(template_path, output_path, data_dict)


if __name__ == '__main__':
    main()
