'''
FlashAlgo
Copyright (c) 2011-2017 ARM Limited

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
import os
import argparse
from flash_algo import PackFlashAlgo

# TODO
# FIXED LENGTH - remove and these (shrink offset to 4 for bkpt only)
BLOB_HEADER = '0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,'
HEADER_SIZE = 0x20


def main():
    parser = argparse.ArgumentParser(description="Blob generator")
    parser.add_argument("elf_path", help="Elf, axf, or flm to extract "
                        "flash algo from")
    parser.add_argument("--blob_start", default=0x20000000, help="Starting "
                        "address of the flash blob. Used only for DAPLink.")
    args = parser.parse_args()

    with open(args.elf_path, "rb") as file_handle:
        algo = PackFlashAlgo(file_handle.read())

    print(algo.flash_info)

    template_dir = os.path.dirname(os.path.realpath(__file__))
    output_dir = os.path.dirname(args.elf_path)
    SP = args.blob_start + 2048
    data_dict = {
        'name': os.path.splitext(os.path.split(args.elf_path)[-1])[0],
        'prog_header': BLOB_HEADER,
        'header_size': HEADER_SIZE,
        'entry': args.blob_start,
        'stack_pointer': SP,
    }

    tmpl_name_list = [
        ("c_blob.tmpl", "c_blob.c"),
        ("py_blob.tmpl", "py_blob.py"),
        ("c_blob_mbed.tmpl", "c_blob_mbed.c")
    ]

    for tmpl, name in tmpl_name_list:
        template_path = os.path.join(template_dir, tmpl)
        output_path = os.path.join(output_dir, name)
        algo.process_template(template_path, output_path, data_dict)


if __name__ == '__main__':
    main()
