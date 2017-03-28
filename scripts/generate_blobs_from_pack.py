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
from os.path import join
from flash_algo import PackFlashAlgo
from ArmPackManager import Cache
from jinja2 import Template

# TODO
# FIXED LENGTH - remove and these (shrink offset to 4 for bkpt only)
BLOB_HEADER = '0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,'
HEADER_SIZE = 0x20

OUTPUT_DIR = "output"


def main():
    parser = argparse.ArgumentParser(description="Blob generator")
    parser.add_argument("device", help="Device name to generate algo for")
    parser.add_argument("--rebuild_all", action="store_true",
                        help="Rebuild entire cache")
    parser.add_argument("--rebuild_descriptors", action="store_true",
                        help="Rebuild descriptors")
    parser.add_argument("--blob_start", type=lambda x: int(x, 0), default=None,
                        help="Starting address of the flash blob."
                        "Used only for DAPLink.")
    parser.add_argument("--all", action="store_true",
                        help="Build all flash algos for devcies")
    args = parser.parse_args()
    device = args.device

    cache = Cache(True, True)
    if args.rebuild_all:
        cache.cache_everything()
        print("Cache rebuilt")
        exit(0)

    if args.rebuild_descriptors:
        cache.cache_descriptors()
        print("Descriptors rebuilt")
        exit(0)

    dev = cache.index[device]
    algo_ram = (get_algo_ram(dev, (None, None))[0] if args.blob_start is None
                else args.blob_start)
    if algo_ram is None:
        print("Ram address must be specified with --blob_start")
        exit(-1)

    try:
        os.mkdir(OUTPUT_DIR)
    except OSError:
        # Directory already exists
        pass

    basename = device.replace("/", "_")
    template_dir = os.path.dirname(os.path.realpath(__file__))
    output_dir = OUTPUT_DIR
    SP = algo_ram + 2048
    data_dict = {
        'name': basename,
        'prog_header': BLOB_HEADER,
        'header_size': HEADER_SIZE,
        'entry': algo_ram,
        'stack_pointer': SP,
    }

    tmpl_name_list = [
        ("c_blob.tmpl", "c_blob", "c"),
        ("py_blob.tmpl", "py_blob", "py"),
        ("c_blob_mbed.tmpl", "c_blob_mbed", "c")
    ]

    binaries = cache.get_flash_algorthim_binary(device, all=True)
    algos = [PackFlashAlgo(binary.read()) for binary in binaries]
    filtered_algos = algos if args.all else filter_algos(dev, algos)
    for idx, algo in enumerate(filtered_algos):
        index_str = ("_%i" % idx if args.all or
                     len(filtered_algos) != 1 else "")
        for tmpl, filebase, ext in tmpl_name_list:
            filename = basename + index_str + "_" + filebase + "." + ext
            template_path = os.path.join(template_dir, tmpl)
            output_path = os.path.join(output_dir, filename)
            algo.process_template(template_path, output_path, data_dict)

        filename = basename + index_str + "_target.c"
        output_path = os.path.join(output_dir, filename)
        TARGET_TEMPLATE = "c_target.tmpl"
        ram_start, ram_size = get_algo_ram(dev)
        rom_start, rom_size = get_algo_rom(dev)
        target_data_dict = {
            "flash_start": rom_start,
            "flash_size": rom_size,
            "sector_size": algo.sector_sizes[0][1],
            "ram_start": ram_start,
            "ram_size": ram_size,
        }
        with open(TARGET_TEMPLATE) as file_handle:
            template_text = file_handle.read()
        template = Template(template_text)
        target_text = template.render(target_data_dict)
        with open(output_path, "wb") as file_handle:
            file_handle.write(target_text)

        warnings = get_algo_warnings(algo)
        if warnings is not None:
            print("Device %s algo %s warnings:" % (device, idx))
            for warning in warnings:
                print("  -%s" % warning)

    warnings = get_dev_warnings(dev)
    if warnings is None:
        print("No Device warnings")
    else:
        print("Device %s warnings" % device)
        for warning in warnings:
            print("  -%s" % warning)


def get_algo_warnings(algo):
    warnings = []
    if len(algo.sector_sizes) != 1:
        warnings.append("Device contains varaiable sized sectors (%s)" %
                        len(algo.sector_sizes))
    if algo.symbols['EraseChip'] == 0xffffffff:
        warnings.append("Device does not support chip erase")
    return None if len(warnings) == 0 else warnings


def get_dev_warnings(dev):
    warnings = []
    if "memory" not in dev:
        return ["No memory region defined for device"]

    memory = dev["memory"]
    for region in ("IROM1", "IRAM1"):
        if region in dev["memory"]:
            try:
                int(memory[region]["start"], 0)
                int(memory[region]["size"], 0)
            except ValueError:
                warnings.append("Region %s attributes invalid" % region)
            except KeyError:
                warnings.append("Region %s attributes missing" % region)
        else:
            warnings.append("Device does not contain required region %s" %
                            region)

    for name, region in dev["memory"].items():
        if name not in ("IROM1", "IRAM1"):
            warnings.append("Unknown region %s" % name)
    return None if len(warnings) == 0 else warnings


def get_algo_rom(dev, errval=(0xDEADBEEF, 0xDEADBEEF)):
    if "memory" not in dev:
        return errval
    if "IROM1" not in dev["memory"]:
        return errval

    rom_rgn = dev["memory"]["IROM1"]
    try:
        start = int(rom_rgn["start"], 0)
        size = int(rom_rgn["size"], 0)
    except ValueError:
        return errval
    return start, size


def get_algo_ram(dev, errval=(0xDEADBEEF, 0xDEADBEEF)):
    if "memory" not in dev:
        return errval
    if "IRAM1" not in dev["memory"]:
        return errval

    ram_rgn = dev["memory"]["IRAM1"]
    try:
        start = int(ram_rgn["start"], 0)
        size = int(ram_rgn["size"], 0)
    except ValueError:
        return errval
    return start, size


def filter_algos(dev, algos):
    if "memory" not in dev:
        return algos
    if "IROM1" not in dev["memory"]:
        return algos
    if "IROM2" in dev["memory"]:
        return algos

    rom_rgn = dev["memory"]["IROM1"]
    try:
        start = int(rom_rgn["start"], 0)
        size = int(rom_rgn["size"], 0)
    except ValueError:
        return algos

    matching_algos = [algo for algo in algos if
                      algo.flash_start == start and algo.flash_size == size]
    return matching_algos if len(matching_algos) == 1 else algos


if __name__ == '__main__':
    main()
