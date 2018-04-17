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
from cmsis_pack_manager import Cache
from jinja2 import Template
from fuzzywuzzy import process
import json

# TODO
# FIXED LENGTH - remove and these (shrink offset to 4 for bkpt only)
BLOB_HEADER = '0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,'
HEADER_SIZE = 0x20

template_dir = os.path.dirname(os.path.realpath(__file__))

tmpl_name_list = [
    ("c_blob.tmpl", "c_blob", "c"),
    ("py_blob.tmpl", "py_blob", "py"),
    ("c_blob_mbed.tmpl", "c_blob_mbed", "c")
]

name_map = {}
no_match = []

def main():
    parser = argparse.ArgumentParser(description="Blob generator")
    parser.add_argument("--rebuild_all", action="store_true",
                        help="Rebuild entire cache")
    parser.add_argument("--rebuild_descriptors", action="store_true",
                        help="Rebuild descriptors")
    parser.add_argument("--blob_start", type=lambda x: int(x, 0), default=None,
                        help="Starting address of the flash blob."
                        "Used only for DAPLink.")
    parser.add_argument("--daplink", default=None,
                        help="Root location of daplink")
    parser.add_argument("--all", action="store_true",
                        help="Build all flash algos for devcies")
    args = parser.parse_args()

    cache = Cache(True, True)
    if args.rebuild_all:
        cache.cache_everything()
        print("Cache rebuilt")
        exit(0)

    if args.rebuild_descriptors:
        cache.cache_descriptors()
        print("Descriptors rebuilt")
        exit(0)

    if args.daplink:
        target_to_dir = get_daplink_files(args.daplink)
        total_targets = len(target_to_dir.keys())
        count = 1
        try:
            board_to_pack = json.load(open('board_to_pack.json'))
        except:
            print "*"*10
            board_to_pack = {}
        for target, output_path in target_to_dir.items():
            print ("%d/%d"%(count,total_targets))
            count += 1
            output_files(cache, target, output_path, args.blob_start, args.all, board_to_pack)
        name_map_path = join(args.daplink, "daplink_to_cmsis.json")
        with open(name_map_path, 'w') as f:
            json.dump(name_map, f, indent=4)

def find_possible(match, choices):
    return process.extractOne(match, choices)

def find_match(cache, daplink_target_name, board_to_pack):
    try:
        device = board_to_pack[daplink_target_name]
        if device not in cache.index:
                return None, None
    except:
        fuzz1 = find_possible(daplink_target_name, cache.aliases.keys())
        fuzz2 = find_possible(daplink_target_name, cache.index.keys())
        if fuzz1[1] >= 90:
            device = cache.aliases[fuzz1[0]]
            if device not in cache.index and fuzz2[1] >= 90:
                device = fuzz2[0]
        elif fuzz2[1] >= 90:
            device = fuzz2[0]
        else:
            return None, None
    return cache.index[device], device

def write_target_c_file(dev, algo, output_path, target):
    tmpl = "c_target.tmpl"
    template_path = os.path.join(template_dir, tmpl)
    ram_start, ram_size = get_algo_ram(dev)
    rom_start, rom_size = get_algo_rom(dev)
    target_data_dict = {
        'name': target,
        "flash_start": rom_start,
        "flash_size": rom_size,
        "sector_size": algo.sector_sizes[0][1],
        "ram_start": ram_start,
        "ram_size": ram_size,
    }
    with open(template_path) as file_handle:
        template_text = file_handle.read()
    template = Template(template_text)
    target_text = template.render(target_data_dict)
    with open(output_path, "wb") as file_handle:
        file_handle.write(target_text)


def output_files(cache, target, output_dir, blob_start, all_algo, board_to_pack):
    dev, device = find_match(cache, target, board_to_pack)
    name_map[target] = device if dev else "No Match"
    if (dev is None):
        print "No match for %s"%(target)
        no_match.append(target)
        return
    algo_ram = (get_algo_ram(dev, (None, None))[0] if blob_start is None
                else blob_start)
    if algo_ram is None:
        print("No ram address for %s"%target)
        return

    print "%s/%s writing to %s"%(target,device,output_dir)
    SP = algo_ram + 2048
    data_dict = {
        'name': target,
        'prog_header': BLOB_HEADER,
        'header_size': HEADER_SIZE,
        'entry': algo_ram,
        'stack_pointer': SP,
    }

    binaries = cache.get_flash_algorthim_binary(device, all=True)
    algos = [PackFlashAlgo(binary.read()) for binary in binaries]
    filtered_algos = algos if all_algo else filter_algos(dev, algos)
    for idx, algo in enumerate(filtered_algos):
        index_str = ("_%i" % idx if all_algo or
                     len(filtered_algos) != 1 else "")

        tmpl = "c_blob.tmpl"
        template_path = os.path.join(template_dir, tmpl)
        output_path = os.path.join(output_dir, "flash_blob.c")
        algo.process_template(template_path, output_path, data_dict)

        output_path = os.path.join(output_dir, "target.c")
        write_target_c_file(dev, algo, output_path, target)

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


def get_daplink_files(daplink_root):
    daplink_targets = join(daplink_root, 'source', 'target')
    target_to_dir = {}
    print(os.getcwd())
    for root,dirs,files in os.walk(daplink_targets):
        if 'flash_blob.c' in files:
            target = os.path.basename(os.path.normpath(root))
            target_to_dir[target] = root
    return target_to_dir


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
    if "IROM1" in dev["memory"]:
        rom_rgn = dev["memory"]["IROM1"]
    elif "PROGRAM_FLASH" in dev["memory"]:
        rom_rgn = dev["memory"]["PROGRAM_FLASH"]
    else:
        return errval

    try:
        start = int(rom_rgn["start"], 0)
        size = int(rom_rgn["size"], 0)
    except ValueError:
        return errval
    return start, size

def merge_ram(rgn1, rgn2):
    ret_rgn = {}
    if(int(rgn1["start"],0) < int(rgn2["start"],0)):
        lower_rgn = rgn1
        upper_rgn = rgn2
    else:
        lower_rgn = rgn2
        upper_rgn = rgn1

    rgn1_start = int(lower_rgn["start"],0)
    rgn2_start = int(upper_rgn["start"],0)
    rgn1_size  = int(lower_rgn["size"],0)
    rgn2_size  = int(upper_rgn["size"],0)

    if((rgn1_start + rgn1_size) == rgn2_start):
        ret_rgn["start"] = hex(rgn1_start)
        ret_rgn["size"] = hex(rgn1_size + rgn2_size)
        return ret_rgn
    else:
        return rgn1


def get_algo_ram(dev, errval=(0xDEADBEEF, 0xDEADBEEF)):
    if "memory" not in dev:
        return errval
    if "IRAM1" not in dev["memory"]:
        return errval

    ram_rgn = dev["memory"]["IRAM1"]
    if "IRAM2" in dev["memory"]:
        ram_rgn2 = dev["memory"]["IRAM2"]
        ram_rgn = merge_ram(ram_rgn, ram_rgn2)
    try:
        start = int(ram_rgn["start"], 0)
        size = int(ram_rgn["size"], 0)
    except ValueError:
        return errval
    return start, size


def filter_algos(dev, algos):
    if "memory" not in dev:
        return algos
    if "IROM2" in dev["memory"]:
        return algos

    if "IROM1" in dev["memory"]:
        rom_rgn = dev["memory"]["IROM1"]
    elif "PROGRAM_FLASH" in dev["memory"]:
        rom_rgn = dev["memory"]["PROGRAM_FLASH"]
    else:
        return algos

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
