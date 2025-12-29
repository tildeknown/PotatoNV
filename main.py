#!/usr/bin/env python3

# Copyright 2019 Penn Mackintosh
# Copyright 2020 Andrey Smirnoff
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import argparse
import hashlib
import xml.etree.ElementTree as ET
from os import path
import traceback
from fastbootpy import FastbootDevice, FastbootManager
import binascii
import serial
import serial.tools.list_ports
import sys
import os
import time
from time import sleep

def calc_crc(data, crc=0):
    for char in data:
        crc = ((crc << 8) | char) ^ binascii.crc_hqx(bytes([(crc >> 8) & 0xFF]), 0)
    for i in range(0,2):
        crc = ((crc << 8) | 0) ^ binascii.crc_hqx(bytes([(crc >> 8) & 0xFF]), 0)
    return crc & 0xFFFF


BOOT_HEAD_LEN = 0x4F00
MAX_DATA_LEN = 0x400
IDT_BAUDRATE = 115200
IDT_VID=0x12D1
IDT_PID=0x3609


class ImageFlasher:
    def __init__(self):
        self.serial = None
        self.headframe = bytes([0xFE, 0x00, 0xFF, 0x01])
        self.dataframe = bytes([0xDA])
        self.tailframe = bytes([0xED])
        self.ack = bytes([0xAA])

    def send_frame(self, data):
        crc = calc_crc(data)
        data += crc.to_bytes(2, byteorder="big", signed=False)
        try:
            self.serial.reset_output_buffer()
            self.serial.reset_input_buffer()
            self.serial.write(data)
            ack = self.serial.read(1)
            if ack and ack != self.ack:
                print(f"Invalid ACK from device! Read: {hex(ack)}, excepted: {hex(self.ack[0])}")
        except Exception as e:
            print(str(e))

    def send_head_frame(self, length, address):
        self.serial.timeout = 0.09
        data = self.headframe
        data += length.to_bytes(4, byteorder="big", signed=False)
        data += address.to_bytes(4, byteorder="big", signed=False)
        self.send_frame(data)

    def send_data_frame(self, n, data):
        self.serial.timeout = 0.45
        head = bytearray(self.dataframe)
        head.append(n & 0xFF)
        head.append((~ n) & 0xFF)
        self.send_frame(bytes(head) + data)

    def send_tail_frame(self, n):
        if self.serial:
            self.serial.timeout = 0.01
        data = bytearray(self.tailframe)
        data.append(n & 0xFF)
        data.append((~ n) & 0xFF)
        self.send_frame(bytes(data))

    def send_data(self, data, length, address):
        if isinstance(data, bytes):
            length = len(data)
        n_frames = length // MAX_DATA_LEN + (1 if length % MAX_DATA_LEN > 0 else 0)
        self.send_head_frame(length, address)
        n = 0
        while length > MAX_DATA_LEN:
            if isinstance(data, bytes):
                f = data[n * MAX_DATA_LEN:(n + 1) * MAX_DATA_LEN]
            else:
                f = data.read(MAX_DATA_LEN)
            self.send_data_frame(n + 1, f)
            n += 1
            length -= MAX_DATA_LEN
            print(round(n/n_frames*100), "%", end="\r")
        if length:
            if isinstance(data, bytes):
                f = data[n * MAX_DATA_LEN:]
            else:
                f = data.read()
            self.send_data_frame(n + 1, f)
            n += 1
        print("100 %")
        self.send_tail_frame(n + 1)
        time.sleep(0.5)

    def download_from_disk(self, fil, address):
        if fil == "-":
            f = sys.stdin
        else:
            f = open(fil, "rb")
        self.send_data(f, os.stat(fil).st_size, address)

    def connect_serial(self, device=None):
        print("Waiting for device in IDT mode")
        while not device:
            ports = serial.tools.list_ports.comports(include_links=False)
            for port in ports:
                if port.vid == IDT_VID and port.pid == IDT_PID:
                    print(f"Autoselecting {port.hwid} aka {port.description} at {port.device}")
                    if device:
                        print("Multiple devices detected in IDT mode")
                    else:
                        device = port.device
            
            if not device:
                sleep(1)

        if not device:
            print("Need a device in IDT mode plugged in to this computer")
        self.serial = serial.Serial(dsrdtr=True, rtscts=True, port=device.replace("COM", r"\\.\COM"), baudrate=IDT_BAUDRATE, timeout=1)

    def close(self):
        try:
            self.serial.close()
        except:
            pass

def handle_exception(e: Exception, message: str):
    print(message)
    traceback.print_exc()
    exit(1)

class Fastboot:
    def connect(self):
        print("Waiting for fastboot device")
        while True:
            devices = FastbootManager.devices()
            if len(devices) == 1:
                self.fb_dev = FastbootDevice.connect(devices[0])
                print(f"Connected to device {devices[0]}")
                break
            elif len(devices) > 1:
                print("More than one fastboot device is connected!")

    def write_nvme(self, prop: str, data: bytes):
        cmd = f"getvar:nve:{prop}@".encode('UTF-8')
        cmd += data
        print(f"Sending command: {cmd}")
        print(f"Writing {prop}")
        result = self.fb_dev.send(cmd)
        if "set nv ok" not in result:
            print(f"Failed to write {prop}: {result}")

    def reboot(self):
        result = self.fb_dev.reboot()
        print(f"Reboot result: {result}")
    
    def reboot_bootloader(self):
        result = self.fb_dev.reboot_bootloader()
        print(f"Reboot bootloader result: {result}")

def setup():
    parser = argparse.ArgumentParser(epilog="""Copyright 2020 mashed-potatoes
Copyright 2019 Penn Mackintosh
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.""")
    parser.add_argument("--skip-bootloader", "-s", action="store_true", help="Skip bootloader flashing")
    parser.add_argument("--skip-write-key", "-S", action="store_true", help="Skip NVME writing")
    parser.add_argument("--key", "-k", help="What key should be set?")
    parser.add_argument("--fblock", "-f", help="Set FBLOCK")
    parser.add_argument("--bootloader", "-b", help="Specify bootloader name")
    args = parser.parse_args()

    if not args.bootloader and not args.skip_bootloader:
        print("Use -b <bootloader> to choose bootloader.")
        print("Bootloaders:", *[i for i in os.listdir("./bootloaders") if os.path.exists("./bootloaders/"+i+"/manifest.xml")], sep="\n")
        exit(1)

    args.manifest = f"./bootloaders/{args.bootloader}/manifest.xml"

    if not path.isfile(args.manifest):
        print("Bootloader is invalid or not found!")

    if not args.key and not args.skip_write_key:
        args.key = "0123456789ABCDEF"
    
    return args

def flash_images(data: dict):
    flasher = ImageFlasher()
    flasher.connect_serial()
    for image in data["images"]:
        print("Flashing {}".format(image['role']))
        flasher.download_from_disk("./bootloaders/{}/{}"
                                   .format(data['name'], image['path']), int(image['address'], 16))
    print("Bootloader uploaded.")

def write_nvme(key: str):
    m = hashlib.sha256()
    m.update(key.encode())
    fb = Fastboot()
    fb.connect()
    fb.write_nvme("USRKEY", m.digest())
    fb.write_nvme("WVLOCK", key.encode())
    print("Bootloader code updated")

def main():
    args = setup()
    if not args.skip_bootloader:
        xmltree = ET.parse(args.manifest)
        bootloader = xmltree.getroot()

        data = bootloader.attrib
        data["images"] = list(map(lambda img: img.attrib, bootloader.findall('image')))
        data["name"] = args.bootloader

        flash_images(data)

    if not args.skip_write_key:
        write_nvme(args.key)

if __name__ == "__main__":
    main()

