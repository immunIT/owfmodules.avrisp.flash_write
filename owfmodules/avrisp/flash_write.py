# -*- coding: utf-8 -*-

# Octowire Framework
# Copyright (c) ImmunIT - Jordan Ovrè / Paul Duncan
# License: Apache 2.0
# Paul Duncan / Eresse <pduncan@immunit.ch>
# Jordan Ovrè / Ghecko <jovre@immunit.ch>

import struct
import time
import hexformat

from io import BytesIO
from tqdm import tqdm

from octowire_framework.module.AModule import AModule
from octowire.gpio import GPIO
from octowire.spi import SPI
from owfmodules.avrisp.device_id import DeviceID


class FlashWrite(AModule):
    def __init__(self, owf_config):
        super(FlashWrite, self).__init__(owf_config)
        self.meta.update({
            'name': 'AVR write flash memory',
            'version': '1.0.0',
            'description': 'Module to write the flash memory of an AVR device using the ISP protocol.',
            'author': 'Jordan Ovrè / Ghecko <jovre@immunit.ch>, Paul Duncan / Eresse <pduncan@immunit.ch>'
        })
        self.options = {
            "spi_bus": {"Value": "", "Required": True, "Type": "int",
                        "Description": "The octowire SPI bus (0=SPI0 or 1=SPI1)", "Default": 0},
            "reset_line": {"Value": "", "Required": True, "Type": "int",
                           "Description": "The octowire GPIO used as the Reset line", "Default": 0},
            "spi_baudrate": {"Value": "", "Required": True, "Type": "int",
                             "Description": "set SPI baudrate (1000000 = 1MHz) maximum = 50MHz", "Default": 1000000},
            "firmware": {"Value": "", "Required": True, "Type": "file_r",
                         "Description": "The firmware to write into the flash memory.\n"
                                        "Allowed file type: IntelHex or Raw binary.", "Default": ""},
            "verify": {"Value": "", "Required": False, "Type": "bool",
                       "Description": "Verify the firmware after the write process", "Default": False},
        }
        self.dependencies.append("owfmodules.avrisp.device_id>=1.0.0")
        self.extended_addr = None
        self.busy_wait = None

    def get_device_id(self, spi_bus, reset_line, spi_baudrate):
        device_id_module = DeviceID(owf_config=self.config)
        # Set DeviceID module options
        device_id_module.options["spi_bus"]["Value"] = spi_bus
        device_id_module.options["reset_line"]["Value"] = reset_line
        device_id_module.options["spi_baudrate"]["Value"] = spi_baudrate
        device_id_module.owf_serial = self.owf_serial
        device_id = device_id_module.run(return_value=True)
        return device_id

    def erase(self, spi_interface, reset, device):
        erase_cmd = b'\xac\x80\x00\x00'
        enable_mem_access_cmd = b'\xac\x53\x00\x00'

        # Drive reset low
        reset.status = 0

        self.logger.handle("Enable Memory Access...", self.logger.INFO)
        # Drive reset low
        reset.status = 0
        # Enable Memory Access
        spi_interface.transmit(enable_mem_access_cmd)
        time.sleep(0.5)

        # Send erase command and wait N ms
        self.logger.handle("Erasing the flash memory...", self.logger.INFO)
        spi_interface.transmit(erase_cmd)
        time.sleep(int(device["erase_delay"]) // 1000)

        # Drive reset high
        reset.status = 1
        self.logger.handle("Flash memory successfully erased.", self.logger.SUCCESS)

    def verify(self, spi_interface, flash_size, firmware):
        low_byte_read = b'\x20'
        high_byte_read = b'\x28'
        load_extended_addr = b'\x4d\x00'
        dump = BytesIO()
        extended_addr = None

        # Read flash loop
        for read_addr in tqdm(range(0, flash_size // 2), desc="Read", unit_divisor=1024, ascii=" #", unit_scale=True,
                              bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} Words "
                                         "[elapsed: {elapsed} left: {remaining}]"):
            # Load extended address
            if read_addr >> 16 != extended_addr:
                extended_addr = read_addr >> 16
                spi_interface.transmit(load_extended_addr + bytes([extended_addr]) + b'\x00')
            # Read low byte
            spi_interface.transmit(low_byte_read + struct.pack(">H", read_addr & 0xFFFF))
            dump.write(spi_interface.receive(1))
            # Read high byte
            spi_interface.transmit(high_byte_read + struct.pack(">H", read_addr & 0xFFFF))
            dump.write(spi_interface.receive(1))

        self.logger.handle("Verifying...", self.logger.INFO)
        for index, byte in enumerate(firmware):
            if byte != dump.getvalue()[index]:
                self.logger.handle("verification error, first mismatch at byte 0x{:04x}"
                                   "\n\t\t0x{:04x} != 0x{:04x}".format(index, byte, dump.getvalue()[index]),
                                   self.logger.ERROR)
                break
        else:
            self.logger.handle("{} bytes of flash successfully verified".format(len(firmware)), self.logger.SUCCESS)
        dump.close()

    def loading_firmware(self, firmware_file):
        try:
            with open(firmware_file, 'r') as file:
                ihex_firmware = hexformat.intelhex.IntelHex.fromihexfh(file)
                self.logger.handle("IntelHex format detected..", self.logger.INFO)
                firmware = ihex_firmware.get(address=0x00, size=ihex_firmware.usedsize())
        except (UnicodeDecodeError, hexformat.base.DecodeError, ValueError):
            self.logger.handle("Raw binary format detected..", self.logger.INFO)
            with open(firmware_file, 'rb') as file:
                firmware = bytearray(file.read())
        return firmware

    def _wait_poll_rdybsy(self, spi_interface, page_buffer, page_addr):
        while spi_interface.transmit_receive(b'\xF0\x00\x00\x00')[-1] & 0x01:
            pass

    def _wait_poll_flash(self, spi_interface, page_buffer, page_addr):
        low_byte_read = b'\x20'
        high_byte_read = b'\x28'
        load_extended_addr = b'\x4d\x00'

        buff_index = next((i for i, j in enumerate(page_buffer) if j != 0xFF), None)
        if buff_index is not None:
            # Load extended address
            spi_interface.transmit(load_extended_addr + bytes([page_addr >> 16]) + b'\x00')
            # Read the low byte
            if buff_index % 2 == 0:
                byte_addr = page_addr + (buff_index // 2)
                while True:
                    spi_interface.transmit(low_byte_read + struct.pack(">H", byte_addr & 0xFFFF))
                    if spi_interface.receive(1)[0] == page_buffer[buff_index]:
                        break
            # Read the high byte
            else:
                byte_addr = page_addr + (buff_index // 2) - 1
                while True:
                    spi_interface.transmit(high_byte_read + struct.pack(">H", byte_addr & 0xFFFF))
                    if spi_interface.receive(1)[0] == page_buffer[buff_index]:
                        break
            # Reload the base extended_addr
            spi_interface.transmit(load_extended_addr + bytes([self.extended_addr]) + b'\x00')

    def program_page(self, spi_interface, page_buffer, page_addr):
        low_byte_write = b'\x40\x00'
        high_byte_write = b'\x48\x00'
        write_program_memory_page = b'\x4c'
        load_extended_addr = b'\x4d\x00'
        # Word address
        page_addr = page_addr // 2

        # Load program memory page; Page indexed by words
        for i in range(len(page_buffer) // 2):
            low_byte, high_byte = page_buffer[2 * i:(2 * i) + 2]
            # Load low byte
            spi_interface.transmit(low_byte_write + bytes([i]) + bytes([low_byte]))
            # Load high byte
            spi_interface.transmit(high_byte_write + bytes([i]) + bytes([high_byte]))

        # Load extended address
        if page_addr >> 16 != self.extended_addr:
            self.extended_addr = page_addr >> 16
            spi_interface.transmit(load_extended_addr + bytes([self.extended_addr]) + b'\x00')

        # Write Program Memory Page
        spi_interface.transmit(write_program_memory_page + struct.pack(">H", page_addr & 0xFFFF) + b'\x00')

        # Wait the MCU finish writing the page buffer
        self.busy_wait(spi_interface, page_buffer, page_addr)

    def write(self, spi_interface, reset, device, firmware):
        enable_mem_access_cmd = b'\xac\x53\x00\x00'
        gesize = int(device["flash_pagesize"], 16)
        verify = self.options["verify"]["Value"]

        page_buffer = bytearray(flash_pagesize)

        # Drive reset low
        reset.status = 0
        # Enable Memory Access
        self.logger.handle("Enable Memory Access...", self.logger.INFO)
        spi_interface.transmit(enable_mem_access_cmd)
        time.sleep(0.5)

        for page_addr in tqdm(range(0, len(firmware), flash_pagesize), desc="Program", ascii=" #",
                              bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} pages "
                                         "[elapsed: {elapsed} left: {remaining}]"):
            # Init empty page in case len(page_buffer) < flash_pagesize
            for index in range(flash_pagesize):
                page_buffer[index] = 0xFF
            # Fulfill the buffer
            for index, byte in enumerate(firmware[page_addr:page_addr + flash_pagesize]):
                page_buffer[index] = byte
            # If page is empty, skip the current page (erased chip byte is already equal to 0xFF)
            if all([v == 0xFF for v in page_buffer]):
                continue
            self.program_page(spi_interface, page_buffer, page_addr)

        self.logger.handle("Successfully write {} byte(s) to flash memory.".format(len(firmware)), self.logger.SUCCESS)

        if verify:
            self.logger.handle("Start verifying flash memory against {}".format(self.options["firmware"]["Value"]))
            self.verify(spi_interface, int(device["flash_size"], 16), firmware)

        # Drive reset high
        reset.status = 1

    def process(self):
        spi_bus = self.options["spi_bus"]["Value"]
        reset_line = self.options["reset_line"]["Value"]
        spi_baudrate = self.options["spi_baudrate"]["Value"]

        device = self.get_device_id(spi_bus, reset_line, spi_baudrate)
        if device is None:
            return

        spi_interface = SPI(serial_instance=self.owf_serial, bus_id=spi_bus)
        reset = GPIO(serial_instance=self.owf_serial, gpio_pin=reset_line)

        # Configure SPI with default phase and polarity
        spi_interface.configure(baudrate=spi_baudrate)
        # Configure GPIO as output
        reset.direction = GPIO.OUTPUT

        # Active Reset is low
        reset.status = 1

        # Erase the target chip
        self.erase(spi_interface, reset, device)

        # Set the needed function to check flash write page status (poll flash or rdy/bsy cmd)
        if device["busy_poll"] == '0x00':
            self.busy_wait = self._wait_poll_rdybsy
        else:
            self.busy_wait = self._wait_poll_flash

        # Loading firmware
        firmware = self.loading_firmware(self.options["firmware"]["Value"])
        if len(firmware) > int(device["flash_size"], 16):
            self.logger.handle("The firmware size is larger than the flash size, exiting..", self.logger.ERROR)
            return

        # Program the device
        self.write(spi_interface, reset, device, firmware)

    def run(self):
        """
        Main function.
        Write the flash memory of an AVR device.
        :return: Nothing.
        """
        # If detect_octowire is True then Detect and connect to the Octowire hardware. Else, connect to the Octowire
        # using the parameters that were configured. It sets the self.owf_serial variable if the hardware is found.
        self.connect()
        if not self.owf_serial:
            return
        try:
            self.process()
        except ValueError as err:
            self.logger.handle(err, self.logger.ERROR)
        except Exception as err:
            self.logger.handle("{}: {}".format(type(err).__name__, err), self.logger.ERROR)
