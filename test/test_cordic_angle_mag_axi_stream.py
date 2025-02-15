#
# 2025 Taras Zaporozhets <zaporozhets.taras@gmail.com>
#

import itertools
import logging
import os
import struct
import math
import numpy as np

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from cocotbext.axi import AxiStreamFrame, AxiStreamBus, AxiStreamSource, AxiStreamSink

from helpers import setup_test, rtl_dir


class TB:
    def __init__(self, dut):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        cocotb.start_soon(Clock(dut.aclk, 2, units="ns").start())
        self.source = AxiStreamSource(AxiStreamBus.from_prefix(dut, "s_axis"), dut.aclk, dut.aresetn, False)
        self.sink = AxiStreamSink(AxiStreamBus.from_prefix(dut, "m_axis"), dut.aclk, dut.aresetn, False)

        self.log.setLevel(logging.CRITICAL)
        self.sink.log.setLevel(logging.CRITICAL)
        self.source.log.setLevel(logging.CRITICAL)
    def set_idle_generator(self, generator=None):
        if generator:
            self.source.set_pause_generator(generator())

    def set_backpressure_generator(self, generator=None):
        if generator:
            self.sink.set_pause_generator(generator())

    async def reset(self):
        self.dut.aresetn.setimmediatevalue(0)
        await RisingEdge(self.dut.aclk)
        await RisingEdge(self.dut.aclk)
        self.dut.aresetn.value = 0
        await RisingEdge(self.dut.aclk)
        await RisingEdge(self.dut.aclk)
        self.dut.aresetn.value = 1
        await RisingEdge(self.dut.aclk)
        await RisingEdge(self.dut.aclk)


def cycle_pause():
    return itertools.cycle([1, 1, 1, 0])


def byte_array_to_uint16_arrays(byte_array):
    # Check if the length of the byte array is even
    if len(byte_array) % 2 != 0:
        raise ValueError("The length of the byte array must be even to interpret as 16-bit integers.")

    # Calculate the number of 16-bit integers
    num_uint16 = len(byte_array) // 2

    # Interpret the byte array as an array of 16-bit integers
    uint16_array = struct.unpack('<' + 'H' * num_uint16, byte_array)

    # Separate the lower and upper 16-bit integers into two arrays
    return uint16_array[::2], uint16_array[1::2]

@cocotb.test()
async def run_test_basic(dut):
    tb = TB(dut)
    await tb.reset()

    tb.set_idle_generator(cycle_pause)
    tb.set_backpressure_generator(cycle_pause)

    test_angles = np.linspace(0, 2 * math.pi, 32)
    input_data = bytearray()
    x_val = []
    y_val = []
    for test_angle in test_angles:

        x = int(math.cos(test_angle) * (pow(2, 15)-1))
        y = int(math.sin(test_angle) * (pow(2, 15)-1))

        x_val.append(x)
        y_val.append(y)

        input_data += x.to_bytes(2, byteorder = 'little', signed = True)
        input_data += y.to_bytes(2, byteorder = 'little', signed = True)

    test_frame = AxiStreamFrame(input_data)
    await tb.source.send(test_frame)

    output_data = await tb.sink.recv()

    byte_array_to_uint16_arrays(bytes(output_data))



def test_main():
    setup_test(
        "test_cordic_angle_mag_axi_stream",
        "cordic_angle_mag_axi_stream",
        [
            os.path.join(rtl_dir, "cordic_angle_mag_axi_stream.sv"),
        ]
    )
