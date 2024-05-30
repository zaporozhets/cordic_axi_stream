# 2024 Taras Zaporozhets <zaporozhets.taras@gmail.com>

import itertools
import logging
import os
import struct
import math

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


def byte_array_to_int16_arrays(byte_array):
    # Check if the length of the byte array is even
    if len(byte_array) % 2 != 0:
        raise ValueError("The length of the byte array must be even to interpret as 16-bit integers.")

    # Calculate the number of 16-bit integers
    num_int16 = len(byte_array) // 2

    # Interpret the byte array as an array of 16-bit integers
    int16_array = struct.unpack('<' + 'h' * num_int16, byte_array)

    # Separate the lower and upper 16-bit integers into two arrays
    return int16_array[::2], int16_array[1::2]

@cocotb.test()
async def run_test_basic(dut):
    tb = TB(dut)
    await tb.reset()

    tb.set_idle_generator(cycle_pause)
#    tb.set_backpressure_generator(cycle_pause)

    test_angles = range(0, pow(2, 16), 1)
    input_data = bytearray()

    for i in test_angles:
        input_data += i.to_bytes(2, byteorder = 'little')

    test_frame = AxiStreamFrame(input_data)
    await tb.source.send(test_frame)

    output_data = await tb.sink.recv()

    out_sin, out_cos = byte_array_to_int16_arrays(bytes(output_data))

    delta_sin = []
    delta_cos = []
    for i, test_angle  in enumerate(test_angles):
        angle_rad = 2 * math.pi * test_angle / pow(2, 16)

        ref_sin = int(math.sin(angle_rad) * pow(2, 15))
        delta_sin.append(abs(ref_sin - out_sin[i]))

        ref_cos = int(math.cos(angle_rad) * pow(2, 15))
        delta_cos.append(abs(ref_cos - out_cos[i]))

    assert max(delta_sin) < 16
    assert max(delta_cos) < 16


def test_whitening():
    setup_test(
        "test_cordic_sin_cos_axi_stream",
        "cordic_sin_cos_axi_stream",
        [
            os.path.join(rtl_dir, "cordic_sin_cos_axi_stream.sv"),
        ]
    )
