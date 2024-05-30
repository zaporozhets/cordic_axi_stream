# 2024 Taras Zaporozhets <zaporozhets.taras@gmail.com>

"""Module with helper functions for testing."""

import os
import cocotb_test.simulator

tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', 'rtl'))
sim_dir = os.path.abspath(os.path.join(tests_dir, '..', 'sim'))

def setup_test(module, toplevel, verilog_sources, parameters=None):
    """
    Set up the test environment for the given module.

    Args:
        module (str): Name of the module.
        toplevel (str): Name of the top-level module.
        verilog_sources (list): List of Verilog source files.

    Returns:
        None
    """
    sim_build = os.path.join("sim_build", module)

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=toplevel,
        module=module,
        sim_build=sim_build,
        includes=[rtl_dir],
        parameters=parameters
    )
