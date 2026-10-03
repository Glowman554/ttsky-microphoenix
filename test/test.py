import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer


CLOCK_PERIOD_PS = 37_037 # 27 MHz
UART_BIT_PS = 233 * CLOCK_PERIOD_PS
PSRAM_MASK = (1 << 23) - 1


def load_flash_image(filename):
    contents = {}
    with open(filename, encoding="ascii") as image:
        for address, line in enumerate(image):
            word = line.strip().split()[0]
            contents[address] = int(word, 16)
    return contents


def write_miso(uio_in, value):
    current = int(uio_in.value)
    uio_in.value = (current & ~0x04) | ((value & 1) << 2)


async def spi_memory_model(cs_n, sclk, mosi, uio_in, memory, *, flash=False):
    reset_enabled = False
    has_been_reset = False

    while True:
        await FallingEdge(cs_n)
        bit_count = 0
        command = 0
        address = 0
        incoming_byte = 0
        write_index = 0

        while int(cs_n.value) == 0:
            await RisingEdge(sclk)
            await Timer(1, unit="ps")
            if not mosi.value.is_resolvable:
                if bit_count >= 32 and command in (0x02, 0x03, 0x0B):
                    bit = 0
                else:
                    raise AssertionError(f"SPI MOSI is {mosi.value} with CS={cs_n.value}, ", f"SCLK={sclk.value}, bit={bit_count}, command=0x{command:02x}")
            else:
                bit = int(mosi.value)
            bit_count += 1

            if bit_count <= 8:
                command = (command << 1) | bit
                if bit_count == 8 and not flash:
                    assert command in (0x02, 0x03, 0x0B, 0x66, 0x99, 0x9F), (f"unsupported APS6404L command 0x{command:02x}")
                    if command not in (0x66, 0x99, 0x9F):
                        assert has_been_reset, "PSRAM command before reset"
            elif bit_count <= 32:
                address = (address << 1) | bit
            elif command == 0x02 and not flash:
                incoming_byte = (incoming_byte << 1) | bit
                if (bit_count - 32) % 8 == 0:
                    memory[(address + write_index) & PSRAM_MASK] = incoming_byte
                    write_index += 1
                    incoming_byte = 0

            await FallingEdge(sclk)
            await Timer(1, unit="ps")
            if int(cs_n.value) != 0:
                break

            if command in (0x03, 0x0B):
                data_bit = bit_count - 32
                if data_bit >= 0:
                    data_address = address + data_bit // 8
                    if flash:                        
                        out_byte = memory.get(data_address & 0xFFFFFF, 0xFF)
                    else:
                        out_byte = memory.get(data_address & PSRAM_MASK, 0)
                    write_miso(uio_in, (out_byte >> (7 - data_bit % 8)) & 1)
            elif command == 0x9F:
                data_bit = bit_count - 8
                if data_bit >= 0:
                    if flash:
                        id_bytes = (0xEF, 0x40, 0x18)
                    else:
                        id_bytes = (0x0D, 0x5D, 0x00)
                    out_byte = id_bytes[(data_bit // 8) % len(id_bytes)]
                    write_miso(uio_in, (out_byte >> (7 - data_bit % 8)) & 1)
            else:
                write_miso(uio_in, 0)

        if not flash and bit_count == 8:
            if command == 0x66:
                reset_enabled = True
            elif command == 0x99:
                assert reset_enabled, "APS6404L reset without reset-enable"
                has_been_reset = True
                reset_enabled = False
            elif command != 0x9F:
                reset_enabled = False


async def uart_receiver(tx, received):
    while True:
        await FallingEdge(tx)

        await Timer(UART_BIT_PS // 2, unit="ps")
        if int(tx.value) != 0:
            continue

        value = 0
        for bit in range(8):
            await Timer(UART_BIT_PS, unit="ps")
            value |= int(tx.value) << bit

        await Timer(UART_BIT_PS, unit="ps")

        assert int(tx.value) == 1, "UART stop bit was low"

        received.append(value)


@cocotb.test()
async def test_firmware_uart(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_PS, unit="ps", period_high=CLOCK_PERIOD_PS // 2,).start())

    flash = load_flash_image(os.path.join(os.path.dirname(__file__), "firmware.hex"))
    psram_a = {}
    psram_b = {}

    cocotb.start_soon(spi_memory_model(dut.flash_cs_n, dut.spi_sclk, dut.spi_mosi, dut.uio_in, flash, flash=True))
    cocotb.start_soon(spi_memory_model(dut.ram_a_cs_n, dut.spi_sclk, dut.spi_mosi, dut.uio_in, psram_a))
    cocotb.start_soon(spi_memory_model(dut.ram_b_cs_n, dut.spi_sclk, dut.spi_mosi, dut.uio_in, psram_b))

    received = []
    cocotb.start_soon(uart_receiver(dut.uart_tx, received))

    dut.ena.value = 1
    dut.ui_in.value = 0x02 
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await Timer(256_000_000, unit="ns")

    output = bytes(received)
    dut._log.info("UART output (%d bytes): %r", len(output), output)

    with open("expected.bin", "rb") as f:
        expected = f.read()

    assert bytes(received) in expected, (
        "firmware did not print its expected output; "
        f"received {bytes(received)!r}"
    )
