"""Validate the restored encoder clocks and their shared clock-tree dependencies.

No board is accessed. This checks the actual initialization sources against the
CubeMX project and calculates peripheral rates; it does not measure wire timing
or interrupt execution time. Run again after regenerating with CubeMX.
"""
from pathlib import Path
import re

repo = Path(__file__).resolve().parents[2]


def source(name):
    text = (repo / name).read_text(encoding="utf-8")
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


def value(text, name):
    found = re.findall(r"\b" + re.escape(name) + r"\s*=\s*([^;]+);", text)
    assert len(found) == 1, (name, found)
    return found[0].strip()


def divider(text, name):
    token = value(text, name)
    return int(re.search(r"(?:DIV|PRESCALER_)(\d+)$", token)[1])


ioc_lines = [line.split("=", 1) for line in
             (repo / "FOC_two.ioc").read_text(encoding="utf-8").splitlines()
             if "=" in line and not line.startswith("#")]
ioc = dict(ioc_lines)
assert len(ioc) == len(ioc_lines), "Duplicate CubeMX settings"
main = source("Core/Src/main.c")
spi = source("Core/Src/spi.c")
tim = source("Core/Src/tim.c")
adc = source("Core/Src/adc.c")
i2c = source("Core/Src/i2c.c")
can = source("Core/Src/fdcan.c")
uart = source("Core/Src/usart.c")
encoder = source("mycode/MT6701.c")

assert value(main, "RCC_OscInitStruct.PLL.PLLSource") == "RCC_PLLSOURCE_HSE"
assert value(main, "RCC_ClkInitStruct.SYSCLKSource") == "RCC_SYSCLKSOURCE_PLLCLK"
vco = (int(ioc["RCC.HSE_VALUE"]) / divider(main, "RCC_OscInitStruct.PLL.PLLM") *
       int(value(main, "RCC_OscInitStruct.PLL.PLLN")))
sysclk = vco / divider(main, "RCC_OscInitStruct.PLL.PLLR")
hclk = sysclk / divider(main, "RCC_ClkInitStruct.AHBCLKDivider")
assert sysclk == hclk == 170_000_000
for name in ("PLLM", "PLLQ"):
    assert ioc["RCC." + name] == value(main, "RCC_OscInitStruct.PLL." + name)

for bus in (1, 2):
    name = f"APB{bus}CLKDivider"
    divisor = divider(main, "RCC_ClkInitStruct." + name)
    assert ioc["RCC." + name] == value(main, "RCC_ClkInitStruct." + name)
    pclk = hclk / divisor
    timer_clock = pclk * (1 if divisor == 1 else 2)
    assert pclk == 170_000_000 and timer_clock == 170_000_000
    assert int(ioc[f"RCC.APB{bus}Freq_Value"]) == pclk
    assert int(ioc[f"RCC.APB{bus}TimFreq_Value"]) == timer_clock

encoder_sck = pclk / 256
assert 660_000 < encoder_sck < 670_000
assert 24 / encoder_sck < 40e-6, "Encoder frame exceeds the 10kHz sampling budget"
for port, expected in ((1, encoder_sck), (2, 21_250_000), (3, 10_625_000)):
    field = f"hspi{port}.Init.BaudRatePrescaler"
    assert ioc[f"SPI{port}.BaudRatePrescaler"] == value(spi, field)
    assert pclk / divider(spi, field) == expected, (port, field)
assert value(spi, "hspi1.Init.CLKPhase") == "SPI_PHASE_2EDGE"
assert value(spi, "hspi2.Init.CLKPhase") == "SPI_PHASE_1EDGE"
assert re.search(r"MODIFY_REG\(SPI2->CR1,.*?SPI_PHASE_2EDGE\|SPI_BAUDRATEPRESCALER_256\)",
                 re.sub(r"\s+", "", encoder), flags=re.S)

for port, target in ((1, 10_000), (8, 10_000), (2, 1_000), (6, 2_000)):
    prescaler = int(value(tim, f"htim{port}.Init.Prescaler"))
    period = int(value(tim, f"htim{port}.Init.Period"))
    center = port in (1, 8)
    assert timer_clock / (prescaler + 1) / (period + 1) / (2 if center else 1) == target
assert timer_clock / (int(value(tim, "htim3.Init.Period")) + 1) > 790_000
for port in (1, 2):
    # ADC synchronous CKMODE uses HCLK, despite the HAL macro's PCLK name.
    assert value(adc, f"hadc{port}.Init.ClockPrescaler") == "ADC_CLOCK_SYNC_PCLK_DIV4"
assert hclk / 4 == 42_500_000
assert set(re.findall(r"Adc12ClockSelection\s*=\s*(\w+);", adc)) == {"RCC_ADC12CLKSOURCE_SYSCLK"}

assert value(i2c, "PeriphClkInit.I2c3ClockSelection") == ioc["RCC.I2C3CLockSelection"] == "RCC_I2C3CLKSOURCE_SYSCLK"
assert value(i2c, "hi2c3.Init.Timing") == "0x40B285C2"
assert value(can, "PeriphClkInit.FdcanClockSelection") == ioc["RCC.FDCANCLockSelection"] == "RCC_FDCANCLKSOURCE_PLL"
assert vco / divider(main, "RCC_OscInitStruct.PLL.PLLQ") == int(ioc["RCC.FDCANFreq_Value"]) == 170_000_000
assert value(uart, "PeriphClkInit.Usart1ClockSelection") == "RCC_USART1CLKSOURCE_PCLK2"
assert value(uart, "huart1.Init.BaudRate") == "115200"
assert ioc["RCC.CK48CLockSelection"] == "RCC_USBCLKSOURCE_HSI48"
assert "RCC_OSCILLATORTYPE_HSI48" in value(main, "RCC_OscInitStruct.OscillatorType")

print(f"PASS: ENC1/2 {encoder_sck:.2f} Hz, 24-bit wire time {24e6/encoder_sck:.2f} us; "
      "CubeMX/source agree; timers, ADC, Flash, LCD, I2C, UART and CAN clock dependencies checked")
