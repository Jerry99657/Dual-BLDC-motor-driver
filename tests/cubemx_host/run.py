"""Regeneration fixtures: change generated sources in scratch copies only."""
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'scripts'))
from check_cubemx import audit, ConfigError, function

output = root / 'build/cubemx-validation'
output.mkdir(parents=True, exist_ok=True)
if not (root/'build/spi-validation/include/spi_constants.h').exists():
    subprocess.run([sys.executable, 'tests/spi_host/run.py'], cwd=root, check=True)
original = {p.relative_to(root): p.read_text(encoding='utf-8')
            for p in (root / 'Core').rglob('*') if p.suffix in ('.c', '.h')}
for name in ('CMakeLists.txt', 'USB_Device/Target/usbd_conf.c', 'USB_Device/Target/usbd_conf.h', 'USB_Device/App/usbd_desc.c'):
    original[Path(name)] = (root/name).read_text(encoding='utf-8')
mutations = 0


def fixture(edits, accepted):
    global mutations
    with tempfile.TemporaryDirectory(dir=output) as directory:
        scratch = Path(directory)
        for name, text in original.items():
            for before, after in edits.get(str(name).replace('\\', '/'), ()):
                assert before in text, before
                text = text.replace(before, after)
            destination = scratch / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(text, encoding='utf-8')
        try:
            audit(scratch)
        except ConfigError:
            if accepted: raise
        else:
            assert accepted, edits
    mutations += 1


audit(root)
fixture({'Core/Src/tim.c':[('DeadTime = 17;','DeadTime = 128;'),('TIM_CLOCKDIVISION_DIV1','TIM_CLOCKDIVISION_DIV4')]},True)
fixture({'Core/Src/tim.c':[('DeadTime = 17;','DeadTime = 255;'),('TIM_CLOCKDIVISION_DIV1','TIM_CLOCKDIVISION_DIV2')]},True)
fixture({'Core/Src/tim.c':[('DeadTime = 17;','DeadTime = 256;')]},False)
fixture({'Core/Src/tim.c':[('TIM_CLOCKDIVISION_DIV1','TIM_CLOCKDIVISION_DIV8')]},False)
fixture({'Core/Src/tim.c':[('HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)','HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)')]},False)
fixture({'USB_Device/Target/usbd_conf.c': [('Init.lpm_enable = DISABLE;', 'Init.lpm_enable =   DISABLE ;')]},True)
fixture({'USB_Device/Target/usbd_conf.h': [('#define USBD_LPM_ENABLED     0U','#define USBD_LPM_ENABLED     1U')]},False)
fixture({'USB_Device/Target/usbd_conf.c': [('Init.lpm_enable = DISABLE','Init.lpm_enable = ENABLE')]},False)
fixture({'USB_Device/Target/usbd_conf.h': [('#define USBD_LPM_ENABLED     0U','#define USBD_LPM_ENABLED     1U')],
         'USB_Device/Target/usbd_conf.c': [('Init.lpm_enable = DISABLE','Init.lpm_enable = ENABLE')]},False)
fixture({'USB_Device/Target/usbd_conf.c': [('0x81 , PCD_SNG_BUF', '0x81 , PCD_DBL_BUF')]}, False)
fixture({'USB_Device/Target/usbd_conf.c': [('0x81 , PCD_SNG_BUF', '0x81 , PROJECT_PMA_MODE')]}, False)
fixture({'USB_Device/Target/usbd_conf.c': [('PCD_SNG_BUF, 0xC0', 'PCD_SNG_BUF, 0x180')]}, True)
fixture({'Core/Src/stm32g4xx_it.c': [('BOARD_DIAG_VECTOR(3);','')]},False)
fixture({'Core/Src/main.c': [('BoardDiag_Init();','')]},False)
fixture({'Core/Src/main.c': [('BoardDiag_LastCrash()->kind','0')]},False)
fixture({'Core/Src/main.c': [('HAL_Delay(30U);','')]},False)
fixture({'Core/Src/main.c': [('HAL_Delay(30U);','HAL_Delay(1U);')]},False)
# Supported changes must never be pinned to a historical divider/priority.
fixture({'Core/Src/spi.c': [('SPI_BAUDRATEPRESCALER_256;', 'SPI_BAUDRATEPRESCALER_128;'),
                           ('GPIO_SPEED_FREQ_MEDIUM;', 'GPIO_SPEED_FREQ_LOW;'),
                           ('DMA_PRIORITY_HIGH;', 'DMA_PRIORITY_VERY_HIGH;')]}, True)
uart_ll = 'LL_USART_Init(USART1,' in original[Path('Core/Src/usart.c')]
uart_prefix = 'LL_USART_' if uart_ll else 'UART_'
uart_width = 'DATAWIDTH' if uart_ll else 'WORDLENGTH'
uart_fifo = ('LL_USART_DisableFIFO(USART1)', 'LL_USART_EnableFIFO(USART1)') if uart_ll else (
    'HAL_UARTEx_DisableFifoMode(&huart1)', 'HAL_UARTEx_EnableFifoMode(&huart1)')
fixture({'Core/Src/usart.c': [('BaudRate = 115200;', 'BaudRate = 230400;'),
                             (uart_prefix+'STOPBITS_1;', uart_prefix+'STOPBITS_2;'),
                             (uart_prefix+'OVERSAMPLING_16;', uart_prefix+'OVERSAMPLING_8;')]}, True)
fixture({'Core/Src/tim.c': [('DeadTime = 17;', 'DeadTime = 34;')]}, True)
fixture({'Core/Src/tim.c': [('DMA_MDATAALIGN_HALFWORD;', 'DMA_MDATAALIGN_WORD;')]}, True)
fixture({'Core/Src/gpio.c': [('GPIO_PULLUP;', 'GPIO_NOPULL;')]}, True)
fixture({'Core/Src/adc.c': [('sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;',
                           'sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;')]}, True)
for file, before, after in (
    ('spi.c', 'SPI_DATASIZE_8BIT;', 'SPI_DATASIZE_16BIT;'),
    ('spi.c', 'DMA_NORMAL;', 'DMA_CIRCULAR;'),
    ('spi.c', 'DMA_MDATAALIGN_BYTE;', 'DMA_MDATAALIGN_HALFWORD;'),
    ('spi.c', 'DMA_REQUEST_SPI1_RX;', 'DMA_REQUEST_ADC1;'),
    ('spi.c', 'DMA1_Channel1;', 'DMA1_Channel2;'),
    ('usart.c', uart_prefix+uart_width+'_8B;', uart_prefix+uart_width+'_9B;'),
    ('usart.c', uart_prefix+'PARITY_NONE;', uart_prefix+'PARITY_EVEN;'),
    ('usart.c', *uart_fifo),
    ('adc.c', 'sConfigInjected.InjectedChannel = ADC_CHANNEL_6;', 'sConfigInjected.InjectedChannel = ADC_CHANNEL_7;'),
    ('adc.c', 'ADC_SAMPLETIME_47CYCLES_5;', 'ADC_SAMPLETIME_92CYCLES_5;'),
    ('adc.c', 'ADC_RESOLUTION_12B;', 'ADC_RESOLUTION_10B;'),
    ('adc.c', 'ADC_CLOCK_SYNC_PCLK_DIV4;', 'ADC_CLOCK_SYNC_PCLK_DIV2;'),
    ('adc.c', 'DMA_CIRCULAR;', 'DMA_NORMAL;'),
    ('tim.c', 'DMA_MDATAALIGN_HALFWORD;', 'DMA_MDATAALIGN_BYTE;'),
    ('tim.c', 'htim3.Init.CounterMode = TIM_COUNTERMODE_UP;', 'htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;'),
    ('tim.c', 'hdma[TIM_DMA_ID_CC1],hdma_tim3_ch1', 'hdma[TIM_DMA_ID_CC2],hdma_tim3_ch1'),
    ('tim.c', 'HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)',
              'HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)'),
    ('tim.c', 'htim1.Init.Period = 8499;', 'htim1.Init.Period = 4249;'),
    ('tim.c', 'htim2.Init.Period = 999;', 'htim2.Init.Period = 499;'),
    ('tim.c', 'TIM_BREAK_ENABLE;', 'TIM_BREAK_DISABLE;'),
    ('comp.c', 'COMP_INPUT_MINUS_DAC3_CH2;', 'COMP_INPUT_MINUS_VREFINT;'),
    ('main.c', 'RCC_HCLK_DIV1;', 'RCC_HCLK_DIV2;'),
    ('main.c', 'RCC_OscInitStruct.PLL.PLLN = 85;', 'RCC_OscInitStruct.PLL.PLLN = 80;'),
    ('gpio.c', 'GPIO_MODE_IT_FALLING;', 'GPIO_MODE_IT_RISING;'),
):
    fixture({'Core/Src/' + file: [(before, after)]}, False)
fixture({'Core/Src/stm32g4xx_it.c': [('HAL_DMA_IRQHandler(&hdma_spi1_rx);',
                                    'SpiPort_DMAIRQ(&hdma_spi2_rx);\n  return;')]}, False)

# Regenerate all seven channel assignments and the corresponding vectors.
mapping = {
    'hdma_spi1_rx': 'DMA2_Channel6', 'hdma_spi1_tx': 'DMA1_Channel6',
    'hdma_spi2_rx': 'DMA1_Channel4', 'hdma_spi2_tx': 'DMA2_Channel3',
    'hdma_spi3_tx': 'DMA2_Channel4', 'hdma_adc1': 'DMA1_Channel2',
    'hdma_tim3_ch1': 'DMA2_Channel1',
}
with tempfile.TemporaryDirectory(dir=output) as directory:
    scratch = Path(directory)
    irq_path = Path('Core/Src/stm32g4xx_it.c')
    for name, text in original.items():
        for handle, instance in mapping.items():
            text = re.sub(r'\b' + handle + r'\.Instance\s*=\s*DMA[12]_Channel\d\s*;',
                          f'{handle}.Instance = {instance};', text)
        if name == irq_path:
            text = re.sub(r'void DMA[12]_Channel\d_IRQHandler\(void\)\s*\{.*?^\}', '', text, flags=re.S | re.M)
            # A new KEY1 EXTI group contains the generated call and NO old
            # manually maintained hook. The shared header must route it.
            text = text.replace('EXTI15_10_IRQHandler', 'EXTI9_5_IRQHandler')
            for handle, instance in mapping.items():
                text += f'\nvoid {instance}_IRQHandler(void)\n{{\n  HAL_DMA_IRQHandler(&{handle});\n}}\n'
        if name == Path('Core/Inc/main.h'):
            text = text.replace('KEY1_Pin GPIO_PIN_13', 'KEY1_Pin GPIO_PIN_7')
        if name == Path('Core/Src/gpio.c'):
            text = text.replace('EXTI15_10_IRQn', 'EXTI9_5_IRQn')
        if name == Path('Core/Src/dma.c'):
            text = 'void MX_DMA_Init(void) {\n' + ''.join(
                f'HAL_NVIC_EnableIRQ({instance}_IRQn);\n' for instance in mapping.values()) + '}\n'
        destination = scratch / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding='utf-8')
    assert dict(audit(scratch)) == mapping
    # Compile actual newly generated vector bodies through the real USER CODE
    # routing header. Capture arguments to prove every new route follows its
    # generated handle, and moving KEY1 into EXTI9_5 keeps the stop callback.
    vectors = (scratch / irq_path).read_text(encoding='utf-8')
    code = '#include <assert.h>\n#include "spi.h"\nvoid HAL_GPIO_EXTI_IRQHandler(uint16_t pin);\n#define KEY1_Pin (1U<<7)\n#include "cubemx_irq.h"\n'
    code += 'SPI_HandleTypeDef ports[3];\nDMA_HandleTypeDef ' + ','.join(mapping) + ';\n'
    code += 'TIM_HandleTypeDef htim3={.hdma={&hdma_tim3_ch1}};\n'
    code += 'static DMA_HandleTypeDef *last,*led_last;static unsigned stopped,led_calls;\n'
    code += 'void SpiPort_DMAIRQ(DMA_HandleTypeDef *d) {last=d;}\nvoid MotorIRQ_Key1(void) {++stopped;}\n'
    code += 'void WS2812_DMAIRQ(DMA_HandleTypeDef *d) {led_last=d;++led_calls;}\n'
    for instance in mapping.values():
        code += f'void {instance}_IRQHandler(void) {{' + function(vectors, instance + '_IRQHandler') + '}\n'
    code += 'void EXTI9_5_IRQHandler(void) {' + function(vectors, 'EXTI9_5_IRQHandler') + '}\nint main(void) {\n'
    for handle, instance in mapping.items():
        if handle == 'hdma_tim3_ch1':
            code += f'last=0;{instance}_IRQHandler();assert(led_last==&{handle} && last==0);\n'
        else:
            code += f'{instance}_IRQHandler();assert(last==&{handle} && led_calls==0);\n'
    code += 'EXTI9_5_IRQHandler();assert(stopped==1 && led_calls==1);return 0;}\n'
    source = output / 'vectors.c'
    source.write_text(code, encoding='utf-8')
    compiler = shutil.which('gcc') or 'gcc'
    subprocess.run([compiler, '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                    '-Ibuild/spi-validation/include', '-Itests/cubemx_host', '-Itests/spi_host', '-Imycode',
                    str(source), '-o', str(output / 'vectors.exe')], cwd=root, check=True)
    subprocess.run([str(output / 'vectors.exe')], check=True)
print(f'PASS: {mutations} generated-config changes accepted/rejected; complete DMA1/2 reassignment and KEY1 EXTI-group regeneration compile and route to new handles')
