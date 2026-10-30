"""Read generated C, never rewrite it. Reject unsupported application contracts.

Run on EVERY firmware build. DMA controller/channel and NVIC priority are not
fixed here: the generated handles and vectors remain their source of truth.
"""
from pathlib import Path
import argparse
import re
import sys


class ConfigError(ValueError):
    pass


def clean(text):
    return re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)


def function(text, name):
    match = re.search(r'\bvoid\s+' + re.escape(name) + r'\s*\([^)]*\)\s*\{', text)
    if not match:
        raise ConfigError(f'{name}: missing generated function; review application integration')
    start = match.end()
    depth = 1
    for at in range(start, len(text)):
        depth += (text[at] == '{') - (text[at] == '}')
        if depth == 0:
            return text[start:at]
    raise ConfigError(f'{name}: incomplete function')


def assignments(text, field):
    return re.findall(r'\b' + re.escape(field) + r'\s*=\s*([^;]+);', text)


def value(text, field):
    found = assignments(text, field)
    if len(found) != 1:
        raise ConfigError(f'{field}: expected one generated assignment, got {found}')
    return found[0].strip()


def require(text, field, expected):
    actual = value(text, field)
    if actual != expected:
        raise ConfigError(f'{field}={actual}: application currently supports {expected}; adapt driver/algorithm before building')


def audit(root):
    def read(name):
        return clean((root / name).read_text(encoding='utf-8'))

    spi, irq = read('Core/Src/spi.c'), read('Core/Src/stm32g4xx_it.c')
    dma = read('Core/Src/dma.c')
    errors = []
    # ST's public size option only removes an unused capability. PMA layout
    # and buffering remain generated; reject future incompatible regeneration.
    build_file = root / 'CMakeLists.txt'
    if build_file.exists() and re.search(r'\bUSE_USB_DOUBLE_BUFFER=0U?\b', clean(build_file.read_text(encoding='utf-8'))):
        usb_file = root / 'USB_Device/Target/usbd_conf.c'
        if not usb_file.exists():
            errors.append('USB single-buffer build: missing generated usbd_conf.c; review PMA configuration')
        else:
            pma = re.findall(r'\bHAL_PCDEx_PMAConfig\s*\([^;]+;', clean(usb_file.read_text(encoding='utf-8')))
            if not pma or any(call.split(',')[-2].strip() != 'PCD_SNG_BUF' for call in pma):
                errors.append('USB: USE_USB_DOUBLE_BUFFER=0 requires every generated PMA endpoint to use PCD_SNG_BUF; restore ST double-buffer support before selecting double buffers in CubeMX')
    projects = list(root.glob('*.ioc'))
    usb_header = root / 'USB_Device/Target/usbd_conf.h'
    if usb_header.exists():
        header=clean(usb_header.read_text(encoding='utf-8'))
        match=re.search(r'^\s*#define\s+USBD_LPM_ENABLED\s+([01])U?\b',header,re.M)
        config=read('USB_Device/Target/usbd_conf.c')
        if not match or [x.strip() for x in assignments(config,'hpcd_USB_FS.Init.lpm_enable')] != ['ENABLE' if match.group(1)=='1' else 'DISABLE']:
            errors.append('USB: middleware USBD_LPM_ENABLED and generated PCD lpm_enable must agree; configure both in CubeMX')
        elif match.group(1)=='1':
            descriptors=read('USB_Device/App/usbd_desc.c')
            table=re.search(r'\bCDC_Desc\s*=\s*\{([^}]+)\}',descriptors)
            entries=table.group(1).split(',') if table else []
            if len(entries)<8 or entries[7].strip() in ('','NULL','0'):
                errors.append('USB LPM: generate a valid BOS descriptor callback before enabling LPM support')
        if projects:
            ioc=projects[0].read_text(encoding='utf-8')
            saved=re.search(r'^USB_DEVICE.USBD_LPM_ENABLED=([01])$',ioc,re.M)
            if saved and match and saved.group(1)!=match.group(1):
                errors.append('USB LPM: saved IOC and generated header differ; regenerate the USB code')
    if projects and 'ProjectManager.KeepUserCode=true' not in projects[0].read_text(encoding='utf-8'):
        errors.append('CubeMX: enable Keep User Code when re-generating to preserve the runtime IRQ integration')

    def check(action):
        try:
            action()
        except ConfigError as error:
            errors.append(str(error))

    # Generated initialization is executed, not replaced by a fixed register
    # image. General port supports either CPOL, CPHA and bit order. SPI2 has
    # two physical devices with their own documented phase/divider switch.
    for port in range(1, 4):
        body = function(spi, f'MX_SPI{port}_Init')
        if f'HAL_SPI_Init(&hspi{port})' not in body or 'return;' in body:
            errors.append(f'SPI{port}: generated initialization must execute; do not bypass it with fixed user setup')
        for field, expected in {
            'Instance': f'SPI{port}', 'Init.Mode': 'SPI_MODE_MASTER',
            'Init.Direction': 'SPI_DIRECTION_2LINES', 'Init.DataSize': 'SPI_DATASIZE_8BIT',
            'Init.NSS': 'SPI_NSS_SOFT', 'Init.TIMode': 'SPI_TIMODE_DISABLE',
            'Init.CRCCalculation': 'SPI_CRCCALCULATION_DISABLE',
        }.items():
            check(lambda p=port, f=field, e=expected: require(spi, f'hspi{p}.{f}', e))
        check(lambda p=port: function(irq, f'SPI{p}_IRQHandler'))
    for field, expected in {'CLKPolarity': 'SPI_POLARITY_LOW', 'FirstBit': 'SPI_FIRSTBIT_MSB',
                            'NSSPMode': 'SPI_NSS_PULSE_DISABLE'}.items():
        check(lambda f=field, e=expected: require(spi, 'hspi2.Init.' + f, e))

    # SSI zero is valid data. An open MISO must have the generated pull-up,
    # rather than a floating input that can look like angle/status/CRC zero.
    # Resolve the pin from CubeMX's own route, never assume PA6/PB14.
    raw_spi = function((root / 'Core/Src/spi.c').read_text(encoding='utf-8'), 'HAL_SPI_MspInit')
    gpio_blocks = re.findall(r'GPIO_InitStruct.Pin\s*=\s*([^;]+);(.*?)HAL_GPIO_Init\(\s*(GPIO[A-G])\s*,', spi, re.S)
    for port in (1, 2):
        route = re.findall(r'\bP([A-G])(\d+)\s*-+>\s*SPI'+str(port)+r'_MISO\b', raw_spi)
        supported = len(route)==1 and any(
            gpio=='GPIO'+route[0][0] and re.search(r'\bGPIO_PIN_'+route[0][1]+r'\b', pins) and
            re.search(r'GPIO_InitStruct\.Pull\s*=\s*GPIO_PULLUP\s*;', body)
            for pins,body,gpio in gpio_blocks)
        if not supported:
            errors.append(f'SPI{port} MISO: configure Pull-up in CubeMX and regenerate; SSI zero/open-wire validation requires a defined idle input')

    sources = '\n'.join(read('Core/Src/' + name) for name in ('spi.c', 'adc.c', 'tim.c'))
    mappings = re.findall(r'\b(hdma_\w+)\.Instance\s*=\s*(DMA[12]_Channel[1-6])\s*;', sources)
    used = {}
    for handle, instance in mappings:
        if instance in used:
            errors.append(f'{instance}: duplicate DMA owners {used[instance]} / {handle}')
        used[instance] = handle

        def route(h=handle, inst=instance):
            body = function(irq, inst + '_IRQHandler')
            if not re.search(r'\bHAL_DMA_IRQHandler\s*\(\s*&' + h + r'\s*\)', body) or 'return;' in body:
                raise ConfigError(f'{inst}: generated vector must dispatch its current handle &{h}; remove stale USER CODE routing')
            if f'HAL_NVIC_EnableIRQ({inst}_IRQn)' not in re.sub(r'\s+', '', dma):
                raise ConfigError(f'{inst}: enable its DMA interrupt in CubeMX')
        check(route)
    for name, request, direction in (
        ('spi1_rx', 'SPI1_RX', 'PERIPH_TO_MEMORY'), ('spi1_tx', 'SPI1_TX', 'MEMORY_TO_PERIPH'),
        ('spi2_rx', 'SPI2_RX', 'PERIPH_TO_MEMORY'), ('spi2_tx', 'SPI2_TX', 'MEMORY_TO_PERIPH'),
        ('spi3_tx', 'SPI3_TX', 'MEMORY_TO_PERIPH'),
    ):
        for field, expected in {
            'Request': 'DMA_REQUEST_' + request, 'Direction': 'DMA_' + direction,
            'PeriphInc': 'DMA_PINC_DISABLE', 'MemInc': 'DMA_MINC_ENABLE',
            'PeriphDataAlignment': 'DMA_PDATAALIGN_BYTE', 'MemDataAlignment': 'DMA_MDATAALIGN_BYTE',
            'Mode': 'DMA_NORMAL',
        }.items():
            check(lambda n=name, f=field, e=expected: require(spi, f'hdma_{n}.Init.{f}', e))
        member = 'hdmarx' if name.endswith('_rx') else 'hdmatx'
        if not re.search(r'__HAL_LINKDMA\s*\(\s*spiHandle\s*,\s*' + member + r'\s*,\s*hdma_' + name + r'\s*\)', spi):
            errors.append(f'hdma_{name}: missing generated parent/handle link')
    if '#include "cubemx_irq.h"' not in irq:
        errors.append('stm32g4xx_it.c USER CODE Includes must include cubemx_irq.h')

    # Baud rate, clock selection, stop bits and GPIO are generated normally.
    # Byte-oriented protocol does not support parity/9-bit/FIFO/flow control.
    uart = read('Core/Src/usart.c')
    uart_body = function(uart, 'MX_USART1_UART_Init')
    if 'LL_USART_Init(USART1,' in uart_body:
        for field, expected in {
            'DataWidth': 'LL_USART_DATAWIDTH_8B', 'Parity': 'LL_USART_PARITY_NONE',
            'TransferDirection': 'LL_USART_DIRECTION_TX_RX',
            'HardwareFlowControl': 'LL_USART_HWCONTROL_NONE',
        }.items():
            check(lambda f=field, e=expected: require(uart_body, 'USART_InitStruct.' + f, e))
        for call in ('LL_USART_DisableFIFO(USART1)', 'LL_USART_ConfigAsyncMode(USART1)',
                     'LL_USART_Enable(USART1)'):
            if call not in uart_body:
                errors.append('USART1: missing generated ' + call)
        if re.search(r'LL_USART_(?:EnableFIFO|Enable(?:DEMode|HalfDuplex|Irda|Smartcard)|Set(?:TXRXSwap|RXPinLevel|TXPinLevel|DataLevel))\s*\(', uart_body):
            errors.append('USART1: advanced mode needs transport adaptation before building')
    elif 'HAL_UART_Init(&huart1)' in uart_body:
        for field, expected in {
            'Instance': 'USART1', 'Init.WordLength': 'UART_WORDLENGTH_8B',
            'Init.Parity': 'UART_PARITY_NONE', 'Init.Mode': 'UART_MODE_TX_RX',
            'Init.HwFlowCtl': 'UART_HWCONTROL_NONE',
            'AdvancedInit.AdvFeatureInit': 'UART_ADVFEATURE_NO_INIT',
        }.items():
            check(lambda f=field, e=expected: require(uart, 'huart1.' + f, e))
        if 'HAL_UARTEx_DisableFifoMode(&huart1)' not in uart:
            errors.append('USART1: protocol requires CubeMX FIFO disabled')
    else:
        errors.append('USART1: execute CubeMX native HAL/LL initialization; do not substitute fixed user setup')
    if 'return;' in uart_body:
        errors.append('USART1: generated initialization must execute')
    if re.search(r'(?:DMA_REQUEST|LL_DMAMUX_REQ)_USART1_', uart):
        errors.append('USART1 transport currently owns byte interrupts; adapt the transport before enabling USART DMA')

    adc = read('Core/Src/adc.c')
    # Rank is an application signal mapping, not a generic ADC capability.
    for port, channels in ((1, ('1', '6', '7', '14')), (2, ('2', '8', '9', '5'))):
        def adc_layout(p=port, expected=channels):
            body = function(adc, f'MX_ADC{p}_Init')
            for f, e in {'Instance': f'ADC{p}', 'Init.Resolution': 'ADC_RESOLUTION_12B',
                         'Init.DataAlign': 'ADC_DATAALIGN_RIGHT', 'Init.OversamplingMode': 'DISABLE',
                         'Init.ClockPrescaler': 'ADC_CLOCK_SYNC_PCLK_DIV4', 'Init.GainCompensation': '0',
                         'Init.ContinuousConvMode': 'DISABLE', 'Init.LowPowerAutoWait': 'DISABLE'}.items():
                require(body, f'hadc{p}.{f}', e)
            got = assignments(body, 'sConfigInjected.InjectedChannel')
            if got != ['ADC_CHANNEL_' + c for c in expected]:
                raise ConfigError(f'ADC{p} injected rank mapping changed: {got}; update motor_sampling signal mapping/HSO skew')
            require(body, 'sConfigInjected.InjectedNbrOfConversion', '4')
            # HSO's voltage rank skew currently assumes this exact aperture.
            channel = rank = sampling = None
            scans = []
            for line in body.splitlines():
                assignment = re.search(r'sConfigInjected\.Injected(Channel|Rank|SamplingTime)\s*=\s*(\w+);', line)
                if assignment:
                    field, token = assignment.groups()
                    if field == 'Channel': channel = token
                    elif field == 'Rank': rank = token
                    else: sampling = token
                if 'HAL_ADCEx_InjectedConfigChannel(' in line:
                    scans.append((rank, channel, sampling))
            times = ('12CYCLES_5', '47CYCLES_5', '47CYCLES_5', '12CYCLES_5')
            expected_scans = [(f'ADC_INJECTED_RANK_{i}', 'ADC_CHANNEL_' + c, 'ADC_SAMPLETIME_' + t)
                              for i, (c, t) in enumerate(zip(expected, times), 1)]
            if scans != expected_scans:
                raise ConfigError(f'ADC{p}: injected rank/aperture changed; adapt HSO voltage-rank skew and signal mapping')
            for f, e in {'InjectedDiscontinuousConvMode': 'DISABLE', 'AutoInjectedConv': 'DISABLE',
                         'QueueInjectedContext': 'DISABLE', 'InjecOversamplingMode': 'DISABLE',
                         'InjectedSingleDiff': 'ADC_SINGLE_ENDED', 'InjectedOffsetNumber': 'ADC_OFFSET_NONE'}.items():
                require(body, 'sConfigInjected.' + f, e)
        check(adc_layout)
    for field, expected in {
        'multimode.Mode': 'ADC_DUALMODE_INJECSIMULT',
        'sConfigInjected.ExternalTrigInjecConv': 'ADC_EXTERNALTRIGINJEC_T1_TRGO2',
        'sConfigInjected.ExternalTrigInjecConvEdge': 'ADC_EXTERNALTRIGINJECCONV_EDGE_RISING',
        'hadc1.Init.NbrOfConversion': '2', 'hadc1.Init.ExternalTrigConv': 'ADC_EXTERNALTRIG_T6_TRGO',
        'hadc1.Init.ExternalTrigConvEdge': 'ADC_EXTERNALTRIGCONVEDGE_RISING',
        'hadc1.Init.DMAContinuousRequests': 'ENABLE',
    }.items():
        check(lambda f=field, e=expected: require(adc, f, e))
    if assignments(adc, 'sConfig.Channel') != ['ADC_CHANNEL_15', 'ADC_CHANNEL_VREFINT']:
        errors.append('ADC1 regular ranks must be [VBUS channel15, VREFINT]; update regular DMA layout before changing')
    if assignments(adc, 'sConfig.Rank') != ['ADC_REGULAR_RANK_1', 'ADC_REGULAR_RANK_2']:
        errors.append('ADC1 regular rank numbers changed; update VBUS/VREFINT buffer mapping')
    for field, expected in {
        'Request': 'DMA_REQUEST_ADC1', 'Direction': 'DMA_PERIPH_TO_MEMORY',
        'PeriphInc': 'DMA_PINC_DISABLE', 'MemInc': 'DMA_MINC_ENABLE',
        'PeriphDataAlignment': 'DMA_PDATAALIGN_HALFWORD', 'MemDataAlignment': 'DMA_MDATAALIGN_HALFWORD',
        'Mode': 'DMA_CIRCULAR',
    }.items():
        check(lambda f=field, e=expected: require(adc, 'hdma_adc1.Init.' + f, e))
    if 'HAL_ADC_AnalogWDGConfig' in adc:
        errors.append('ADC watchdog interrupt requires extending the application ADC IRQ dispatcher')

    # Algorithms presently contain 170 MHz / 10 kHz / 1 kHz constants. Reject
    # an unsupported rate rather than overwrite the generated PSC/ARR values.
    main = read('Core/Src/main.c')
    clock = function(main, 'SystemClock_Config')
    for field, expected in {
        'RCC_OscInitStruct.PLL.PLLSource': 'RCC_PLLSOURCE_HSE',
        'RCC_ClkInitStruct.SYSCLKSource': 'RCC_SYSCLKSOURCE_PLLCLK',
        'RCC_ClkInitStruct.AHBCLKDivider': 'RCC_SYSCLK_DIV1',
        'RCC_ClkInitStruct.APB1CLKDivider': 'RCC_HCLK_DIV1',
        'RCC_ClkInitStruct.APB2CLKDivider': 'RCC_HCLK_DIV1',
    }.items():
        check(lambda f=field, e=expected: require(clock, f, e))

    def rates():
        hse = re.search(r'#define\s+HSE_VALUE\s+\(?\s*(\d+)', read('Core/Inc/stm32g4xx_hal_conf.h'))
        m = int(value(clock, 'RCC_OscInitStruct.PLL.PLLM').split('DIV')[1])
        n = int(value(clock, 'RCC_OscInitStruct.PLL.PLLN'))
        r = int(value(clock, 'RCC_OscInitStruct.PLL.PLLR').split('DIV')[1])
        if not hse or int(hse[1]) * n != 170000000 * m * r:
            raise ConfigError('System clock: motor timing currently requires 170 MHz; update timing/observer constants first')
    check(rates)
    tim = read('Core/Src/tim.c')
    if not re.search(r'__HAL_LINKDMA\s*\(\s*tim_pwmHandle\s*,\s*hdma\[TIM_DMA_ID_CC1\]\s*,\s*hdma_tim3_ch1\s*\)', tim):
        errors.append('WS2812: keep the generated TIM3 CH1 DMA handle link; runtime follows this mapping')
    if re.search(r'\bhdma_tim3_ch[234]\b', tim):
        errors.append('WS2812 currently owns TIM3 exclusively; adapt runtime before enabling another TIM3 DMA channel')
    led_timer = function(tim, 'MX_TIM3_Init')
    if re.search(r'HAL_TIM_(?:PWM|OC|IC)_ConfigChannel\s*\(\s*&htim3,\s*&\w+,\s*TIM_CHANNEL_[234]', led_timer):
        errors.append('WS2812 stops TIM3 at frame completion; adapt runtime before sharing TIM3 with another channel')
    if re.search(r'HAL_TIM_(?:SlaveConfigSynchro|OnePulse_Init)\s*\(\s*&htim3', led_timer):
        errors.append('WS2812 requires a free-running internal-clock TIM3; slave/one-pulse modes need driver adaptation')
    if assignments(led_timer, 'sClockSourceConfig.ClockSource'):
        check(lambda: require(led_timer, 'sClockSourceConfig.ClockSource', 'TIM_CLOCKSOURCE_INTERNAL'))
    if re.search(r'HAL_NVIC_EnableIRQ\s*\(\s*TIM3_IRQn', tim):
        errors.append('WS2812 uses DMA IRQ only; adapt runtime before enabling TIM3 timer interrupts')
    for field, expected in {
        'Request': 'DMA_REQUEST_TIM3_CH1', 'Direction': 'DMA_MEMORY_TO_PERIPH',
        'PeriphInc': 'DMA_PINC_DISABLE', 'MemInc': 'DMA_MINC_ENABLE', 'Mode': 'DMA_NORMAL',
    }.items():
        check(lambda f=field, e=expected: require(tim, 'hdma_tim3_ch1.Init.' + f, e))
    for field, prefix in (('MemDataAlignment', 'DMA_MDATAALIGN_'), ('PeriphDataAlignment', 'DMA_PDATAALIGN_')):
        if value(tim, 'hdma_tim3_ch1.Init.' + field) not in (prefix + 'HALFWORD', prefix + 'WORD'):
            errors.append('WS2812 DMA requires halfword/word alignment; byte alignment needs driver adaptation')
    if re.search(r'HAL_TIM_IRQHandler\s*\(\s*&htim15\s*\)', irq):
        errors.append('TIM15 shares TIM1 Break vector: extend the application dispatcher before enabling TIM15 IRQ')
    for port in (1, 8):
        body = function(tim, f'MX_TIM{port}_Init')
        for field, expected in {'Instance': f'TIM{port}', 'Init.Prescaler': '0',
                                'Init.Period': '8499', 'Init.CounterMode': 'TIM_COUNTERMODE_CENTERALIGNED3',
                                'Init.RepetitionCounter': '0'}.items():
            check(lambda b=body, p=port, f=field, e=expected: require(b, f'htim{p}.{f}', e))
        for field, expected in {'OCMode': 'TIM_OCMODE_PWM1', 'OCPolarity': 'TIM_OCPOLARITY_HIGH',
                                'OCNPolarity': 'TIM_OCNPOLARITY_HIGH'}.items():
            check(lambda b=body, f=field, e=expected: require(b, 'sConfigOC.' + f, e))
        check(lambda b=body: require(b, 'sBreakDeadTimeConfig.AutomaticOutput', 'TIM_AUTOMATICOUTPUT_DISABLE'))
        # Encoder timing uses both center-aligned update boundaries, while
        # inverter compensation decodes the GENERATED DTG/CKD at runtime.
        def deadtime_format(b=body,p=port):
            if value(b,f'htim{p}.Init.ClockDivision') not in ('TIM_CLOCKDIVISION_DIV1','TIM_CLOCKDIVISION_DIV2','TIM_CLOCKDIVISION_DIV4'):
                raise ConfigError(f'TIM{p}: adapt inverter tDTS decoder before using another CKD format')
            try:
                dtg=int(value(b,'sBreakDeadTimeConfig.DeadTime'),0)
            except ValueError:
                raise ConfigError(f'TIM{p}: inverter compensation requires a numeric generated 8-bit DTG')
            if not 0<=dtg<=255:
                raise ConfigError(f'TIM{p}: DTG must fit its generated 8-bit field')
            for ch in (1,2,3):
                if not re.search(r'HAL_TIM_PWM_ConfigChannel\s*\(\s*&htim'+str(p)+r'\s*,\s*&sConfigOC\s*,\s*TIM_CHANNEL_'+str(ch)+r'\s*\)',b):
                    raise ConfigError(f'TIM{p}: encoder voltage commit requires generated PWM/preload setup for all phases')
        check(deadtime_format)
    for port, fields in {
        1: {'sMasterConfig.MasterOutputTrigger2': 'TIM_TRGO2_OC4REF',
            'sBreakInputConfig.Source': 'TIM_BREAKINPUTSOURCE_COMP4',
            'sBreakInputConfig.Enable': 'TIM_BREAKINPUTSOURCE_ENABLE',
            'sBreakDeadTimeConfig.BreakState': 'TIM_BREAK_ENABLE',
            'sBreakDeadTimeConfig.BreakPolarity': 'TIM_BREAKPOLARITY_HIGH',
            'sBreakDeadTimeConfig.OffStateRunMode': 'TIM_OSSR_ENABLE'},
        8: {'sSlaveConfig.SlaveMode': 'TIM_SLAVEMODE_TRIGGER', 'sSlaveConfig.InputTrigger': 'TIM_TS_ITR0'},
        # Outer controller and deadline monitor both assume an up-counting
        # 1 kHz TIM2 from the checked 170 MHz APB timer clock.
        2: {'htim2.Init.Prescaler': '169', 'htim2.Init.Period': '999',
            'htim2.Init.CounterMode': 'TIM_COUNTERMODE_UP',
            'sClockSourceConfig.ClockSource': 'TIM_CLOCKSOURCE_INTERNAL'},
        3: {'htim3.Init.Prescaler': '0', 'htim3.Init.Period': '212',
            'htim3.Init.CounterMode': 'TIM_COUNTERMODE_UP',
            'sConfigOC.OCMode': 'TIM_OCMODE_PWM1', 'sConfigOC.OCPolarity': 'TIM_OCPOLARITY_HIGH'},
        6: {'sMasterConfig.MasterOutputTrigger': 'TIM_TRGO_UPDATE'},
    }.items():
        body = function(tim, f'MX_TIM{port}_Init')
        for field, expected in fields.items():
            check(lambda b=body, f=field, e=expected: require(b, f, e))
    comp = read('Core/Src/comp.c')
    for field, expected in {'Instance': 'COMP4', 'Init.InputPlus': 'COMP_INPUT_PLUS_IO2',
                            'Init.InputMinus': 'COMP_INPUT_MINUS_DAC3_CH2',
                            'Init.OutputPol': 'COMP_OUTPUTPOL_NONINVERTED',
                            'Init.BlankingSrce': 'COMP_BLANKINGSRC_NONE'}.items():
        check(lambda f=field, e=expected: require(comp, 'hcomp4.' + f, e))
    dac = read('Core/Src/dac.c')
    for field, expected in {'hdac3.Instance': 'DAC3', 'sConfig.DAC_Trigger': 'DAC_TRIGGER_NONE',
                            'sConfig.DAC_SignedFormat': 'DISABLE',
                            'sConfig.DAC_SampleAndHold': 'DAC_SAMPLEANDHOLD_DISABLE',
                            'sConfig.DAC_ConnectOnChipPeripheral': 'DAC_CHIPCONNECT_INTERNAL'}.items():
        check(lambda f=field, e=expected: require(dac, f, e))
    if not re.search(r'HAL_DAC_ConfigChannel\s*\(&hdac3,\s*&sConfig,\s*DAC_CHANNEL_2\)', dac):
        errors.append('IBUS protection requires generated DAC3 channel2 initialization')

    # Pin labels follow main.h; dispatch follows CubeMX's newly generated
    # EXTI vector, even when KEY1 moves into a different EXTI group.
    key_routes = re.findall(r'HAL_GPIO_EXTI_IRQHandler\s*\(\s*KEY1_Pin\s*\)', irq)
    if len(key_routes) != 1:
        errors.append('KEY1: expected one generated EXTI call using KEY1_Pin')
    for name, body in re.findall(r'void (EXTI\w+_IRQHandler)\(void\)\s*\{(.*?)^\}', irq, re.S | re.M):
        if 'HAL_GPIO_EXTI_IRQHandler(KEY1_Pin)' in body and 'return;' in body:
            errors.append(f'{name}: remove stale early-return routing before the generated KEY1 call')
    gpio = read('Core/Src/gpio.c')
    key = re.search(r'GPIO_InitStruct.Pin\s*=\s*KEY1_Pin\s*;(.*?)HAL_GPIO_Init\(KEY1_GPIO_Port,', gpio, re.S)
    if not key or 'GPIO_MODE_IT_FALLING' not in key[1]:
        errors.append('KEY1 global stop requires generated falling-edge EXTI configuration')
    pin = re.search(r'#define\s+KEY1_Pin\s+GPIO_PIN_(\d+)\b', read('Core/Inc/main.h'))
    if not pin:
        errors.append('KEY1: keep the generated KEY1_Pin label for its input pin')
    else:
        number = int(pin[1])
        vector = f'EXTI{number}' if number < 5 else 'EXTI9_5' if number < 10 else 'EXTI15_10'
        if not re.search(r'HAL_NVIC_EnableIRQ\s*\(\s*' + vector + r'_IRQn\s*\)', gpio):
            errors.append(f'KEY1: enable {vector} for GPIO pin {number} in CubeMX')
        check(lambda: function(irq, vector + '_IRQHandler'))
    for text, names in ((spi, ('SPI1', 'SPI2', 'SPI3')), (uart, ('USART1',)),
                        (adc, ('ADC1_2',)), (tim, ('TIM2', 'TIM1_BRK_TIM15'))):
        for name in names:
            if not re.search(r'(?:HAL_)?NVIC_EnableIRQ\s*\(\s*' + name + r'_IRQn\s*\)', text):
                errors.append(f'{name}: enable the required application interrupt in CubeMX')
    # These application hooks must survive regeneration. Exception declarations
    # carry naked attributes; the generated definitions remain inside USER CODE.
    for name,kind in [('NMI_Handler',2),('HardFault_Handler',3),('MemManage_Handler',4),('BusFault_Handler',5),('UsageFault_Handler',6)]:
        try:
            if f'BOARD_DIAG_VECTOR({kind});' not in function(irq,name):
                errors.append(f'{name}: preserve the R95 USER CODE exception route')
        except ConfigError as error:
            errors.append(str(error))
    if '#include "board_diag.h"' not in irq or 'BoardDiag_Init();' not in main or 'BoardDiag_LastCrash()->kind' not in main:
        errors.append('Preserve R95 diagnostic initialization and boot output inhibit USER CODE hooks')
    if not re.search(r'g_boot_stage\s*=\s*4U;\s*'
                     r'if\s*\(BoardDiag_LastCrash\(\)->kind\)\s*MotorSampling_GlobalStopIRQ\(\);\s*'
                     r'HAL_Delay\(30U\);\s*\(void\)MotorSampling_IsGlobalStopped\(\);',main):
        errors.append('KEY1: preserve the R103 30ms RC settling before the first level poll, after CPU boot inhibition (main USER CODE 2)')
    if errors:
        raise ConfigError('\n'.join(errors))
    return mappings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        mappings = audit(args.root)
    except (ConfigError, ValueError, OSError) as error:
        print('CubeMX/application configuration mismatch:\n' + str(error), file=sys.stderr)
        return 1
    print('CubeMX configuration OK; generated DMA routes: ' + ', '.join(f'{h}={c}' for h, c in mappings))
    return 0


if __name__ == '__main__':
    sys.exit(main())
