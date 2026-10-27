"""Fail after CubeMX changes a fixed LL runtime pin/channel/format contract."""
from pathlib import Path
import re

root=Path(__file__).resolve().parents[2]
spi=(root/'Core/Src/spi.c').read_text(encoding='utf-8')
dma=(root/'Core/Src/dma.c').read_text(encoding='utf-8')
def assigned(field,expected):
    values=re.findall(re.escape(field)+r'\s*=\s*([^;]+);',spi)
    assert values==[expected],(field,values)
for port in range(1,4):
    for field,value in {
        'Mode':'SPI_MODE_MASTER','Direction':'SPI_DIRECTION_2LINES',
        'DataSize':'SPI_DATASIZE_8BIT','CLKPolarity':'SPI_POLARITY_LOW',
        'NSS':'SPI_NSS_SOFT','FirstBit':'SPI_FIRSTBIT_MSB',
        'TIMode':'SPI_TIMODE_DISABLE','CRCCalculation':'SPI_CRCCALCULATION_DISABLE',
    }.items():assigned(f'hspi{port}.Init.{field}',value)
for name,instance,request,direction,priority in (
    ('spi1_rx','DMA1_Channel1','SPI1_RX','PERIPH_TO_MEMORY','LOW'),
    ('spi1_tx','DMA1_Channel2','SPI1_TX','MEMORY_TO_PERIPH','LOW'),
    ('spi2_rx','DMA2_Channel1','SPI2_RX','PERIPH_TO_MEMORY','HIGH'),
    ('spi2_tx','DMA2_Channel2','SPI2_TX','MEMORY_TO_PERIPH','HIGH'),
    ('spi3_tx','DMA1_Channel4','SPI3_TX','MEMORY_TO_PERIPH','LOW'),
):
    assigned('hdma_'+name+'.Instance',instance)
    for field,value in {
        'Request':'DMA_REQUEST_'+request,'Direction':'DMA_'+direction,
        'PeriphInc':'DMA_PINC_DISABLE','MemInc':'DMA_MINC_ENABLE',
        'PeriphDataAlignment':'DMA_PDATAALIGN_BYTE','MemDataAlignment':'DMA_MDATAALIGN_BYTE',
        'Mode':'DMA_NORMAL','Priority':'DMA_PRIORITY_'+priority,
    }.items():assigned('hdma_'+name+'.Init.'+field,value)
for irq,priority in {
    'DMA1_Channel1':1,'DMA1_Channel2':1,'DMA1_Channel3':5,'DMA1_Channel4':4,
    'DMA1_Channel5':2,'DMA2_Channel1':1,'DMA2_Channel2':1,
}.items():
    assert f'HAL_NVIC_SetPriority({irq}_IRQn, {priority}, 0);' in dma,irq
for port,priority in ((1,1),(2,1),(3,4)):
    assert f'HAL_NVIC_SetPriority(SPI{port}_IRQn, {priority}, 0);' in spi,port
print('PASS: actual CubeMX SPI1/2/3 fixed 8-bit master, five DMA channel/request/direction/alignment mappings and all existing priorities')
