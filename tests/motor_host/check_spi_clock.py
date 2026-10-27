"""Audit the actual generated C; do not pin a supported SPI divider to R58."""
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'scripts'))
from check_cubemx import audit, clean, value
audit(root)
spi=clean((root/'Core/Src/spi.c').read_text(encoding='utf-8'))
for port in (1,2,3):
    divider=int(value(spi,f'hspi{port}.Init.BaudRatePrescaler').rsplit('_',1)[1])
    print(f'PASS: SPI{port} generated divider /{divider}, initial SCK {170000000/divider:g} Hz; SPI2 runtime device modes remain an explicit exception')
