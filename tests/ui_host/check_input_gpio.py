"""Check generated key-port/pin layouts and input polarity on real ui_input.c."""
from pathlib import Path
import re, shutil, subprocess

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/key-size-r128/gpio'
OUT.mkdir(parents=True,exist_ok=True)
names=('KEY_Up','KEY_Down','KEY_OK','KEY_Left','KEY_Right','KEY2')
includes=('mycode','mycode/kk_ui/include','tests/ui_host')
driver=r'''
#include <assert.h>
#include <stdio.h>
#include "main.h"
#include "ui_input.h"
#include "kk_ui.h"
GPIO_TypeDef model_gpio[3];
static unsigned reads;
static const struct { GPIO_TypeDef *port; uint16_t pin; uint8_t key; } inputs[]={
 {KEY_Up_GPIO_Port,KEY_Up_Pin,KK_UI_KEY_UP},
 {KEY_Down_GPIO_Port,KEY_Down_Pin,KK_UI_KEY_DOWN},
 {KEY_OK_GPIO_Port,KEY_OK_Pin,KK_UI_KEY_OK},
 {KEY_Left_GPIO_Port,KEY_Left_Pin,KK_UI_KEY_LEFT},
 {KEY_Right_GPIO_Port,KEY_Right_Pin,KK_UI_KEY_RIGHT},
 {KEY2_GPIO_Port,KEY2_Pin,KK_UI_KEY_BACK}
};
uint32_t HAL_GetTick(void) { return 1000; }
uint8_t App_NavigationEvent(uint32_t now) { (void)now;return 0; }
bool KK_UI_IsTransitioning(void) { return false; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin)
 { ++reads;return (port->IDR&pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
int main(void) {
 ui_input_init();
 for(unsigned expected=0;expected<64;++expected) {
  for(unsigned p=0;p<3;++p) model_gpio[p].IDR=0xffff;
  for(unsigned i=0;i<6;++i) {
   bool high=((expected^UI_INPUT_ACTIVE_HIGH_MASK)&inputs[i].key)==0;
   if(!high) inputs[i].port->IDR&=~inputs[i].pin;
  }
  unsigned before=reads;
  ui_input_tick();
  assert(reads-before==EXPECTED_READS);
  before=reads;
  assert(ui_input_poll().keys==expected && ui_input_raw_keys()==expected);
  assert(reads==before);
 }
 puts("PASS: all 64 physical key combinations, generated layout and active polarity");
}
'''
(OUT/'driver.c').write_text(driver)
base=(ROOT/'tests/ui_host/main.h').read_text().replace('extern GPIO_TypeDef host_gpio;',
    'extern GPIO_TypeDef model_gpio[3];')
for layout,ports,bits in (
        ('common',[0]*6,[0,15,3,7,9,12]),
        ('split',[0,1,2,0,1,2],[14,2,9,3,15,0])):
    header=base
    for name,port,bit in zip(names,ports,bits):
        header=re.sub(r'#define '+name+r'_GPIO_Port[^\n]*',
            f'#define {name}_GPIO_Port (&model_gpio[{port}])',header)
        header=re.sub(r'#define '+name+r'_Pin[^\n]*',f'#define {name}_Pin (1U<<{bit})',header)
    (OUT/'main.h').write_text(header)
    for polarity in (0,0x15,0x3f):
        exe=OUT/f'{layout}-{polarity}.exe'
        command=[shutil.which('gcc') or 'gcc','-std=c11','-O1','-Wall','-Wextra','-Werror',
            '-I'+str(OUT),*('-I'+str(ROOT/p) for p in includes),
            f'-DUI_INPUT_ACTIVE_HIGH_MASK={polarity}',
            '-DEXPECTED_READS='+('0' if layout=='common' else '6'),
            str(ROOT/'mycode/ui_input.c'),str(OUT/'driver.c'),'-o',str(exe)]
        subprocess.run(command,cwd=ROOT,check=True)
        subprocess.run([str(exe)],cwd=ROOT,check=True)
print('PASS: CubeMX GPIO remapping: shared/split ports, shuffled pins, three polarity masks')
