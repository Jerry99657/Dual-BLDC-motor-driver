#include <assert.h>
#include <stdio.h>
#include "main.h"
#include "ui_input.h"

GPIO_TypeDef host_gpio={0xffff};
static uint32_t now;
static bool scanning,transition;
static unsigned reads;
static uint8_t navigation;
uint32_t HAL_GetTick(void) { return now; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin)
 { assert(scanning);++reads;return port->IDR&pin ? GPIO_PIN_SET : GPIO_PIN_RESET; }
uint8_t App_NavigationEvent(uint32_t at)
 { (void)at;assert(!scanning);uint8_t e=navigation;navigation=0;return e; }
bool KK_UI_IsTransitioning(void) { assert(!scanning);return transition; }
static const uint16_t pins[]={KEY_Up_Pin,KEY_Down_Pin,KEY_OK_Pin,KEY_Left_Pin,KEY_Right_Pin,KEY2_Pin};
static void scan(uint8_t keys,unsigned ms) {
    while(ms--) {
        host_gpio.IDR=0xffff;
        for(unsigned i=0;i<6;++i) if(keys&(1U<<i)) host_gpio.IDR&=~pins[i];
        ++now;scanning=true;ui_input_tick();scanning=false;
    }
}
static uint8_t peek(void) {
    unsigned before=reads;KK_UI_Input in=ui_input_poll();
    assert(reads==before);return in.key_event;
}
static uint8_t take(void) { uint8_t e=peek();KK_UI_InputAccepted();return e; }
static void boot(void) {
    transition=false;navigation=0;ui_input_init();scan(0,25);ui_input_context(1);
}
static void tap(uint8_t key) { scan(key,21);scan(0,21); }
static uint8_t event(unsigned key,KK_UI_InputSource source) { return KK_UI_KEY_EVENT(key,source); }
int main(void) {
    _Static_assert(sizeof(KK_UI_Input)==4,"Input frame must retain the original size");
    boot();host_gpio.IDR=0;assert(ui_input_poll().keys==0 && ui_input_raw_keys()==0);
    for(unsigned key=0;key<6;++key) {
        boot();tap(1U<<key); /* No foreground visit during press or release. */
        assert(take()==event(key,KK_UI_INPUT_PRESS));
        if(key==3 || key==4) assert(take()==event(key,KK_UI_INPUT_RELEASE));
        assert(!take());
    }
    boot();tap(KK_UI_KEY_UP);tap(KK_UI_KEY_DOWN);tap(KK_UI_KEY_OK);
    assert(take()==event(0,KK_UI_INPUT_PRESS));assert(take()==event(1,KK_UI_INPUT_PRESS));
    assert(take()==event(2,KK_UI_INPUT_PRESS));assert(!take());
    boot();tap(KK_UI_KEY_OK);tap(KK_UI_KEY_OK);
    assert(take()==event(2,KK_UI_INPUT_PRESS));assert(take()==event(2,KK_UI_INPUT_PRESS));assert(!take());
    boot();tap(KK_UI_KEY_OK);tap(KK_UI_KEY_BACK);
    assert(take()==event(5,KK_UI_INPUT_PRESS));assert(!take()); /* Back cancels a queued START. */
    boot();for(unsigned i=0;i<6;++i) { scan(KK_UI_KEY_UP,5);scan(0,5); }assert(!take());
    for(unsigned i=0;i<35;++i) scan(KK_UI_KEY_BACK|((i/5)%2 ? KK_UI_KEY_OK : 0),1);
    scan(0,25);assert(take()==event(5,KK_UI_INPUT_PRESS));assert(!take());
    boot();scan(KK_UI_KEY_UP|KK_UI_KEY_DOWN,1000);scan(0,25);assert(!take());
    boot();scan(KK_UI_KEY_UP,900);scan(0,25);
    assert(take()==event(0,KK_UI_INPUT_PRESS));assert(take()==event(0,KK_UI_INPUT_REPEAT));assert(!take());
    boot();scan(KK_UI_KEY_RIGHT,700);scan(0,25);
    unsigned longs=0,releases=0;uint8_t e;
    while((e=take())) { longs+=e==event(4,KK_UI_INPUT_LONG);releases+=e==event(4,KK_UI_INPUT_RELEASE); }
    assert(longs==1 && releases==1);
    boot();scan(KK_UI_KEY_RIGHT,21);assert(take()==event(4,KK_UI_INPUT_PRESS));
    scan(KK_UI_KEY_RIGHT|KK_UI_KEY_UP,35);scan(0,25);
    assert(!take()); /* A chord cancels the pending L/R gesture even with no foreground scan. */
    boot();tap(KK_UI_KEY_UP);assert(peek()==event(0,KK_UI_INPUT_PRESS));
    assert(peek()==event(0,KK_UI_INPUT_PRESS)); /* Check failed before UI acceptance. */
    ui_input_complete(UI_INPUT_DISPLAY_ERROR);assert(take()==event(0,KK_UI_INPUT_PRESS));assert(!take());
    tap(KK_UI_KEY_UP);assert(take()==event(0,KK_UI_INPUT_PRESS));
    ui_input_complete(UI_INPUT_DISPLAY_ERROR);assert(!take()); /* Failure after routing cannot replay. */
    tap(KK_UI_KEY_UP);tap(KK_UI_KEY_OK);ui_input_complete(UI_INPUT_DISPLAY_ERROR);
    assert(take()==event(0,KK_UI_INPUT_PRESS));assert(!take());
    tap(KK_UI_KEY_UP);assert(peek()==event(0,KK_UI_INPUT_PRESS));
    ui_input_complete(UI_INPUT_CANCEL);tap(KK_UI_KEY_DOWN);KK_UI_InputAccepted();
    assert(take()==event(1,KK_UI_INPUT_PRESS)); /* Old ACK cannot consume a new event. */
    boot();for(unsigned i=0;i<17;++i) tap(KK_UI_KEY_UP);assert(!take());
    tap(KK_UI_KEY_OK);assert(take()==event(2,KK_UI_INPUT_PRESS));assert(!take());
    boot();tap(KK_UI_KEY_UP);assert(peek()==event(0,KK_UI_INPUT_PRESS));
    for(unsigned i=0;i<16;++i) tap(KK_UI_KEY_UP); /* IRQ overflow while a frame is outstanding. */
    tap(KK_UI_KEY_DOWN);KK_UI_InputAccepted();assert(take()==event(1,KK_UI_INPUT_PRESS));assert(!take());
    boot();for(unsigned i=0;i<16;++i) tap(KK_UI_KEY_UP);tap(KK_UI_KEY_BACK);
    assert(take()==event(5,KK_UI_INPUT_PRESS));assert(!take());
    boot();tap(KK_UI_KEY_OK);scan(0,1001);assert(!take());
    tap(KK_UI_KEY_UP);navigation=event(5,KK_UI_INPUT_PRESS);
    assert(take()==event(5,KK_UI_INPUT_PRESS));assert(take()==event(0,KK_UI_INPUT_PRESS));assert(!take());
    boot();tap(KK_UI_KEY_OK);ui_input_context(2);assert(!take());
    scan(KK_UI_KEY_OK,25);ui_input_context(3);scan(KK_UI_KEY_OK,800);assert(!take());
    scan(0,25);tap(KK_UI_KEY_OK);assert(take()==event(2,KK_UI_INPUT_PRESS));
    now=0xfffffff0;boot();tap(KK_UI_KEY_LEFT);
    assert(take()==event(3,KK_UI_INPUT_PRESS));assert(take()==event(3,KK_UI_INPUT_RELEASE));assert(!take());
    puts("PASS: unified six-key timer input: stalled foreground, FIFO order/double taps, debounce/chords, coalesced repeat, long/release, accept/retry, cancel/late ACK, full/expiry, navigation and tick wrap; no foreground GPIO reads");
}
