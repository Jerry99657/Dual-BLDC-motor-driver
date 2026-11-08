#include "ui_input.h"
#include "kk_ui.h"
#include "main.h"
#include "app_control.h"

/* CubeMX owns GPIO mode/pull. Default logic is active-low; active-high
 * boards may select KK_UI_KEY_* bits here and configure their pulls in IOC. */
#ifndef UI_INPUT_ACTIVE_HIGH_MASK
#define UI_INPUT_ACTIVE_HIGH_MASK 0U
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t key;
} InputPin;

static const InputPin s_pins[] = {
    {KEY_Up_GPIO_Port, KEY_Up_Pin, KK_UI_KEY_UP},
    {KEY_Down_GPIO_Port, KEY_Down_Pin, KK_UI_KEY_DOWN},
    {KEY_OK_GPIO_Port, KEY_OK_Pin, KK_UI_KEY_OK},
    {KEY_Left_GPIO_Port, KEY_Left_Pin, KK_UI_KEY_LEFT},
    {KEY_Right_GPIO_Port, KEY_Right_Pin, KK_UI_KEY_RIGHT},
    {KEY2_GPIO_Port, KEY2_Pin, KK_UI_KEY_BACK},
};

/* All six keys share one timed recognizer and one ordered event FIFO. */
typedef struct { uint32_t pressed_at; uint16_t changed_at,repeat_at; } KeyTime;
static volatile KeyTime s_times[KK_UI_KEY_COUNT];
typedef struct {
    uint32_t pending_at;
    uint16_t context;
    uint8_t raw,candidate,stable,armed,long_sent,suppressed,head,count;
    uint8_t events[16];
} InputState;
static volatile InputState s_input;
static uint8_t s_read_head,s_read_valid;
_Static_assert(KK_UI_KEY_DEBOUNCE_MS<32768U && KK_UI_KEY_REPEAT_DELAY_MS<32768U,
               "Key timing exceeds halfword deadline range");

uint8_t ui_input_raw_keys(void) { return s_input.raw; }
void ui_input_init(void)
{
    s_input=(InputState){0};
    /* Candidate/press edges seed their clocks. Logical reset invalidates old
     * timers, so no second initialization of the BSS timestamp array is needed. */
    s_read_valid=0U;
}

static uint8_t physical_keys(void)
{
    uint8_t keys = 0U;
    if (KEY_Up_GPIO_Port==KEY_Down_GPIO_Port && KEY_Up_GPIO_Port==KEY_OK_GPIO_Port &&
        KEY_Up_GPIO_Port==KEY_Left_GPIO_Port && KEY_Up_GPIO_Port==KEY_Right_GPIO_Port &&
        KEY_Up_GPIO_Port==KEY2_GPIO_Port) {
        /* One port snapshot when CubeMX puts the six inputs together. Pin
         * positions still come from main.h; other layouts use the table. */
        uint32_t low=~KEY_Up_GPIO_Port->IDR;
#define INPUT_BIT(pin,key) (((low>>__builtin_ctz((unsigned)(pin)))&1U)*(key))
        keys=(uint8_t)(INPUT_BIT(KEY_Up_Pin,KK_UI_KEY_UP) |
            INPUT_BIT(KEY_Down_Pin,KK_UI_KEY_DOWN) | INPUT_BIT(KEY_OK_Pin,KK_UI_KEY_OK) |
            INPUT_BIT(KEY_Left_Pin,KK_UI_KEY_LEFT) | INPUT_BIT(KEY_Right_Pin,KK_UI_KEY_RIGHT) |
            INPUT_BIT(KEY2_Pin,KK_UI_KEY_BACK));
#undef INPUT_BIT
        keys ^= UI_INPUT_ACTIVE_HIGH_MASK & KK_UI_KEY_MASK;
    } else {
        for (unsigned i = 0U; i < sizeof(s_pins) / sizeof(s_pins[0]); ++i) {
            bool high = HAL_GPIO_ReadPin(s_pins[i].port, s_pins[i].pin) == GPIO_PIN_SET;
            bool active_high = (UI_INPUT_ACTIVE_HIGH_MASK & s_pins[i].key) != 0U;
            if (high == active_high) keys |= s_pins[i].key;
        }
    }
    return keys;
}

static void queue_key(unsigned key,KK_UI_InputSource source,uint32_t now)
{
    uint8_t event=KK_UI_KEY_EVENT(key,source);
    if (source==KK_UI_INPUT_REPEAT && s_input.count &&
        s_input.events[(s_input.head+s_input.count-1U)&15U]==event) return;
    if (s_input.count==sizeof(s_input.events) || event==KK_UI_KEY_EVENT(KK_UI_INPUT_BACK,KK_UI_INPUT_PRESS)) {
        /* Back cancels queued operations instead of waiting behind a START.
         * Overflow cannot replay a START; KEY1 remains independent EXTI. */
        s_input.head += s_input.count;
        s_input.count=0U;
        s_input.suppressed |= s_input.raw | s_input.stable;
        if (event!=KK_UI_KEY_EVENT(KK_UI_INPUT_BACK,KK_UI_INPUT_PRESS)) return;
    }
    if (!s_input.count) s_input.pending_at=now;
    s_input.events[(s_input.head+s_input.count)&15U]=event;
    ++s_input.count;
}

__attribute__((noinline,noclone)) void ui_input_tick(void)
{
    uint8_t raw=physical_keys(),press=0U,release=0U;
    uint32_t now=HAL_GetTick();
    uint8_t changed=raw^s_input.candidate,stable=s_input.stable,armed=s_input.armed;
    uint8_t long_sent=s_input.long_sent;
    s_input.raw=raw;s_input.candidate=raw;
    if (s_input.count && now-s_input.pending_at>1000U) {
        s_input.head += s_input.count;
        s_input.count=0U;
        s_input.suppressed |= raw | s_input.stable;
    }
    for(unsigned i=0;i<KK_UI_KEY_COUNT;++i) {
        uint8_t bit=(uint8_t)(1U<<i);
        if (changed&bit) {
            s_times[i].changed_at=(uint16_t)now;
        }
        if ((uint16_t)(now-s_times[i].changed_at)<KK_UI_KEY_DEBOUNCE_MS) continue;
        if ((raw^stable)&bit) {
            stable ^= bit;
            if (raw&bit) {
                if (armed&bit) press |= bit;
                s_times[i].pressed_at=now;
                s_times[i].repeat_at=(uint16_t)(now+KK_UI_KEY_REPEAT_DELAY_MS);
                long_sent &= (uint8_t)~bit;
            } else {
                release |= bit;armed |= bit;
            }
        } else if (!(raw&bit)) armed |= bit;
    }
    s_input.stable=stable;s_input.armed=armed;
    /* Back/OK take precedence; opposite/chorded directions do not move UI. */
    uint8_t directions=stable & (KK_UI_KEY_UP|KK_UI_KEY_DOWN|KK_UI_KEY_LEFT|KK_UI_KEY_RIGHT);
    for(unsigned i=0;i<KK_UI_KEY_COUNT;++i) {
        uint8_t bit=(uint8_t)(1U<<i);
        bool lateral=i==KK_UI_INPUT_LEFT || i==KK_UI_INPUT_RIGHT;
        /* Cancel an L/R gesture on any other raw key, including chords
         * occurring entirely while foreground UI is unavailable. */
        if (lateral && (raw & (uint8_t)~bit)) s_input.suppressed |= bit;
        if (s_input.suppressed&bit) continue;
        if (lateral && (release&bit)) {
            /* A delayed timer may first observe release after 600ms. Keep
             * long/short classification, even if no LONG tick was serviced. */
            if (!(long_sent&bit) && now-s_times[i].pressed_at>=600U)
                queue_key(i,KK_UI_INPUT_LONG,now);
            queue_key(i,KK_UI_INPUT_RELEASE,now);
        }
        if (!(stable & raw & armed & bit)) continue;
        bool direction=i!=KK_UI_INPUT_OK && i!=KK_UI_INPUT_BACK;
        if (press&bit) {
            if ((press&KK_UI_KEY_BACK) ? i==KK_UI_INPUT_BACK :
                (press&KK_UI_KEY_OK) ? i==KK_UI_INPUT_OK : directions==bit)
                queue_key(i,KK_UI_INPUT_PRESS,now);
        }
        if (direction && directions==bit) {
            uint32_t held=now-s_times[i].pressed_at;
            if (lateral && !(long_sent&bit) && held>=600U) {
                queue_key(i,KK_UI_INPUT_LONG,now);long_sent |= bit;
            }
            if ((int16_t)((uint16_t)now-s_times[i].repeat_at)>=0) {
                queue_key(i,KK_UI_INPUT_REPEAT,now);
                s_times[i].repeat_at=(uint16_t)(now+(held>=1500U ? 60U : held>=800U ? 100U : 160U));
            }
        }
    }
    s_input.long_sent=long_sent;
    s_input.suppressed &= raw | stable;
}

__attribute__((noinline,noclone,optimize("Oz"))) void ui_input_complete(UiInputResult result)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if (result==UI_INPUT_ACCEPT) {
        if (s_read_valid && s_input.count && s_input.head==s_read_head) {
            ++s_input.head;--s_input.count;
        }
    } else {
        unsigned kept=0;
        if (result==UI_INPUT_DISPLAY_ERROR) {
            unsigned count=s_input.count;
            for(unsigned i=0;i<count;++i) {
                uint8_t event=s_input.events[(s_input.head+i)&15U];
                unsigned key=event&7U;
                if (key!=KK_UI_INPUT_OK+1U && key!=KK_UI_INPUT_BACK+1U)
                    s_input.events[(s_input.head+kept++)&15U]=event;
            }
        }
        s_input.count=(uint8_t)kept;
        s_input.suppressed |= (s_input.raw|s_input.stable)&(KK_UI_KEY_OK|KK_UI_KEY_BACK);
    }
    s_read_valid=0U;
    __set_PRIMASK(mask);
}
bool ui_input_context(uint16_t context)
{
    if (context==s_input.context) return false;
    s_input.context=context;ui_input_complete(UI_INPUT_CANCEL);
    return true;
}
void KK_UI_InputAccepted(void) { ui_input_complete(UI_INPUT_ACCEPT); }

__attribute__((noinline,noclone,optimize("Oz"))) KK_UI_Input ui_input_poll(void)
{
    uint32_t now=HAL_GetTick();
    uint8_t navigation=App_NavigationEvent(now);
    bool transition=KK_UI_IsTransitioning();
    uint32_t mask=__get_PRIMASK();__disable_irq();
    KK_UI_Input input={.keys=s_input.raw | s_input.stable};
    s_read_valid=0U;
    if (transition) {
        s_input.head += s_input.count;
        s_input.count=0U;
        for(unsigned i=0;i<KK_UI_KEY_COUNT;++i) {
            s_times[i].pressed_at=now;
            s_times[i].repeat_at=(uint16_t)(now+KK_UI_KEY_REPEAT_DELAY_MS);
        }
    } else if (navigation) input.key_event=navigation;
    else if (s_input.count) {
        input.key_event=s_input.events[s_input.head&15U];
        s_read_head=s_input.head;s_read_valid=1U;
    }
    __set_PRIMASK(mask);
    return input;
}
