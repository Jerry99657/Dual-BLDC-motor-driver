#include "kk_ui_app.h"

#include "kk_home_icons.h"
#include "kk_oled.h"
#include "kk_ui_draw.h"
#include "kk_ui_font.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

enum {
    PAGE_HOME = 1,
    PAGE_SETTINGS,
    PAGE_MONITOR,
    PAGE_WAVE,
    PAGE_ABOUT,
    PAGE_COMPONENTS
};

enum {
    INT_BRIGHTNESS = 0,
    INT_INTERVAL
};

enum {
    BOOL_SWITCH = 0,
    BOOL_SHOW_ITEM,
    BOOL_DISABLE_ITEM
};

enum {
    CONFIRM_RESET = 0
};

static int32_t brightness = 80;
static int32_t animation_interval = 20;
static bool demo_switch = true;
static bool show_item = true;
static bool disable_item;
static uint8_t settings_states[2];
static uint8_t component_states[2];
static char uptime_text[16] = "0s";
static char brightness_text[12] = "80%";
static char switch_text[8] = "开";
static char interval_text[12] = "20ms";
static char brightness_state[12] = "等待";
static char screen_text[12];
static uint32_t app_started_at;
static uint32_t info_updated_at;
static uint32_t app_now;
static uint16_t wave_phase;
static uint16_t wave_progress;
static uint32_t wave_entered_at;
static uint32_t wave_fps_started;
static uint32_t wave_draw_count;
static uint32_t wave_draw_epoch;
static uint32_t app_update_epoch;
static char wave_fps_text[5] = "--";
static char wave_progress_text[16] = "自动 0%";
static bool wave_manual;
static int32_t requested_brightness;
static int32_t applied_brightness;
static bool brightness_pending;
static bool brightness_applied;
static bool brightness_unsupported;
static bool brightness_success_notification;
static const char *pending_toast;
static uint32_t pending_toast_duration;

static const KK_UI_HomeItem home_items[] = {
    {"设置", home_icons[0], PAGE_SETTINGS},
    {"数据监测", home_icons[1], PAGE_MONITOR},
    {"波形", home_icons[2], PAGE_WAVE},
    {"关于设备", home_icons[3], PAGE_ABOUT}
};

static const KK_UI_MenuItem settings_items[] = {
    {"屏幕亮度", KK_UI_MENU_INT, INT_BRIGHTNESS},
    {"开关", KK_UI_MENU_BOOL, BOOL_SWITCH},
    {"组件演示", KK_UI_MENU_PAGE, PAGE_COMPONENTS},
    {"确认框", KK_UI_MENU_CONFIRM, CONFIRM_RESET},
    {"消息框", KK_UI_MENU_ACTION, KK_UI_EVENT_SHOW_MESSAGE},
    {"Toast", KK_UI_MENU_ACTION, KK_UI_EVENT_SHOW_TOAST}
};

static const KK_UI_MenuItem component_items[] = {
    {"动画间隔", KK_UI_MENU_INT, INT_INTERVAL},
    {"显示条目", KK_UI_MENU_BOOL, BOOL_SHOW_ITEM},
    {"禁用条目", KK_UI_MENU_BOOL, BOOL_DISABLE_ITEM},
    {"操作示例", KK_UI_MENU_ACTION, KK_UI_EVENT_ADVANCED_ACTION}
};

static const KK_UI_InfoRow monitor_rows[] = {
    {"运行时间", uptime_text},
    {"屏幕亮度", brightness_text},
    {"开关状态", switch_text},
    {"动画间隔", interval_text},
    {"亮度控制", brightness_state}
};

#if KK_UI_REFRESH_MODE == KK_UI_REFRESH_BLOCKING
#define APP_REFRESH_TEXT "Blocking"
#elif KK_UI_REFRESH_MODE == KK_UI_REFRESH_IT
#define APP_REFRESH_TEXT "IT"
#else
#define APP_REFRESH_TEXT "DMA"
#endif

static const KK_UI_InfoRow about_rows[] = {
    {"设备名称", "KK_UI"},
    {"UI版本", "0.1.0"},
    {"显示驱动", "KK_OLED"},
    {"屏幕尺寸", screen_text},
    {"刷新方式", APP_REFRESH_TEXT},
    {"断电保留", "否"},
    {"方向键", "选择/调整"},
    {"确定键", "进入/确认"},
    {"信息波形", "确定退出"},
    {"帧率口径", "波形绘制"}
};

static const KK_UI_HomePage home_pages[] = {
    {home_items, (uint16_t)(sizeof(home_items) / sizeof(home_items[0]))}
};

static const KK_UI_MenuPage menu_pages[] = {
    {"设置", settings_items, settings_states,
     (uint16_t)(sizeof(settings_items) / sizeof(settings_items[0]))},
    {"组件演示", component_items, component_states,
     (uint16_t)(sizeof(component_items) / sizeof(component_items[0]))}
};

static const KK_UI_InfoPage info_pages[] = {
    {"数据监测", monitor_rows,
     (uint16_t)(sizeof(monitor_rows) / sizeof(monitor_rows[0]))},
    {"关于设备", about_rows,
     (uint16_t)(sizeof(about_rows) / sizeof(about_rows[0]))}
};

static const KK_UI_IntBinding int_bindings[] = {
    {"屏幕亮度", &brightness, 10, 100, 1U, "%",
     KK_UI_EVENT_BRIGHTNESS_CHANGED},
    {"动画间隔", &animation_interval, 10, 100, 10U, "ms",
     KK_UI_EVENT_INTERVAL_CHANGED}
};

static const KK_UI_BoolBinding bool_bindings[] = {
    {"开关", &demo_switch, KK_UI_EVENT_SWITCH_CHANGED},
    {"显示条目", &show_item,
     KK_UI_EVENT_ADVANCED_VISIBILITY_CHANGED},
    {"禁用条目", &disable_item, KK_UI_EVENT_DEMO_LOCK_CHANGED}
};

static const KK_UI_ConfirmDesc confirm_descs[] = {
    {"恢复默认设置？\n包含屏幕亮度", KK_UI_EVENT_RESET_CONFIRMED,
     KK_UI_EVENT_RESET_CANCELLED}
};

static const KK_UI_PageRoute routes[] = {
    {KK_UI_PAGE_HOME, 0U},
    {KK_UI_PAGE_MENU, 0U},
    {KK_UI_PAGE_INFO, 0U},
    {KK_UI_PAGE_CUSTOM, 0U},
    {KK_UI_PAGE_INFO, 1U},
    {KK_UI_PAGE_MENU, 1U}
};

static const KK_UI_App app = {
    .root_page = PAGE_HOME,
    .routes = routes,
    .route_count = (uint16_t)(sizeof(routes) / sizeof(routes[0])),
    .home_pages = home_pages,
    .home_page_count = (uint16_t)(sizeof(home_pages) / sizeof(home_pages[0])),
    .menu_pages = menu_pages,
    .menu_page_count = (uint16_t)(sizeof(menu_pages) / sizeof(menu_pages[0])),
    .info_pages = info_pages,
    .info_page_count = (uint16_t)(sizeof(info_pages) / sizeof(info_pages[0])),
    .custom_page_count = 1U,
    .int_bindings = int_bindings,
    .int_binding_count = (uint16_t)(sizeof(int_bindings) / sizeof(int_bindings[0])),
    .bool_bindings = bool_bindings,
    .bool_binding_count = (uint16_t)(sizeof(bool_bindings) / sizeof(bool_bindings[0])),
    .confirm_descs = confirm_descs,
    .confirm_desc_count = (uint16_t)(sizeof(confirm_descs) / sizeof(confirm_descs[0])),
    .fonts = {kk_font_ui_home, kk_font_ui_title, kk_font_ui_body},
    .texts = {"返回", "取消", "确定", "开", "关", "这是消息框"}
};

static void app_format_unsigned(uint32_t value, const char *suffix,
                                char *buffer, uint16_t capacity)
{
    char reverse[10];
    uint16_t count = 0U;
    uint16_t out = 0U;
    uint16_t i = 0U;
    do {
        reverse[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U && count < sizeof(reverse));
    while (count != 0U && out + 1U < capacity) {
        buffer[out++] = reverse[--count];
    }
    while (suffix[i] != '\0' && out + 1U < capacity) {
        buffer[out++] = suffix[i++];
    }
    buffer[out] = '\0';
}

static void app_update_switch_text(void)
{
    strcpy(switch_text, demo_switch ? "开" : "关");
}

static void app_set_brightness_state(const char *state)
{
    if (strcmp(brightness_state, state) != 0) {
        strcpy(brightness_state, state);
        KK_UI_Invalidate();
    }
}

static void app_update_progress_text(void)
{
    strcpy(wave_progress_text, wave_manual ? "手动 " : "自动 ");
    app_format_unsigned(wave_progress, "%",
                        wave_progress_text + strlen(wave_progress_text),
                        (uint16_t)(sizeof(wave_progress_text) -
                                   strlen(wave_progress_text)));
}

const KK_UI_App *KK_UI_AppGet(void)
{
    return &app;
}

static void app_reset_parameters(void)
{
    brightness = 80;
    animation_interval = 20;
    demo_switch = true;
    show_item = true;
    disable_item = false;
    app_format_unsigned((uint32_t)brightness, "%", brightness_text,
                        sizeof(brightness_text));
    app_update_switch_text();
    app_format_unsigned((uint32_t)animation_interval, "ms", interval_text,
                        sizeof(interval_text));
}

void KK_UI_AppReset(uint32_t now_ms)
{
    uint16_t used;
    app_reset_parameters();
    memset(settings_states, 0, sizeof(settings_states));
    memset(component_states, 0, sizeof(component_states));
    memcpy(uptime_text, "0s", sizeof("0s"));
    app_format_unsigned(OLED_GetWidth(), "x", screen_text, sizeof(screen_text));
    used = (uint16_t)strlen(screen_text);
    app_format_unsigned(OLED_GetHeight(), "", screen_text + used,
                        (uint16_t)(sizeof(screen_text) - used));
    app_started_at = now_ms;
    info_updated_at = now_ms;
    app_now = now_ms;
    app_update_epoch = 0U;
    wave_draw_epoch = UINT32_MAX;
    wave_phase = 0U;
    wave_progress = 0U;
    wave_entered_at = now_ms;
    wave_fps_started = now_ms;
    wave_draw_count = 0U;
    strcpy(wave_fps_text, "--");
    wave_manual = false;
    app_update_progress_text();
    requested_brightness = brightness;
    brightness_applied = false;
    brightness_pending = true;
    brightness_unsupported = false;
    brightness_success_notification = false;
    strcpy(brightness_state, "等待");
    pending_toast = NULL;
    pending_toast_duration = 0U;
}

static void app_apply_component_state(void)
{
    KK_UI_ItemState state = !show_item ? KK_UI_ITEM_HIDDEN :
                           disable_item ? KK_UI_ITEM_DISABLED : KK_UI_ITEM_NORMAL;
    (void)KK_UI_SetMenuItemState(PAGE_COMPONENTS, 3U, state);
}

static void app_show_toast_when_ready(const char *text, uint32_t duration)
{
    KK_UI_Status status = KK_UI_ShowToast(text, duration);
    if (status == KK_UI_BUSY) {
        pending_toast = text;
        pending_toast_duration = duration;
    } else {
        pending_toast = NULL;
        pending_toast_duration = 0U;
    }
}

static void app_request_brightness(int32_t value, bool force)
{
    if (brightness_unsupported) return;
    if (force || value != requested_brightness) {
        requested_brightness = value;
        brightness_pending = true;
        app_set_brightness_state("等待");
    }
    if (brightness_applied && applied_brightness == value &&
        brightness_pending) {
        brightness_pending = false;
        app_set_brightness_state("正常");
    }
}

static void app_service_brightness(void)
{
    int32_t draft;
    int32_t desired = brightness;
    OLED_Status status;

    if (KK_UI_GetIntEditorDraft(INT_BRIGHTNESS, &draft)) desired = draft;
    app_request_brightness(desired, false);
    if (brightness_pending && KK_UI_IsDisplayIdle()) {
        uint8_t contrast = (uint8_t)((requested_brightness * 255 + 50) / 100);
        status = OLED_SetContrast(contrast);
        if (status == OLED_OK) {
            applied_brightness = requested_brightness;
            brightness_applied = true;
            brightness_pending = false;
            app_set_brightness_state("正常");
        } else if (status == OLED_ERROR) {
            brightness_pending = false;
            brightness_success_notification = false;
            app_set_brightness_state("失败");
            app_show_toast_when_ready("亮度设置失败", 0U);
        } else if (status == OLED_UNSUPPORTED) {
            brightness_pending = false;
            brightness_unsupported = true;
            brightness_success_notification = false;
            app_set_brightness_state("不支持");
            (void)KK_UI_SetMenuItemState(PAGE_SETTINGS, 0U,
                                         KK_UI_ITEM_DISABLED);
            app_show_toast_when_ready("亮度不支持", 0U);
        }
    }
    if (brightness_success_notification && !brightness_pending &&
        brightness_applied && applied_brightness == brightness &&
        !brightness_unsupported) {
        brightness_success_notification = false;
        app_show_toast_when_ready("设置已更新", 0U);
    }
}

void KK_UI_AppProcessEvents(void)
{
    KK_UI_EventId event;
    if (pending_toast != NULL) {
        app_show_toast_when_ready(pending_toast, pending_toast_duration);
    }
    while (KK_UI_PollEvent(&event)) {
        switch (event) {
        case KK_UI_EVENT_RESET_CONFIRMED:
            app_reset_parameters();
            app_apply_component_state();
            KK_UI_Invalidate();
            if (brightness_unsupported) {
                app_show_toast_when_ready("亮度不支持", 0U);
            } else {
                brightness_success_notification = true;
                app_request_brightness(brightness, true);
            }
            break;
        case KK_UI_EVENT_SHOW_MESSAGE:
            (void)KK_UI_ShowMessage("确定关闭",
                                    KK_UI_EVENT_MESSAGE_ACKNOWLEDGED);
            break;
        case KK_UI_EVENT_SHOW_TOAST:
            app_show_toast_when_ready("这是 Toast 提示", 0U);
            break;
        case KK_UI_EVENT_BRIGHTNESS_CHANGED:
            app_format_unsigned((uint32_t)brightness, "%", brightness_text,
                                sizeof(brightness_text));
            KK_UI_Invalidate();
            if (brightness_unsupported) {
                brightness_success_notification = false;
                app_show_toast_when_ready("亮度不支持", 0U);
            } else {
                brightness_success_notification = true;
                app_request_brightness(brightness, true);
            }
            break;
        case KK_UI_EVENT_SWITCH_CHANGED:
            app_update_switch_text();
            KK_UI_Invalidate();
            app_show_toast_when_ready("设置已更新", 0U);
            break;
        case KK_UI_EVENT_INTERVAL_CHANGED:
            app_format_unsigned((uint32_t)animation_interval, "ms",
                                interval_text, sizeof(interval_text));
            KK_UI_Invalidate();
            app_show_toast_when_ready("设置已更新", 0U);
            break;
        case KK_UI_EVENT_ADVANCED_VISIBILITY_CHANGED:
        case KK_UI_EVENT_DEMO_LOCK_CHANGED:
            app_apply_component_state();
            app_show_toast_when_ready("设置已更新", 0U);
            break;
        case KK_UI_EVENT_ADVANCED_ACTION:
            app_show_toast_when_ready("操作已触发", 0U);
            break;
        default:
            break;
        }
    }
    app_service_brightness();
}

void KK_UI_AppUpdate(uint32_t now)
{
    app_now = now;
    ++app_update_epoch;
    if (now - info_updated_at >= 1000U) {
        info_updated_at = now;
        app_format_unsigned((now - app_started_at) / 1000U, "s", uptime_text,
                            sizeof(uptime_text));
        KK_UI_Invalidate();
    }
}

void KK_UI_CustomOnEnter(KK_UI_PageId page)
{
    (void)page;
    wave_phase = 0U;
    wave_progress = 0U;
    wave_manual = false;
    wave_entered_at = app_now;
    wave_fps_started = app_now;
    wave_draw_count = 0U;
    wave_draw_epoch = UINT32_MAX;
    strcpy(wave_fps_text, "--");
    app_update_progress_text();
}

void KK_UI_CustomOnLeave(KK_UI_PageId page)
{
    (void)page;
}

void KK_UI_CustomOnInput(KK_UI_PageId page, KK_UI_InputEvent event)
{
    (void)page;
    if (event.action == KK_UI_INPUT_OK) {
        (void)KK_UI_CustomRequestClose();
    } else if (event.action == KK_UI_INPUT_UP) {
        wave_manual = true;
        wave_progress = event.steps >= wave_progress ? 0U :
                        (uint16_t)(wave_progress - event.steps);
        app_update_progress_text();
        KK_UI_Invalidate();
    } else if (event.action == KK_UI_INPUT_DOWN) {
        uint32_t next = (uint32_t)wave_progress + event.steps;
        wave_manual = true;
        wave_progress = next > 100U ? 100U : (uint16_t)next;
        app_update_progress_text();
        KK_UI_Invalidate();
    }
}

bool KK_UI_CustomOnTick(KK_UI_PageId page, uint32_t now)
{
    uint16_t next = (uint16_t)(((now - wave_entered_at) /
                                (uint32_t)animation_interval) & 31U);
    bool changed = next != wave_phase;
    uint32_t elapsed = now - wave_fps_started;
    (void)page;
    wave_phase = next;
    if (!wave_manual) {
        uint16_t progress = (uint16_t)(((now - wave_entered_at) / 50U) % 101U);
        if (progress != wave_progress) {
            wave_progress = progress;
            app_update_progress_text();
            changed = true;
        }
    }
    if (elapsed >= 1000U) {
        uint32_t fps = (wave_draw_count * 1000U + elapsed / 2U) / elapsed;
        if (fps > 999U) fps = 999U;
        app_format_unsigned(fps, "", wave_fps_text, sizeof(wave_fps_text));
        wave_draw_count = 0U;
        wave_fps_started = now;
        changed = true;
    }
    return changed;
}

void KK_UI_CustomOnDraw(KK_UI_PageId page, int16_t x_offset,
                        int16_t clip_x, uint16_t clip_width)
{
    int16_t x;
    int16_t previous_y = 27;
    (void)page;
    if (wave_draw_epoch != app_update_epoch) {
        wave_draw_epoch = app_update_epoch;
        ++wave_draw_count;
    }
    OLED_SetClipWindow(clip_x, 0, clip_width, 64U);
    OLED_SetFont(kk_font_ui_title);
    OLED_DrawUTF8((int16_t)(4 + x_offset), 1, "波形");
    OLED_DrawUTF8((int16_t)(52 + x_offset), 1, "FPS:");
    OLED_DrawUTF8((int16_t)(78 + x_offset), 1, wave_fps_text);
    OLED_DrawLine((int16_t)(117 + x_offset), 5,
                  (int16_t)(123 + x_offset), 11);
    OLED_DrawLine((int16_t)(123 + x_offset), 5,
                  (int16_t)(117 + x_offset), 11);
    OLED_SetDrawMode(OLED_DRAW_XOR);
    OLED_DrawRBox((int16_t)(113 + x_offset), 1, 14U, 14U, 3U);
    OLED_SetDrawMode(OLED_DRAW_SET);
    for (x = 0; x < 128; ++x) {
        static const int8_t wave[16] = {
            0, -3, -7, -11, -7, -3, 0, 3, 7, 11, 7, 3, 0, -2, -1, 0
        };
        int16_t y = (int16_t)(27 + wave[(x + wave_phase) & 15] * 8 / 11);
        if (x != 0) {
            OLED_DrawLine((int16_t)(x - 1 + x_offset), previous_y,
                          (int16_t)(x + x_offset), y);
        }
        previous_y = y;
    }
    OLED_SetFont(kk_font_ui_body);
    OLED_DrawUTF8((int16_t)(10 + x_offset), 40, wave_progress_text);
    KK_UI_DrawProgressBar((int16_t)(10 + x_offset), 55, 108U, 8U,
                          wave_progress, 100U);
    OLED_ResetClipWindow();
}

#ifdef KK_OLED_TEST
uint16_t KK_UI_AppTestWaveProgress(void) { return wave_progress; }
bool KK_UI_AppTestWaveManual(void) { return wave_manual; }
uint16_t KK_UI_AppTestWavePhase(void) { return wave_phase; }
const char *KK_UI_AppTestWaveFps(void) { return wave_fps_text; }
uint32_t KK_UI_AppTestWaveEnteredAt(void) { return wave_entered_at; }
#endif
