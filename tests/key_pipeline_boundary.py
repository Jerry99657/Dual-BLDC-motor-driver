"""Limit the R129 port exception to key decoding and its one main-loop call."""
import hashlib,re

def verify(root,out,baseline,changed):
    allowed={'Core/Src/main.c','mycode/kk_ui/include/kk_ui.h',
        'mycode/kk_ui/src/kk_ui.c','mycode/kk_ui/src/kk_ui_internal.h','mycode/kk_ui/src/kk_ui_nav.c'}
    assert set(changed)==allowed,changed
    for name in allowed:
        assert hashlib.sha256((out/name).read_bytes()).hexdigest()==baseline['protected'][name],name
        before=(out/name).read_text(encoding='utf-8');after=(root/name).read_text(encoding='utf-8')
        expected=before
        if name.endswith('main.c'):
            expected=expected.replace('''        uint8_t ui_keys = ui_input_poll();
        KK_UI_Status ui_status = KK_UI_Update(
            now, (KK_UI_Input){.keys = ui_keys});''','''        KK_UI_Status ui_status = KK_UI_Update(now, ui_input_poll());''')
        elif name.endswith('kk_ui_internal.h'):
            expected=re.sub(r'typedef struct \{\n    uint32_t changed_at;.*?\} KK_UI_KeyState;\n\n','',expected,flags=re.S)
            expected=expected.replace('    KK_UI_KeyState keys[KK_UI_KEY_COUNT];\n','')
            expected=expected.replace('    uint8_t blocked_keys;','    uint8_t blocked_keys;\n    uint8_t active_keys;')
        elif name.endswith('kk_ui_nav.c'):
            start=expected.index('    for (uint8_t i = 0U; i < KK_UI_KEY_COUNT; ++i) {')
            end=expected.index('#if KK_UI_ENABLE_CUSTOM',start)
            expected=expected[:start]+'    kk_ui.blocked_keys |= kk_ui.active_keys;\n'+expected[end:]
        elif name.endswith('/kk_ui.h'):
            expected=expected.replace('    uint8_t keys;','''    uint8_t keys; /* Current active levels, for held-key/transition guards. */
    uint8_t key_event; /* Zero, or KK_UI_KEY_EVENT(action, source). */''')
            expected=expected.replace('    KK_UI_INPUT_ENCODER\n','    KK_UI_INPUT_ENCODER,\n    KK_UI_INPUT_RELEASE,\n    KK_UI_INPUT_LONG\n')
            addition='''
/* Input hardware owns debounce/repeat. One normalized event per update;
 * key_event occupies the former padding byte, preserving sizeof(Input). */
#define KK_UI_KEY_EVENT(action,source) ((uint8_t)(1U+(unsigned)(action)+8U*(unsigned)(source)))
'''
            expected=expected.replace('} KK_UI_InputSource;\n','} KK_UI_InputSource;\n'+addition)
            expected=expected.replace('KK_UI_Status KK_UI_Update(uint32_t now_ms, KK_UI_Input input);',
                '''KK_UI_Status KK_UI_Update(uint32_t now_ms, KK_UI_Input input);
/* Input port acknowledges a queued frame here, after transfer checks and
 * before routing it. Rendering failure after routing cannot replay a key. */
void KK_UI_InputAccepted(void);''')
            expected=expected.replace('KK_UI_PageId KK_UI_GetCurrentPage(void);','KK_UI_PageId KK_UI_GetCurrentPage(void);\nbool KK_UI_IsTransitioning(void);')
        else:
            start=expected.index('static uint32_t kk_ui_repeat_interval(')
            end=expected.index('static KK_UI_Status kk_ui_latch_display',start)
            port_start=after.index('/* Hardware input has already been sampled/debounced.')
            port_end=after.index('static KK_UI_Status kk_ui_latch_display',port_start)
            port=after[port_start:port_end]
            assert not re.search(r'\b(?:HAL_|LL_|Motor|GPIO|OLED_|TIM|ADC)',port),'Hardware/business in UI key router'
            expected=expected[:start]+port+expected[end:]
            expected=expected.replace('''        kk_ui.next_frame = now;
    }''','''        kk_ui.next_frame = now;
        kk_ui.keys_initialized = 1U;
        kk_ui.blocked_keys |= input.keys & KK_UI_KEY_MASK;
    }''',1)
            expected=expected.replace('''
    KK_UI_AnimateNavigation(now);''','''    KK_UI_InputAccepted();

    KK_UI_AnimateNavigation(now);''',1)
            expected=expected.replace('KK_UI_PageId KK_UI_GetCurrentPage(void)',
                'bool KK_UI_IsTransitioning(void) { return kk_ui.transition.active!=0U; }\n\nKK_UI_PageId KK_UI_GetCurrentPage(void)')
            start=expected.index('    for (uint8_t i = 0U; i < KK_UI_KEY_COUNT; ++i) {',expected.index('KK_UI_Status KK_UI_RecoverDisplay'))
            end=expected.index('    return KK_UI_OK;',start)
            expected=expected[:start]+'    kk_ui.blocked_keys |= kk_ui.active_keys & (KK_UI_KEY_OK|KK_UI_KEY_BACK);\n'+expected[end:]
        assert expected==after,'Unexpected non-input change: '+name
