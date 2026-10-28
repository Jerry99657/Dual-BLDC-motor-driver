"""Async confirmations, truthful empty states, motion preference and touch targets.

Runs the actual offline HTML in Edge. Commands and board replies are simulated.
"""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/component-details'
OUT.mkdir(exist_ok=True)
fixture=list((ROOT/'build/console-r87/rpc-fixture.bin').read_bytes()[:106])

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        context=browser.new_context(viewport={'width':1440,'height':1100},reduced_motion='no-preference')
        page=context.new_page();errors=[];network=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('request',lambda r:network.append(r.url))
        page.goto((ROOT/'pc_tool/index.html').as_uri());page.wait_for_timeout(900)
        assert page.locator('#sourceEmpty').get_attribute('data-state')=='disconnected'
        assert page.locator('#statusEmpty').get_attribute('data-state')=='disconnected'
        assert page.locator('#status').is_hidden()
        assert page.evaluate('plots[0].selected.size===0&&plots[0].layout.panels[0].items.length===0')
        assert page.locator('#plotChannels .ui-empty-channel').is_visible()
        page.screenshot(path=str(OUT/'empty-light.png'))

        page.evaluate("""bytes=>{
            window.componentConfig=decodeReply(new WireParser().feed(new Uint8Array(bytes))[0]);
            componentConfig.work=0;componentConfig.drive=0;componentConfig.profiles=[0,0];
            componentConfig.settings.control_config&=~(7|8|16384|32768);
            window.componentWrites=[];
            window.send=async command=>{componentWrites.push(command);if(command==='Stop')batchEpoch++;log('ACK '+command);return {result:1}};
            window.confirm=()=>{throw Error('Native blocking confirm must not be used')};
            window.readyComponents=()=>{connected=true;connecting=false;boardConfig=componentConfig;populateControls(boardConfig);populateSettings();renderConfig()};
            readyComponents();
        }""",fixture)
        page.wait_for_function("document.getElementById('sourceEmpty').dataset.state==='waiting'")
        page.evaluate('boardConfig=null;connecting=true;connected=false')
        page.wait_for_function("document.getElementById('sourceEmpty').dataset.state==='connecting'")
        page.evaluate('connecting=false;connected=true;pending.set(65000,{command:"Get"})')
        page.wait_for_function("document.getElementById('sourceEmpty').dataset.state==='reading'")
        page.evaluate('pending.clear()')
        page.wait_for_function("document.getElementById('sourceEmpty').dataset.state==='unverified'")
        page.evaluate('connected=false;renderConfig()')
        page.click('#demo');page.wait_for_function('demo&&series.size===6')
        assert page.locator('#sourceEmpty').is_hidden()
        assert page.evaluate('plots[0].selected.size===0')
        assert page.locator('#statusEmpty').get_attribute('data-state')=='demo'
        page.click('#demo');page.evaluate('readyComponents()')

        # Progress uses measured source work, including after the palette change.
        page.evaluate(r"""async()=>{
            const file=new Blob(['host_time,board_ms,sequence,epoch,packet_kind,field,unit,value\n'+
                't,0,0,0,1,m1.feedback_speed,rad/s,5\n']);file.name='palette.csv';await loadCsv(file);
        }""")
        page.wait_for_function("document.getElementById('csvProgress').dataset.status==='done'")
        assert page.locator('#csvProgress i[data-filled=true]').count()==24
        assert page.locator('#csvProgress i').first.evaluate('e=>getComputedStyle(e).backgroundColor')=='rgb(255, 250, 0)'
        assert page.evaluate('plots[0].selected.size===0')
        page.click('#live');page.evaluate('readyComponents()')
        page.locator('#ledEnable').check()
        assert page.locator('#ledEnable').evaluate('e=>getComputedStyle(e).backgroundColor')=='rgb(255, 250, 0)'
        assert page.locator('#ledEnable').evaluate("e=>getComputedStyle(e,'::before').backgroundColor")=='rgb(32, 33, 30)'
        assert page.evaluate('componentWrites')==[]

        def open_align():
            page.click('#align')
            page.wait_for_function('document.getElementById("uiConfirm").open')
            assert page.locator('#confirmCancel').evaluate('e=>e===document.activeElement')
            assert page.locator('#align').get_attribute('aria-busy')=='true'

        def closed():
            page.wait_for_function('!document.getElementById("uiConfirm").open')
            page.wait_for_function('document.getElementById("align").getAttribute("aria-busy")===null')

        open_align();page.keyboard.press('Enter');closed()
        assert page.evaluate('componentWrites')==[], 'Enter on the default Cancel never executes a command'
        assert page.locator('#align').evaluate('e=>e===document.activeElement')
        open_align();page.keyboard.press('Escape');closed()
        open_align();page.mouse.click(10,600);closed()
        assert page.evaluate('componentWrites')==[]
        assert page.locator('#align').get_attribute('data-ui-complete') is None
        assert not page.locator('#uiToasts').get_by_text('对齐请求已受理',exact=True).count()

        # Native modal focus stays within the operation and its global Stop.
        open_align()
        for _ in range(9):
            page.keyboard.press('Tab')
            assert page.evaluate("document.getElementById('uiConfirm').contains(document.activeElement)")
        page.wait_for_timeout(320)
        page.screenshot(path=str(OUT/'confirmation-light.png'))
        page.click('#confirmAccept');closed()
        assert page.evaluate('componentWrites')==['Align:M1']

        page.evaluate("""()=>{
            componentWrites.length=0;window.componentSample=0;window.componentPaints=0;
            const original=plots[0].draw;plots[0].draw=function(){componentPaints++;return original()};
            window.componentTimer=setInterval(()=>add({kind:1,epoch:0,ms:componentSample++*10,values:[['m1.feedback_speed','rad/s',50]]}),10);
        }""")
        page.wait_for_function('series.size===1')
        page.locator('#signals [data-channel="m1.feedback_speed|rad/s"] input').check()
        open_align();before=page.evaluate('({samples:componentSample,paints:componentPaints})')
        page.wait_for_timeout(200)
        assert page.evaluate('componentSample')>before['samples']
        rect=page.locator('#plot').bounding_box()
        if rect['y']+rect['height']>=-64 and rect['y']<=1164:
            assert page.evaluate('componentPaints')>before['paints']
        else:
            assert page.evaluate('componentPaints')==before['paints'], 'Offscreen plots keep receiving without repainting'
        original_stop=page.locator('#stop').bounding_box();modal_stop=page.locator('#confirmStop').bounding_box()
        assert all(abs(original_stop[key]-modal_stop[key])<1 for key in ('x','y','width','height')),(original_stop,modal_stop)
        page.click('#confirmStop')
        assert page.evaluate('componentWrites')==['Stop'], 'Stop runs before the exit animation finishes'
        closed();page.wait_for_timeout(100)
        assert page.evaluate('componentWrites')==['Stop'], 'The abandoned alignment cannot execute after Stop'

        page.evaluate('componentWrites.length=0');open_align()
        page.evaluate("connected=false;document.getElementById('confirmAccept').click()")
        closed();assert page.evaluate('componentWrites')==[], 'An accept click cannot race a disconnect'
        page.evaluate('readyComponents()');open_align()
        page.evaluate('lastEpoch++;status({})');closed()
        assert page.evaluate('componentWrites')==[], 'Board restart cancels the pending operation'

        # Repeated animation retirement previously crashed this Edge build as
        # live data arrived. Exercise it with normal JIT and normal motion.
        for _ in range(10):
            open_align();page.wait_for_timeout(60);page.click('#confirmCancel');closed()
        page.wait_for_timeout(800)
        assert page.evaluate('componentWrites')==[]
        assert page.locator('#uiMotionLayer .ui-motion-ghost').count()==0

        # Superseding a confirmation must not let an older native close event
        # cancel the new question, or execute the older action.
        page.evaluate("()=>{window.firstQuestion=confirmAction('第一项');window.secondQuestion=confirmAction('第二项')}")
        assert page.evaluate('firstQuestion') is False
        page.wait_for_timeout(50);assert page.locator('#uiConfirmText').inner_text()=='第二项'
        page.click('#confirmAccept');assert page.evaluate('secondQuestion') is True
        page.wait_for_function('!document.getElementById("uiConfirm").open')
        page.evaluate('clearInterval(componentTimer)')

        # This preference is independent of board settings and survives reload.
        page.select_option('#motionMode','calm')
        assert page.locator('html').get_attribute('data-ef-motion')=='calm'
        assert page.evaluate("localStorage.getItem('foc-console-motion')")=='calm'
        assert page.evaluate('componentWrites')==[]
        page.reload();page.wait_for_timeout(100)
        assert page.locator('html').get_attribute('data-ef-motion')=='calm'
        assert page.evaluate("document.getAnimations().every(a=>a.id!=='ui-page-enter')")
        assert page.evaluate('plots[0].selected.size===0'), 'Motion preference never restores curve bindings'
        page.click('#themeToggle')
        assert page.locator('html').get_attribute('data-theme')=='dark'
        assert page.locator('.ui-theme-floor').count()==0
        assert page.locator('html').get_attribute('data-theme-reveal') is None
        page.click('#plotGuideToggle')
        assert page.locator('#plotGuideText').evaluate("e=>e.getAnimations().some(a=>a.id==='ui-disclosure')"), 'Calm retains operation responses'
        page.emulate_media(reduced_motion='reduce')
        page.wait_for_timeout(50)
        assert page.evaluate("document.getAnimations().filter(a=>a.playState==='running').length")==0
        assert '系统已开启减少动态效果' in page.locator('#motionNote').inner_text()
        page.select_option('#motionMode','full')
        page.click('#align');page.wait_for_function('document.getElementById("uiConfirm").open')
        assert page.locator('#uiConfirm').evaluate("e=>getComputedStyle(e,'::backdrop').animationName")=='none'
        assert page.evaluate("document.getAnimations().filter(a=>a.playState==='running').length")==0
        page.screenshot(path=str(OUT/'confirmation-dark.png'))
        page.click('#confirmCancel');page.wait_for_function('!document.getElementById("uiConfirm").open')
        assert not errors,errors
        assert not [url for url in network if url.startswith(('http:','https:'))]
        context.close()

        context=browser.new_context(viewport={'width':375,'height':900},is_mobile=True,has_touch=True,device_scale_factor=2,reduced_motion='reduce')
        page=context.new_page();touch_errors=[];page.on('pageerror',lambda e:touch_errors.append(str(e)))
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        page.evaluate("""()=>{
            document.getElementById('window').value='100';document.getElementById('window').onchange();
            for(let i=0;i<=10000;i++)add({kind:1,epoch:0,ms:i*10,values:[['touch.speed','rad/s',50+Math.sin(i)]]});
            assignPlotChannel(0,'touch.speed|rad/s',true);plotView.span=.05;plotView.offset=-.5;draw();
        }""")
        page.locator('#plotBuffer').scroll_into_view_if_needed()
        for suffix in ('Start','End','Move'):
            element=page.locator('#plotBuffer'+suffix);rect=element.bounding_box()
            assert rect['width']>=44 and rect['height']>=44,rect
            assert element.evaluate("e=>{const r=e.getBoundingClientRect();return document.elementFromPoint(r.x+r.width/2,r.y+r.height/2)===e}"),suffix
            assert element.evaluate("e=>getComputedStyle(e,'::after').width")=='12px', 'Only the hit target grows'
        start_rect=page.locator('#plotBufferStart').bounding_box()
        before=page.evaluate('({end:plotView.offset,span:plotView.span})')
        x=start_rect['x']+start_rect['width']/2;y=start_rect['y']+start_rect['height']/2
        cdp=context.new_cdp_session(page)
        cdp.send('Input.dispatchTouchEvent',{'type':'touchStart','touchPoints':[{'x':x,'y':y}]})
        cdp.send('Input.dispatchTouchEvent',{'type':'touchMove','touchPoints':[{'x':x-12,'y':y}]})
        cdp.send('Input.dispatchTouchEvent',{'type':'touchEnd','touchPoints':[]})
        assert abs(page.evaluate('plotView.offset')-before['end'])<1e-8
        assert page.evaluate('plotView.span')>before['span'], 'A touch on the enlarged left handle stretches only the left boundary'
        remove=page.locator('#plotChannels .plot-channel button');rect=remove.bounding_box()
        assert rect['width']>=44 and rect['height']>=44,rect
        remove.tap();assert page.evaluate('plots[0].selected.size')==0
        page.click('#align');page.wait_for_function('document.getElementById("uiConfirm").open')
        rect=page.locator('#confirmStop').bounding_box()
        assert rect['width']>=44 and rect['height']>=44 and rect['x']>=0 and rect['y']>=0
        assert rect['x']+rect['width']<=375
        page.screenshot(path=str(OUT/'touch-confirmation.png'))
        page.click('#confirmCancel');page.wait_for_function('!document.getElementById("uiConfirm").open')
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        assert not touch_errors,touch_errors
        context.close()
        print('PASS: component details: truthful source/status/empty canvas states, yellow switch and real progress, async modal cancellation/acceptance/focus/Stop and stale-context guards, repeated animation retirement with normal JIT and live data, overlapping confirmations, full/calm persistence and reduced-motion priority, independent 44px touch targets and native touch resize, themes, layouts and zero network requests')
    finally:
        browser.close()
