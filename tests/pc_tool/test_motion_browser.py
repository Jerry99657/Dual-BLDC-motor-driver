"""Presentation motion with live samples and simulated commands. No hardware I/O."""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/console-motion'
OUT.mkdir(exist_ok=True)

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        context=browser.new_context(viewport={'width':1440,'height':1100},reduced_motion='no-preference')
        page=context.new_page();errors=[];requests=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('request',lambda r:requests.append(r.url))
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        page.wait_for_function("document.documentElement.classList.contains('motion-ready')")
        page.wait_for_timeout(900)
        page.hover('#demo');page.wait_for_timeout(100)
        assert page.locator('#demo').evaluate("e=>getComputedStyle(e,'::before').transform")!='matrix(0, 0, 0, 1, 0, 0)'
        page.click('#plotGuideToggle')
        assert page.locator('#plotGuideText').is_visible()
        assert page.locator('#plotGuideText').evaluate("e=>e.getAnimations().some(a=>a.id==='ui-disclosure')")
        page.click('#plotGuideToggle')
        assert page.locator('#plotGuideText').is_hidden(), 'Logical close must be immediate'
        page.wait_for_function("document.querySelector('#uiMotionLayer .ui-motion-ghost')!==null")
        page.wait_for_timeout(260)
        assert page.locator('#uiMotionLayer .ui-motion-ghost').count()==0

        page.locator('#runPanel summary').click()
        assert page.locator('#runPanel details').evaluate('e=>e.open')
        page.wait_for_function("document.querySelector('#runPanel details .row').getAnimations().some(a=>a.id==='ui-disclosure')")
        page.locator('#runPanel summary').click()
        assert not page.locator('#runPanel details').evaluate('e=>e.open')

        # A disconnected theme change can reveal the whole UI; unsupported APIs
        # still have a direct/floor fallback and persist the user's choice.
        page.evaluate('scrollTo(0,0)')
        page.evaluate("()=>{window.idleStops=0;window.send=command=>{if(command==='Stop')idleStops++;return Promise.resolve({result:1})}}")
        page.click('#themeToggle')
        page.wait_for_function("document.documentElement.dataset.theme==='dark'")
        page.click('#stop')
        assert page.evaluate('idleStops')==1
        page.wait_for_timeout(700)
        assert page.evaluate("localStorage.getItem('foc-console-theme')")=='dark'
        page.evaluate("()=>{for(let i=0;i<4;i++)document.getElementById('themeToggle').click()}")
        page.wait_for_function("document.documentElement.dataset.theme==='dark'")
        page.wait_for_timeout(700)
        assert page.locator('html').get_attribute('data-theme-reveal') is None
        page.locator('.nav-item[href="#workPanel"]').click()
        page.wait_for_function("document.querySelector('.nav-item[href=\"#workPanel\"]').getAttribute('aria-current')==='location'")
        assert page.locator('.ui-nav-marker').count()==1
        page.wait_for_timeout(1000)

        # Add/remove is immediate in the application; exiting images are inert,
        # ID-free ghosts and never duplicate the actual Canvas controls.
        page.evaluate("document.getElementById('plotPanel').scrollIntoView()")
        page.click('#plotAdd')
        assert page.evaluate('visiblePlots')==2
        assert page.locator('#plot-2').is_visible()
        page.wait_for_timeout(350)
        page.click('#plotRemove')
        assert page.evaluate('visiblePlots')==1
        assert page.locator('#plot-2').is_hidden()
        ids=page.locator('[id]').evaluate_all('els=>els.map(e=>e.id)')
        assert len(ids)==len(set(ids))
        assert page.locator('#uiMotionLayer [id]').count()==0
        page.wait_for_timeout(260)
        assert page.locator('#uiMotionLayer .ui-motion-ghost').count()==0

        page.evaluate("""()=>{
            window.motionWrites=[];window.motionSample=0;window.motionPaints=0;
            const nativeDraw=plots[0].draw;plots[0].draw=function(){motionPaints++;return nativeDraw()};
            window.motionFeed=()=>{const ms=motionSample++*10;add({kind:1,epoch:0,ms,values:[['motion.speed','rad/s',Math.sin(ms/100)]]})};
            for(let i=0;i<1500;i++)motionFeed();addAllPlotChannels();draw();window.motionTimer=setInterval(motionFeed,10);
            window.send=command=>{motionWrites.push(command);if(command==='Stop')return Promise.resolve({result:1});return new Promise(resolve=>{window.resolveMotion=resolve})};
        }""")
        # Open native details without replacing the browser's confirmation APIs.
        page.locator('#runPanel summary').click();page.fill('#command','Get')
        page.click('#send')
        assert page.evaluate('motionWrites')==['Get']
        assert page.locator('#send').get_attribute('aria-busy')=='true'
        before=page.evaluate('({samples:motionSample,paints:motionPaints})')
        page.click('#stop')
        assert page.evaluate('motionWrites')==['Get','Stop'], 'Stop must run during an outstanding action'
        assert not page.locator('#stop').is_disabled()
        assert page.locator('#stop').get_attribute('aria-busy') is None
        assert page.locator('#send').get_attribute('aria-busy')=='true'
        assert page.locator('#send').get_attribute('data-ui-complete') is None
        page.evaluate('resolveMotion({result:1})')
        page.wait_for_function("document.getElementById('send').getAttribute('aria-busy')===null")
        page.wait_for_function("document.getElementById('send').hasAttribute('data-ui-complete')")
        page.wait_for_function("document.querySelector('#uiToasts .ui-toast')!==null")
        assert page.evaluate('motionSample')>before['samples']
        assert page.evaluate('motionPaints')>before['paints']

        # Theme motion with telemetry never captures the document or covers the
        # actual plots. The same samples, cache setting and target draft survive.
        page.evaluate("document.getElementById('target').value='23';window.motionCache=plots[0].view.cacheSeconds;window.motionSelected=[...plots[0].selected];scrollTo(0,0)")
        before=page.evaluate('motionPaints')
        page.click('#themeToggle')
        page.wait_for_function("document.documentElement.dataset.theme==='light'")
        assert page.locator('.ui-theme-floor').count()==2
        assert page.locator('html').get_attribute('data-theme-reveal') is None
        page.wait_for_timeout(250)
        assert page.evaluate('motionPaints')>before
        assert page.evaluate("plots[0].view.cacheSeconds===motionCache&&JSON.stringify([...plots[0].selected])===JSON.stringify(motionSelected)&&document.getElementById('target').value==='23'")
        assert page.evaluate("document.getAnimations().every(a=>a.effect?.target?.tagName!=='CANVAS'&&!a.effect?.target?.closest?.('.hero-readouts,.plot-latest'))")
        page.wait_for_timeout(450)
        assert page.locator('.ui-theme-floor').count()==0

        page.evaluate("()=>{window.send=command=>{motionWrites.push(command);return Promise.reject(Error('测试拒绝：状态不允许'))}}")
        if not page.locator('#runPanel details').evaluate('e=>e.open'):
            page.locator('#runPanel summary').click()
        page.fill('#command','Get');page.click('#send')
        page.wait_for_function("document.getElementById('error').textContent.includes('测试拒绝')")
        page.wait_for_function("[...document.querySelectorAll('.ui-toast[data-kind=error]')].some(e=>e.textContent.includes('测试拒绝'))")
        assert page.locator('#send').get_attribute('aria-busy') is None

        # Turning reduced motion on also ends animations already in flight.
        page.click('#plotGuideToggle')
        page.emulate_media(reduced_motion='reduce')
        page.wait_for_timeout(50)
        assert page.evaluate("document.getAnimations().filter(a=>a.playState==='running').length") == 0
        assert page.locator('#uiMotionLayer .ui-motion-ghost,.ui-theme-floor').count()==0
        assert page.locator('#demo').evaluate("e=>getComputedStyle(e).transitionDuration")=='0s'
        page.click('#themeToggle')
        assert page.locator('html').get_attribute('data-theme')=='dark'
        page.click('#plotGuideToggle');assert page.locator('#plotGuideText').is_hidden()
        page.evaluate('clearInterval(motionTimer)')
        page.screenshot(path=str(OUT/'motion-feedback.png'))
        assert not errors,errors
        assert not [url for url in requests if url.startswith(('https:','http:'))]
        context.close()

        # Touch navigation shares the marker; all overlays leave global Stop free.
        context=browser.new_context(viewport={'width':390,'height':844},is_mobile=True,has_touch=True,reduced_motion='reduce')
        page=context.new_page();page.goto((ROOT/'pc_tool/index.html').as_uri())
        page.locator('.nav-item[href="#settingsPanel"]').tap()
        assert page.locator('.nav-item[href="#settingsPanel"]').get_attribute('aria-current')=='location'
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        assert page.locator('#stop').bounding_box()['y']>=0
        assert page.locator('.ui-nav-marker').bounding_box()['width']>60
        context.close()
        print('PASS: motion: button wipe, guide/details entry/exit, moving navigation marker, plot entry/exit with ID-free inert ghosts, truthful busy/error/completion feedback, immediate Stop, live 100 Hz samples/paints during theme motion, retained cache/drafts, live values unanimated, reduced-motion cancellation, touch layout and zero external requests')
    finally:
        browser.close()
