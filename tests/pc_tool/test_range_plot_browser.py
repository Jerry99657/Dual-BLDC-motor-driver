"""The 4..7 s recording gestures: anchored endpoint resize + middle translation.

Actual Edge pointer/keyboard events while 100 Hz samples continue to arrive.
"""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/plot-range-r101';OUT.mkdir(exist_ok=True)

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        for dpr in (1,2):
            context=browser.new_context(viewport={'width':1540,'height':1400},device_scale_factor=dpr)
            page=context.new_page();errors=[]
            page.on('pageerror',lambda e:errors.append(str(e)))
            page.goto((ROOT/'pc_tool/index.html').as_uri())
            page.evaluate("""()=>{
                window.rangeCommands=[];window.send=async command=>rangeCommands.push(command);
                window.rangeSample=0;
                window.rangeFeed=n=>{for(let i=0;i<n;i++){const t=rangeSample++*.01;
                    add({kind:1,epoch:0,ms:t*1000,values:[['range.target','rad/s',50],['range.actual','rad/s',50+Math.sin(t*10)]]});
                }};
                rangeFeed(2200);plotView.y.set('__xy',{lo:47,hi:53});draw();
                window.rangeTimer=setInterval(()=>rangeFeed(1),10);
            }""")

            def position(selector):
                page.locator(selector).scroll_into_view_if_needed()
                rect=page.locator(selector).bounding_box()
                return rect['x']+rect['width']/2,rect['y']+rect['height']/2

            # Red endpoint changes only the start offset; newer samples still render.
            end=page.evaluate('plotView.offset');span=page.evaluate('plotView.span')
            x,y=position('#plotBufferStart');page.mouse.move(x,y);page.mouse.down();page.mouse.move(x-120,y,steps=8)
            assert page.evaluate('plotView.span')>span
            assert abs(page.evaluate('plotView.offset')-end)<1e-7
            held=page.evaluate('plotLayout.end');page.wait_for_function('t=>plotLayout.end>t+.12',arg=held)
            assert page.evaluate("plotView.drag?.kind==='buffer'&&plotView.frozen===null")
            page.mouse.up();assert not page.locator('#plotBuffer').get_attribute('data-drag')
            assert page.evaluate('plotView.y.get("__xy").lo===47&&plotView.y.get("__xy").hi===53')

            # Purple endpoint changes only the end offset, including during the hold.
            start=page.evaluate('plotView.offset-plotView.span');span=page.evaluate('plotView.span')
            x,y=position('#plotBufferEnd');page.mouse.move(x,y);page.mouse.down();page.mouse.move(x-80,y,steps=8)
            assert page.evaluate('plotView.span')<span
            assert abs(page.evaluate('plotView.offset-plotView.span')-start)<1e-7
            held=page.evaluate('plotLayout.end');page.wait_for_function('t=>plotLayout.end>t+.12',arg=held)
            assert abs(page.evaluate('plotView.offset-plotView.span')-start)<1e-7
            page.mouse.up()

            # Green midpoint translates both endpoints with an unchanged span.
            span=page.evaluate('plotView.span');offset=page.evaluate('plotView.offset')
            x,y=position('#plotBufferMove');page.mouse.move(x,y);page.mouse.down();page.mouse.move(x-45,y,steps=7);page.mouse.up()
            assert abs(page.evaluate('plotView.span')-span)<1e-7
            assert page.evaluate('plotView.offset')<offset
            assert page.evaluate('plotView.frozen===null&&!paused')
            page.locator('#plotBufferStart').press('ArrowRight')
            assert page.evaluate('plotView.span')<span
            if dpr==1:page.locator('#plotPanel').screenshot(path=str(OUT/'buffer-three-handles.png'))

            # Close endpoints must remain independently reachable at the 1 ms limit.
            page.click('#plotReset');x,y=position('#plotBufferStart')
            page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+1800,y,steps=10);page.mouse.up()
            assert page.evaluate('plotView.span>=.001&&plotView.span<.002&&Number.isFinite(plotView.offset)')
            assert page.locator('#plotBufferEnd').get_attribute('role')=='slider'

            # The preset supports arbitrary fractions/long spans; it stays distinct
            # from temporary gesture zoom and is restored by Reset/Home.
            for value in ('0.2','2.5','30.125','86400'):
                page.fill('#window',value);page.locator('#window').press('Enter')
                assert page.evaluate('plotView.preset')==float(value)
                assert page.evaluate('plotView.span')==float(value)
                assert page.evaluate('Number.isFinite(plotLayout.end)&&plotView.frozen===null')
            for value in ('','0','-1','86401'):
                page.fill('#window',value);page.locator('#window').press('Enter')
                assert page.locator('#window').get_attribute('aria-invalid')=='true'
                assert page.locator('#plotPresetError').is_visible()
                assert page.evaluate('plotView.preset===86400&&plotView.span===86400')
            page.click('#plotReset')
            assert page.locator('#window').input_value()=='86400'
            assert page.locator('#plotPresetError').is_hidden()
            page.fill('#window','0.2');page.locator('#window').press('Enter')
            page.click('#plotFitX')
            assert page.evaluate('plotView.span>20&&plotView.preset===.2')
            page.click('#plotReset');assert page.evaluate('plotView.span===.2')

            # Pause remains explicit, and numeric/range changes preserve its snapshot.
            page.click('#plotPause');frozen=page.evaluate('plotBounds().hi')
            page.fill('#window','1.25');page.locator('#window').press('Enter')
            page.locator('#plotBufferStart').press('ArrowLeft')
            page.wait_for_timeout(160)
            assert page.evaluate('plotView.frozen!==null') and page.evaluate('plotBounds().hi')==frozen
            page.click('#plotPause')
            page.select_option('#plotCount','2');page.select_option('#plotTarget','2')
            page.locator('#signals [data-channel="range.actual|rad/s"] input').check()
            page.fill('#window-2','3.75');page.locator('#window-2').press('Enter')
            assert page.evaluate('plots[1].view.preset===3.75&&plots[0].view.preset===1.25')
            for width in (1540,768,390):
                page.set_viewport_size({'width':width,'height':1100});page.evaluate('draw()')
                assert page.evaluate('document.body.scrollWidth<=innerWidth'),width
                assert page.locator('#plotBufferMove').is_visible()
            ids=page.locator('[id]').evaluate_all('els=>els.map(e=>e.id)')
            assert len(ids)==len(set(ids))
            page.evaluate('clearInterval(rangeTimer)')
            page.set_viewport_size({'width':1540,'height':1400})
            page.select_option('#plotTarget','1')
            x,y=position('#plotBufferMove');page.mouse.move(x,y);page.mouse.down()
            assert page.evaluate("plots[0].view.drag?.kind==='buffer'")
            page.evaluate("add({kind:2,epoch:1,ms:10,values:[['range.target','rad/s',50],['range.actual','rad/s',49]]});draw()")
            assert page.evaluate('plots.every(p=>p.view.drag===null&&p.view.frozen===null)')
            assert not page.locator('#plotBuffer').get_attribute('data-drag')
            assert page.evaluate('plots[0].view.preset===1.25&&plots[1].view.preset===3.75')
            page.mouse.up()
            assert page.evaluate('rangeCommands')==[]
            assert not errors,errors
            context.close()
            print(f'PASS: Edge DPR {dpr}: anchored red/purple endpoint stretching, green translation while 100 Hz stream continues, independent close handles, keyboard, fractional/long presets, invalid-draft safety, pause retention, per-plot settings and narrow layouts')
    finally:
        browser.close()
