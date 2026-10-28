"""Duration retention through real Canvas/UI/CSV. No serial port is opened."""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/cache-duration-r102'
OUT.mkdir(exist_ok=True)

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        page=browser.new_page(viewport={'width':1540,'height':1200})
        errors=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        assert page.locator('#window').get_attribute('aria-label')=='缓存时长秒数'
        assert page.locator('#plotApplyWindow').inner_text()=='应用缓存'
        page.fill('#window','100');page.locator('#window').press('Enter')
        page.evaluate("""()=>{
            window.cacheCommands=[];window.send=async command=>cacheCommands.push(command);
            window.cacheSample=0;
            window.feedCache=end=>{for(;cacheSample<=end;cacheSample++){
                const t=cacheSample*.005;
                add({kind:1,epoch:0,ms:cacheSample*5,values:[
                    ['m1.target_speed','rad/s',100],['m1.feedback_speed','rad/s',100+Math.sin(t)],
                    ['m1.position','rad',t],['m1.iq','A',.2],['m2.feedback_speed','rad/s',90],['vbus','V',20]]});
            }draw()};
            feedCache(30000);addAllPlotChannels();
        }""")
        assert page.evaluate('[...series.values()].every(s=>s.points.length===20001)')
        assert page.evaluate('plotBounds().lo===50&&plotBounds().hi===150')
        assert '缓存跨度 100 s' in page.locator('#plotWindowState').inner_text()
        page.click('#plotZoomIn')
        assert page.evaluate('plotView.span===80&&plotView.cacheSeconds===100')
        page.click('#plotApplyWindow')
        assert page.evaluate('plotView.span===100&&plotView.offset===0')
        page.select_option('#plotCount','2')
        page.locator('#signals [data-channel="m1.feedback_speed|rad/s"] input').check()
        page.evaluate("assignPlotChannel(1,'m1.feedback_speed|rad/s',true)")
        page.fill('#window-2','5');page.locator('#window-2').press('Enter')
        assert page.evaluate('plots[0].bounds().lo===50&&plots[1].bounds().lo===145')
        assert page.evaluate("series.get('m1.feedback_speed|rad/s').points.length===20001")
        page.select_option('#plotCount','1');page.select_option('#plotCount','2')
        assert page.evaluate('plots[1].view.cacheSeconds===5&&plots[1].bounds().lo===145')

        page.fill('#window','120');page.locator('#window').press('Enter')
        assert page.evaluate('plots[0].bounds().lo===50'), 'Larger retention cannot recover discarded samples'
        page.evaluate('feedCache(36000)')
        assert page.evaluate('[...series.values()].every(s=>s.points.length===24001)')
        assert page.evaluate('plots[0].bounds().lo===60&&plots[0].bounds().hi===180')
        page.click('#plotPause')
        page.evaluate('feedCache(48000)')
        assert page.evaluate('plots[0].bounds().hi===180&&plots[1].bounds().hi===240')
        page.fill('#window','20');page.locator('#window').press('Enter')
        assert page.evaluate('plots[0].view.frozen!==null&&plots[0].bounds().lo===160&&plots[0].bounds().hi===180')
        assert page.evaluate('[...series.values()].every(s=>s.points.length===4001)')
        page.fill('#window','100');page.locator('#window').press('Enter')
        assert page.evaluate('plots[0].bounds().lo===160'), 'Growing paused retention does not fabricate history'
        page.click('#plotPause');page.evaluate('feedCache(68000)')
        assert page.evaluate('plots[0].bounds().lo===240&&plots[0].bounds().hi===340')
        assert page.evaluate('[...series.values()].every(s=>s.points.length===20001)')

        # The imported CSV uses the same time window, not the old 2200-point cap.
        rows=['host_time,board_ms,sequence,epoch,packet_kind,field,unit,value']
        rows.extend(f't,{i*5},{i},0,1,m1.feedback_speed,rad/s,{i}' for i in range(30001))
        page.set_input_files('#file',{'name':'duration.csv','mimeType':'text/csv','buffer':('\n'.join(rows)+'\n').encode()})
        page.wait_for_function("replaying&&latestTime===150&&document.getElementById('replay').textContent.includes('按缓存时长')")
        assert page.evaluate("series.get('m1.feedback_speed|rad/s').points.length===20001")
        assert page.evaluate('plots[0].bounds().lo===50&&plots[0].bounds().hi===150&&plots[1].bounds().lo===145')
        assert not errors,errors
        assert page.evaluate('cacheCommands')==[]
        for width in (375,768,1024,1540):
            page.set_viewport_size({'width':width,'height':1100})
            assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),width
        page.locator('#plotCard-1').screenshot(path=str(OUT/'duration-100s.png'))
        print('PASS: real Edge: 100/120 s at 200 Hz beyond 2200 samples, original data, zoom-independent retention, shared/hidden canvases, immediate shrink/gradual growth, paused snapshots, timed CSV replay, 375/768/1024/1540 layouts and zero commands')
    finally:
        browser.close()
