"""DOM reuse, viewport paints, shared snapshots and actual local diagnostic export."""
import os,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/performance-ui'
OUT.mkdir(exist_ok=True)
with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        page=browser.new_page(viewport={'width':1540,'height':1200});errors=[];network=[]
        page.on('pageerror',lambda e:errors.append(str(e)));page.on('request',lambda r:network.append(r.url))
        page.goto((ROOT/'pc_tool/index.html').as_uri());page.wait_for_timeout(800)
        page.evaluate("""()=>{
            document.getElementById('window').value='100';document.getElementById('window').onchange();window.perfSample=0;
            window.feedPerformance=n=>{for(let i=0;i<n;i++){const ms=perfSample++*5;add({kind:1,epoch:0,ms,values:[['m1.target_speed','rad/s',100],['m1.feedback_speed','rad/s',100+Math.sin(ms)]]})}};
            feedPerformance(20001);setPlotCount(6);for(const p of plots){p.control('window').value='100';p.control('window').onchange();assignPlotChannel(p.index,'m1.target_speed|rad/s',true);assignPlotChannel(p.index,'m1.feedback_speed|rad/s',true)}activatePlot(0);draw();
            window.perfWrites=[];window.send=async command=>{perfWrites.push(command);return {result:1}};
            window.perfPaints=Array(6).fill(0);for(const p of plots){const native=p.draw;p.draw=function(){perfPaints[p.index]++;return native()}};
            window.perfStatus={work:'Single M1',drive:'ENCODER',stream:'M1',flags:[152,0],axis_status:['RUN','STOP'],mode:['Speed','Torque'],input:['USB','--'],encoders:[true,false],control_result:0,hso_result:0,command_status:'OK',detect_result:0,detect_status:'IDLE',detect_ls_uh:0,detect_flux_uwb:0};status(perfStatus);
            window.perfStatusNodes=[...document.getElementById('status').children];window.perfValueNode=document.querySelector('#latest .value b');
            const start=performance.now(),base=perfSample;window.perfTimer=setInterval(()=>feedPerformance(Math.max(0,base+Math.floor((performance.now()-start)/5)-perfSample)),5);
        }""")
        page.locator('#plot').scroll_into_view_if_needed();page.wait_for_timeout(350)
        page.evaluate("for(let i=0;i<100;i++){perfStatus.command_status='OK '+i;status(perfStatus)}")
        assert page.evaluate("perfStatusNodes.every((node,i)=>document.getElementById('status').children[i]===node)")
        assert page.evaluate("document.querySelector('#latest .value b')===perfValueNode"), 'Live values retain their DOM nodes'
        page.locator('#runtimePanel').scroll_into_view_if_needed()
        page.wait_for_function('plots.every(p=>p.inViewport===false)')
        before=page.evaluate('({paints:perfPaints.slice(),sample:perfSample})');page.wait_for_timeout(350)
        assert page.evaluate('perfSample')>before['sample']
        assert page.evaluate('perfPaints')==before['paints'], 'No automatic paints while all canvases are offscreen'
        page.locator('#plot-5').scroll_into_view_if_needed();page.wait_for_function('plots[4].inViewport')
        before=page.evaluate('perfPaints.slice()');page.wait_for_timeout(350)
        assert page.evaluate('perfPaints[4]')>before[4]
        assert page.evaluate('perfPaints[0]')==before[0]
        page.evaluate("document.getElementById('pause').click()")
        assert page.evaluate('plots.every(p=>p.view.frozen!==null)&&cacheMemory().snapshotStores===2')
        snapshot=page.evaluate('plots.map(p=>p.bounds().hi)');page.wait_for_timeout(250)
        assert page.evaluate('plots.map(p=>p.bounds().hi)')==snapshot
        page.evaluate("plots[0].control('window').value='10';plots[0].control('window').onchange()")
        assert page.evaluate('plots[0].bounds().hi-plots[0].bounds().lo===10&&plots[1].bounds().hi-plots[1].bounds().lo===100')
        page.evaluate("document.getElementById('pause').click()")
        assert page.evaluate('cacheMemory().snapshotBytes===0&&plots.every(p=>!p.view.frozen)')
        page.evaluate("for(let i=0;i<150;i++)rememberRuntimeError('受控测试错误 '+i,'test');window.dispatchEvent(new ErrorEvent('error',{message:'受控脚本错误'}))")
        page.wait_for_timeout(1100)
        page.locator('#runtimeDetails summary').click()
        assert page.locator('#runtimeMemory').inner_text()!='--'
        with page.expect_download() as info:page.click('#exportDiagnostics')
        payload=json.loads(Path(info.value.path()).read_text(encoding='utf-8'))
        assert payload['format']=='FOC-browser-diagnostics' and payload['version']==1
        assert len(payload['errors'])==100 and payload['errors'][-1]['message']=='受控脚本错误'
        assert payload['memory']['totalBytes']<=payload['memory']['budgetBytes']
        assert len(payload['history'])<=120 and len(payload['plots'])==6
        assert payload['rendering']['indexNodes']>0 and payload['browser']['dpr']==1
        assert not any(k in json.dumps(payload) for k in ('"times"','"values"','"payload"'))
        assert page.evaluate('perfWrites')==[], 'All diagnostics and viewport operations stay local'
        page.wait_for_function("document.getElementById('runtimeValues').getAnimations().length===0")
        assert page.locator('#runtimeValues .config-row').count()==6
        assert page.locator('#runtimeViews').is_visible()
        page.screenshot(path=str(OUT/'local-runtime.png'))
        for width in (768,390):
            page.set_viewport_size({'width':width,'height':1000})
            page.locator('#runtimePanel').scroll_into_view_if_needed()
            assert page.evaluate('document.body.scrollWidth<=innerWidth'),width
            assert page.locator('#runtimeMemory').is_visible()
        page.click('#themeToggle');page.wait_for_timeout(600)
        assert page.locator('html').get_attribute('data-theme')=='dark'
        page.screenshot(path=str(OUT/'local-runtime-narrow-dark.png'))
        page.evaluate("clearInterval(perfTimer);window.dispatchEvent(new Event('pagehide'))")
        checkpoint=page.evaluate("JSON.parse(localStorage.getItem('foc-runtime-checkpoint'))")
        assert checkpoint['closed'] and checkpoint['memory']['livePoints']>0
        assert len(checkpoint['errors'])<=20 and 'previousSession' not in checkpoint
        page.reload();page.wait_for_timeout(400)
        previous=page.evaluate('runtimeSnapshot().previousSession')
        assert previous['format']=='FOC-browser-checkpoint' and previous['closed']
        assert previous['memory']['livePoints']==checkpoint['memory']['livePoints']
        assert page.evaluate('series.size===0&&plots.length===1&&plots[0].selected.size===0'), 'Checkpoint never restores raw samples or bindings'
        assert not errors,errors
        assert not [url for url in network if url.startswith(('http:','https:'))]
        print('PASS: stable live-value/status nodes, offscreen reception with zero paints, immediate visible repaint, shared immutable snapshots and independent shrink, released buffers, bounded errors/history, actual diagnostic JSON download, narrow/dark layout and checkpoint recovery with empty canvases; no raw samples or board commands')
    finally:browser.close()
