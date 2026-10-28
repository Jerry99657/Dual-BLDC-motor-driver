"""Real wall-clock Edge soak; local Native packet source, no device/motor access.

Default duration is 1800 seconds. FOC_SOAK_SECONDS permits a shorter smoke run;
the report explicitly records whether the full 30 minutes were completed.
Output goes to FOC_SOAK_OUT, or the OS temporary directory (not firmware build/).
"""
import hashlib
import json
import os
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(os.environ.get('FOC_SOAK_OUT', str(Path(tempfile.gettempdir()) / 'foc-pc-soak')))
OUT.mkdir(parents=True, exist_ok=True)
SECONDS = float(os.environ.get('FOC_SOAK_SECONDS', '1800'))
assert SECONDS > 0
SOURCE = ROOT / 'pc_tool/index.html'
SOURCE_HASH = hashlib.sha256(SOURCE.read_bytes()).hexdigest()

SETUP = r"""()=>{
 connected=true;parser=new WireParser();seenParserEpoch=0;lastEpoch=++recordEpoch;clear();setOutputFormat('Native');
 window.soak={sample:0,sequence:0,commands:[],paints:Array(6).fill(0),stopTimes:[],confirmations:0};
 window.soakWire=(kind,payload,ms)=>{
   const bytes=new Uint8Array(payload.length+14),v=new DataView(bytes.buffer);
   bytes.set([165,90,1,kind]);v.setUint16(4,soak.sequence++&65535,true);v.setUint16(6,payload.length,true);v.setUint32(8,ms,true);bytes.set(payload,12);
   v.setUint16(bytes.length-2,crc16(bytes.subarray(0,-2)),true);
   const consumeBytes=part=>{for(const frame of parser.feed(part)){if(frame.epoch!==seenParserEpoch){seenParserEpoch=frame.epoch;++recordEpoch}frame.epoch=recordEpoch;consume(frame)}};
   if(soak.sample%37===0){consumeBytes(bytes.subarray(0,9));consumeBytes(bytes.subarray(9))}else consumeBytes(bytes);
 };
 const payload=new Uint8Array(26),v=new DataView(payload.buffer);v.setUint16(24,63,true);
 const state=new Uint8Array(96);state.set([0,0,1,0,2,0,152,0,0,0,0,0,64,1]);
 for(const [i,text]of ['RUN','STOP','OK','IDLE'].entries())state.set(new TextEncoder().encode(text),32+i*16);
 window.feedSoak=n=>{for(let i=0;i<n;i++){
   const ms=soak.sample++*5;[100,100+Math.sin(ms*.007)+.1*Math.sin(ms*.3),0,ms/1000,.2,20].forEach((x,j)=>v.setFloat32(j*4,x,true));
   soakWire(1,payload,ms);if(soak.sample%10===0)soakWire(16,state,ms);
 }};
 document.getElementById('window').value='100';document.getElementById('window').onchange();feedSoak(20001);setPlotCount(6);
 for(const p of plots){p.control('window').value='100';p.control('window').onchange();assignPlotChannel(p.index,'m1.target_speed|rad/s',true);assignPlotChannel(p.index,'m1.feedback_speed|rad/s',true)}activatePlot(0);draw();
 for(const p of plots){const native=p.draw;p.draw=function(){soak.paints[p.index]++;return native()}};
 const start=performance.now(),base=soak.sample;
 window.soakTimer=setInterval(()=>feedSoak(Math.max(0,base+Math.floor((performance.now()-start)/5)-soak.sample)),5);
 window.send=async command=>{if(command!=='Stop')throw Error('Unexpected test command: '+command);soak.commands.push(command);soak.stopTimes.push(performance.now()-soak.stopClick);return {result:1}};
 document.getElementById('stop').addEventListener('click',()=>soak.stopClick=performance.now(),true);
 window.soakStatusNodes=[...document.getElementById('status').children];
 window.soakRead=()=>({sample:soak.sample,paints:soak.paints.slice(),commands:soak.commands.length,maxStopMs:Math.max(0,...soak.stopTimes),
   lengths:[...series.values()].map(s=>s.points.length),memory:cacheMemory(),statusStable:soakStatusNodes.every((n,i)=>document.getElementById('status').children[i]===n),
   runtime:runtimeSnapshot(),animations:document.getAnimations().length});
}"""

def utc():
    return datetime.now(timezone.utc).isoformat()

with sync_playwright() as p:
    browser = p.chromium.launch(channel='msedge' if os.name == 'nt' else 'chromium', headless=True)
    report = {'sourceSHA256': SOURCE_HASH, 'browser': browser.version, 'requestedSeconds': SECONDS,
              'source': 'synthetic Native 200 Hz, 6 channels + 20 Hz status, fragmented frames; no serial device',
              'viewport': [1540, 1200], 'checkpoints': [], 'errors': [], 'crashes': [], 'network': []}
    try:
        page = browser.new_page(viewport={'width': 1540, 'height': 1200})
        page.on('pageerror', lambda e: report['errors'].append(str(e)))
        page.on('crash', lambda: report['crashes'].append(utc()))
        page.on('request', lambda r: report['network'].append(r.url) if r.url.startswith(('http:', 'https:')) else None)
        print('SETUP: loading local HTML', flush=True)
        page.goto(SOURCE.as_uri())
        page.wait_for_timeout(800)
        assert page.evaluate('plots.every(p=>p.selected.size===0)&&series.size===0'), 'Start empty'
        print('SETUP: feeding native seed and binding 6 canvases', flush=True)
        page.evaluate(SETUP)
        print('SETUP: seed ready', flush=True)
        page.locator('#plot').scroll_into_view_if_needed()
        print('SETUP: viewport ready', flush=True)
        page.wait_for_timeout(1800)
        print('SETUP: collecting baseline', flush=True)
        cdp = page.context.new_cdp_session(page)

        def measure():
            cdp.send('HeapProfiler.collectGarbage')
            result = page.evaluate('soakRead()')
            result['heap'] = cdp.send('Runtime.getHeapUsage')
            result['dom'] = cdp.send('Memory.getDOMCounters')
            return result

        report['startedAt'] = utc()
        begin = time.monotonic()
        report['baseline'] = measure()
        print('SETUP: baseline ready', flush=True)
        next_check, phase, release, release_kind, frozen = 0, 0, None, None, None
        while time.monotonic() - begin < SECONDS:
            elapsed = time.monotonic() - begin
            if release is not None and elapsed >= release:
                if release_kind == 'pause':
                    assert page.evaluate('plots.map(p=>p.bounds().hi)') == frozen
                    page.evaluate("document.getElementById('pause').click()")
                    assert page.evaluate('cacheMemory().snapshotBytes===0')
                elif release_kind == 'hidden':
                    page.evaluate('setPlotCount(6)')
                    assert page.evaluate('plots.every(p=>p.selected.size===2&&p.view.cacheSeconds===100)')
                release = None
            if elapsed >= next_check:
                # Rotate visible charts, exercise real controls and full animation
                # lifecycles while the source keeps running.
                page.locator('#plot' if phase % 3 == 0 else f'#plot-{3 if phase % 3 == 1 else 5}').scroll_into_view_if_needed()
                page.wait_for_timeout(300)
                print(f'CHECK {phase}: Stop and confirmation', flush=True)
                page.click('#stop')
                page.evaluate("()=>{window.soakQuestion=confirmAction('持续运行测试：取消本地确认框',{title:'本地验证',action:'确认'}).then(x=>{if(x)throw Error('Unexpected confirmation');soak.confirmations++})}")
                page.wait_for_timeout(350)
                page.keyboard.press('Escape')
                page.wait_for_function('!document.querySelector("#uiConfirm[open]")')
                if phase % 6 == 1:
                    page.evaluate("document.getElementById('pause').click()")
                    assert page.evaluate('plots.every(p=>!!p.view.frozen)&&cacheMemory().snapshotStores===2')
                    frozen = page.evaluate('plots.map(p=>p.bounds().hi)')
                    release, release_kind = elapsed + 10, 'pause'
                elif phase % 6 == 3:
                    page.evaluate('setPlotCount(2)')
                    release, release_kind = elapsed + 10, 'hidden'
                elif phase % 6 == 5:
                    page.click('#themeToggle')
                state = measure()
                assert state['statusStable'], 'Status DOM identity changed'
                assert all(19999 <= n <= 20001 for n in state['lengths']), state['lengths']
                assert state['memory']['totalBytes'] <= state['memory']['budgetBytes']
                assert len(state['runtime']['history']) <= 120 and len(state['runtime']['errors']) <= 100
                assert state['maxStopMs'] < 50, state['maxStopMs']
                assert not report['errors'] and not report['crashes'] and not report['network'], report
                state['elapsedSeconds'] = time.monotonic() - begin
                # Avoid duplicating bounded diagnostic history in every checkpoint.
                state['runtime'].pop('history', None)
                report['checkpoints'].append(state)
                (OUT / 'progress.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
                print(f"SOAK {state['elapsedSeconds']:.1f}/{SECONDS:.0f}s, frames={state['sample']}, retained={state['memory']['totalBytes']}, heap={state['heap']['usedSize']}, stop={state['maxStopMs']:.3f}ms", flush=True)
                next_check += 60
                phase += 1
            page.wait_for_timeout(min(5000, max(1, (SECONDS - (time.monotonic() - begin)) * 1000)))

        if release_kind == 'pause' and release is not None:
            page.evaluate("document.getElementById('pause').click()")
        page.evaluate('setPlotCount(6)')
        report['final'] = measure()
        report['elapsedSeconds'] = time.monotonic() - begin
        report['finishedAt'] = utc()
        report['fullThirtyMinutes'] = report['elapsedSeconds'] >= 1800
        assert report['final']['sample'] - report['baseline']['sample'] >= SECONDS * 195
        assert report['final']['heap']['usedSize'] <= report['baseline']['heap']['usedSize'] + 16 * 1024 * 1024
        assert report['final']['heap'].get('backingStorageSize', 0) <= report['baseline']['heap'].get('backingStorageSize', 0) + 4 * 1024 * 1024
        assert report['final']['dom']['nodes'] <= report['baseline']['dom']['nodes'] + 150
        assert not report['errors'] and not report['crashes'] and not report['network']
        assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == SOURCE_HASH, 'Page source changed during soak'
        with page.expect_download() as info:
            page.click('#exportDiagnostics')
        diagnostic = Path(info.value.path()).read_bytes()
        (OUT / 'browser-diagnostics.json').write_bytes(diagnostic)
        assert json.loads(diagnostic)['parser']['crcErrors'] == 0
        page.screenshot(path=str(OUT / 'soak-final.png'))
        page.evaluate('clearInterval(soakTimer)')
        report['passed'] = True
        (OUT / 'stability-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f"PASS: {report['elapsedSeconds']:.1f} REAL seconds; fullThirtyMinutes={report['fullThirtyMinutes']}; report={OUT / 'stability-report.json'}", flush=True)
    except Exception as error:
        report['passed'] = False
        report['failure'] = str(error)
        (OUT / 'stability-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        raise
    finally:
        browser.close()
