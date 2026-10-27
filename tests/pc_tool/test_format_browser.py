"""Real browser/Streams/Web Serial handshake against actual C frame fixtures.

No device is opened and no motor is commanded. Requires local Edge/Playwright.
"""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/justfloat-r101'

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        page=browser.new_page(viewport={'width':1440,'height':1100})
        errors=[]
        page.on('pageerror',lambda error: errors.append(str(error)))
        page.add_init_script("""(()=>{
            const board=window.fakeBoard={writes:[],format:'JustFloat',hold:false,sequence:300};
            const port={open:async options=>{board.options=options},close:async()=>{},
                readable:new ReadableStream({start(controller){board.rx=controller}}),
                writable:new WritableStream({write(bytes){
                    const text=new TextDecoder().decode(bytes).trim();board.writes.push(text);
                    if(text==='Format:Native')board.format='Native';
                    if(text==='Format:JustFloat')board.format='JustFloat';
                    const m=text.match(/^@(\\d+):Get$/);
                    if(m&&board.format==='Native'&&!board.hold){
                        const data=Uint8Array.from(board.reply),v=new DataView(data.buffer);
                        v.setUint16(4,board.sequence++,true);v.setUint16(12,Number(m[1]),true);
                        v.setUint16(data.length-2,crc16(data.subarray(0,-2)),true);board.rx.enqueue(data);
                    }
                }})};
            Object.defineProperty(navigator,'serial',{value:{requestPort:async()=>port},configurable:true});
        })()""")
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        reply=(ROOT/'build/console-r87/rpc-fixture.bin').read_bytes()[:106]
        jf=(ROOT/'build/telemetry-r86/justfloat-fixture.bin').read_bytes()
        page.evaluate('data=>{fakeBoard.reply=data.reply;fakeBoard.jf=data.jf}',{'reply':list(reply),'jf':list(jf)})
        assert page.evaluate('fakeBoard.writes')==[]
        page.click('#connect')
        page.wait_for_function("outputFormat==='Native'&&boardConfig?.firmware===101")
        assert page.evaluate('fakeBoard.writes')==['Format:Native','@1:Get']
        assert page.locator('#formatState').inner_text()=='Native 已确认 · 完整控制台'
        saved=page.evaluate('JSON.stringify(boardConfig.settings)')
        page.select_option('#wireFormat','JustFloat');page.click('#setFormat')
        page.wait_for_function("outputFormat==='JustFloat'&&!boardConfig")
        assert page.evaluate('fakeBoard.writes.at(-1)')=='Format:JustFloat'
        assert page.locator('#apply').is_disabled() and page.locator('#readDiag').is_disabled()
        assert not page.locator('#stop').is_disabled()
        page.evaluate('fakeBoard.rx.enqueue(Uint8Array.from(fakeBoard.jf))')
        page.wait_for_function('formatVerified')
        page.click('#stop');page.wait_for_function("fakeBoard.writes.at(-1)==='Stop'")
        count=page.evaluate('fakeBoard.writes.length')
        page.wait_for_timeout(150)
        assert page.evaluate('fakeBoard.writes.length')==count, 'No automatic Native override'
        page.select_option('#wireFormat','Native');page.click('#setFormat')
        page.wait_for_function("outputFormat==='Native'&&boardConfig!==null")
        assert page.evaluate('JSON.stringify(boardConfig.settings)')==saved
        page.evaluate('fakeBoard.hold=true')
        page.click('#readConfig');page.wait_for_function('pending.size===1')
        page.evaluate("fakeBoard.format='JustFloat';fakeBoard.rx.enqueue(Uint8Array.from(fakeBoard.jf))")
        page.wait_for_function("outputFormat==='JustFloat'&&pending.size===0")
        assert 'JustFloat' in page.locator('#error').inner_text()
        assert page.locator('#apply').is_disabled()
        page.evaluate('fakeBoard.hold=false')
        page.select_option('#wireFormat','Native');page.click('#setFormat')
        page.wait_for_function("outputFormat==='Native'&&boardConfig!==null")
        for width in (1440,768,390):
            page.set_viewport_size({'width':width,'height':1000})
            assert page.evaluate('document.body.scrollWidth<=innerWidth'),width
        page.set_viewport_size({'width':1440,'height':1100})
        page.screenshot(path=str(OUT/'native-console.png'),full_page=True)
        assert not errors,errors
        assert all(line=='Stop' or line.startswith('Format:') or line.endswith(':Get') for line in page.evaluate('fakeBoard.writes'))
        page.click('#disconnect');page.wait_for_function('port===null')
        print('PASS: real Edge Web Serial/Streams: raw Native-first/Get-confirmed handshake, R101 settings, explicit pure JustFloat, raw Stop, board-side format detection/cancel/recovery, no mode override/motion and responsive controls')
    finally:
        browser.close()
