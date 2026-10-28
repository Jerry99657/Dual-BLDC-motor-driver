"""Native-backed forms, restrained hover feedback and measured task progress."""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/console-polish'
OUT.mkdir(exist_ok=True)

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        page=browser.new_page(viewport={'width':1540,'height':1200},reduced_motion='no-preference')
        errors=[];network=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('request',lambda r:network.append(r.url))
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        page.wait_for_timeout(900)
        page.hover('#demo')
        assert page.locator('#demo').evaluate("e=>getComputedStyle(e,'::after').animationName")=='ui-button-edge'
        page.hover('#serialPanel');page.wait_for_timeout(450)
        assert page.locator('#serialPanel').evaluate("e=>getComputedStyle(e,'::before').transform")=='matrix(1, 0, 0, 1, 0, 0)'
        page.evaluate("add({kind:1,epoch:0,ms:0,values:[['m1.feedback_speed','rad/s',5]]});draw()")
        row=page.locator('#signals .signal-source').first
        row.hover();page.wait_for_timeout(300)
        assert row.locator('label').evaluate('e=>getComputedStyle(e).transform')=='matrix(1, 0, 0, 1, 6, 0)'

        fixture=(ROOT/'build/console-r87/rpc-fixture.bin').read_bytes()[:106]
        page.evaluate("""bytes=>{
            window.formConfig=decodeReply(new WireParser().feed(new Uint8Array(bytes))[0]);
            formConfig.work=0;formConfig.drive=0;formConfig.settings.control_config&=~(7|8|16384|32768);
            connected=true;boardConfig=formConfig;populateControls(formConfig);populateSettings();renderConfig();
            window.polishWrites=[];window.send=async command=>{polishWrites.push(command);return {result:1}};
            window.segmentEvents=0;document.getElementById('plotAxis').addEventListener('change',()=>segmentEvents++);
        }""",list(fixture))
        led=page.locator('#ledEnable');initial=led.is_checked()
        assert led.get_attribute('role')=='switch'
        led.set_checked(not initial);assert led.is_checked()!=initial
        assert page.evaluate('polishWrites')==[]
        group=page.locator('#plotCard-1 [data-select-id="plotAxis"]')
        group.get_by_role('radio',name='仅 X',exact=True).click()
        assert page.locator('#plotAxis').input_value()=='x'
        assert page.evaluate('segmentEvents')==1
        group.get_by_role('radio',name='仅 X',exact=True).press('ArrowRight')
        assert page.locator('#plotAxis').input_value()=='y'
        assert page.evaluate('segmentEvents')==2
        page.select_option('#plotAxis','xy')
        assert group.get_by_role('radio',name='X + Y',exact=True).get_attribute('aria-checked')=='true'
        page.evaluate('document.getElementById("plotAxis").disabled=true')
        page.wait_for_function("[...document.querySelectorAll('#plotCard-1 [data-select-id=plotAxis] button')].every(e=>e.disabled)")
        page.evaluate('document.getElementById("plotAxis").disabled=false')
        page.evaluate('formConfig.drive=1;formConfig.settings.control_config|=8;renderConfig()')
        assert page.locator('#currentLoop').is_disabled() and page.locator('#currentLoop').is_checked()
        page.evaluate('formConfig.drive=0;formConfig.settings.control_config&=~8;renderConfig()')
        assert not page.locator('#currentLoop').is_disabled()
        assert page.evaluate('polishWrites')==[], 'Native-backed switches/segments only edit local UI state'

        # Count raw bytes across a split UTF-8 character, and never show 100%
        # until EOF and parsing succeed, even after all bytes have been read.
        page.evaluate(r"""()=>{
            const text='\ufeffhost_time,board_ms,sequence,epoch,packet_kind,field,unit,value\n'+
                't,0,0,0,1,m1.feedback_speed,rad/s,5\n'+
                't,10,1,0,33,diagnostic,json,"含中文数据"\n'+
                't,1000,2,0,1,m1.feedback_speed,rad/s,6\n';
            window.csvBytes=new TextEncoder().encode(text);window.csvCut=csvBytes.findIndex((b,i)=>i>100&&b>=224)+1;
            window.slowFile={name:'分块.csv',size:csvBytes.length,stream:()=>new ReadableStream({start(c){window.csvController=c}})};
            window.csvRun=act(()=>loadCsv(slowFile));
            csvController.enqueue(csvBytes.slice(0,csvCut));
        }""")
        page.wait_for_function("document.getElementById('csvProgress').dataset.completed===String(csvCut)")
        assert page.evaluate('taskProgress.csv.completed===csvCut&&taskProgress.csv.status==="running"')
        page.evaluate('csvController.enqueue(csvBytes.slice(csvCut))')
        page.wait_for_function("document.getElementById('csvProgress').dataset.completed===String(csvBytes.length)")
        assert page.locator('#csvProgress [role=progressbar]').get_attribute('aria-valuenow')=='99'
        assert page.locator('#csvProgress i[data-filled=true]').count()==23
        page.evaluate('csvController.close()');page.evaluate('csvRun')
        page.wait_for_function("document.getElementById('csvProgress').dataset.status==='done'")
        assert page.locator('#csvProgress [role=progressbar]').get_attribute('aria-valuenow')=='100'
        assert page.locator('#csvProgress i[data-filled=true]').count()==24
        assert page.evaluate("series.get('m1.feedback_speed|rad/s').points.length===2&&series.get('m1.feedback_speed|rad/s').value===6")
        assert page.evaluate('plots[0].selected.size===0&&plots[0].layout.panels[0].items.length===0'), 'CSV import keeps an unconfigured canvas empty'

        # The existing Return to live action cancels a blocked import, so it
        # cannot overwrite live data or emit a false completion notification.
        page.evaluate(r"""()=>{
            window.csvCancelled=0;
            const file={name:'等待.csv',size:1000,stream:()=>new ReadableStream({start(c){window.cancelController=c},cancel(){csvCancelled++}})};
            Object.defineProperty(document.getElementById('file'),'files',{configurable:true,get:()=>[file]});
            document.getElementById('file').dispatchEvent(new Event('change',{bubbles:true}));
            cancelController.enqueue(new TextEncoder().encode('host_time,board_ms,sequence,epoch,packet_kind,field,unit,value\n'));
        }""")
        page.wait_for_function("taskProgress.csv.status==='running'&&taskProgress.csv.completed>0")
        page.click('#live')
        page.wait_for_function("document.getElementById('csvProgress').dataset.status==='cancelled'")
        assert page.evaluate('!replaying&&series.size===0&&csvCancelled===1')
        assert not page.locator('#uiToasts').get_by_text('CSV 回放已载入',exact=True).count()

        # A failed parse has a failed state even if every source byte was read.
        page.evaluate("""()=>{const file=new Blob(['bad header']);file.name='bad.csv';window.failedCsv=act(()=>loadCsv(file))}""")
        page.evaluate('failedCsv')
        page.wait_for_function("document.getElementById('csvProgress').dataset.status==='error'")
        assert page.locator('#csvProgress [role=progressbar]').get_attribute('aria-valuenow')!='100'
        page.click('#live')

        # A replacement import owns both the plot and progress after the old
        # blocked reader is cancelled, including when cleanup resolves later.
        page.evaluate(r"""()=>{
            window.supersededCancelled=0;
            const older={name:'older.csv',size:1000,stream:()=>new ReadableStream({
                start(c){window.olderController=c},cancel(){supersededCancelled++}
            })};
            window.olderRun=loadCsv(older);
            olderController.enqueue(new TextEncoder().encode('host_time,board_ms,sequence,epoch,packet_kind,field,unit,value\n'));
        }""")
        page.wait_for_function("taskProgress.csv.status==='running'&&taskProgress.csv.completed>0")
        page.evaluate(r"""()=>{
            const newer=new Blob(['host_time,board_ms,sequence,epoch,packet_kind,field,unit,value\n'+
                't,0,0,0,1,m1.feedback_speed,rad/s,9\n']);
            newer.name='newer.csv';window.newerRun=loadCsv(newer);
        }""")
        page.evaluate('newerRun')
        assert page.evaluate('olderRun')=={'cancelled':True}
        page.wait_for_function("document.getElementById('csvProgress').dataset.status==='done'")
        assert page.evaluate("supersededCancelled===1&&taskProgress.csv.label==='newer.csv'&&taskProgress.csv.status==='done'&&series.get('m1.feedback_speed|rad/s').value===9")
        page.click('#live')

        # The total is unknown until the first actual row reply. Resolve only
        # measured replies; failures and Stop keep the completed count partial.
        page.evaluate("""()=>{
            window.diagWait=[];window.polishWrites=[];
            window.send=command=>{
                polishWrites.push(command);
                if(command==='Stop'){batchEpoch++;for(const pending of diagWait.splice(0))pending.reject(Error('已发送停止'));return Promise.resolve({result:1})}
                return new Promise((resolve,reject)=>diagWait.push({resolve,reject,command}));
            };
            window.replyDiag=(count)=>{const item=diagWait.shift();item.resolve({count,label:'诊断行',value:item.command,ms:10})};
        }""")
        page.click('#readDiag')
        page.wait_for_function("document.getElementById('diagnosticsProgress').dataset.status==='running'")
        assert page.locator('#diagnosticsProgress [role=progressbar]').get_attribute('aria-valuenow') is None
        page.wait_for_timeout(150)
        assert page.evaluate('taskProgress.diagnostics.completed')==0
        for count,percent in ((1,'33'),(2,'66'),(3,'100')):
            page.evaluate('replyDiag(3)')
            page.wait_for_function("value=>document.querySelector('#diagnosticsProgress [role=progressbar]').getAttribute('aria-valuenow')===value",arg=percent)
            assert page.evaluate('taskProgress.diagnostics.completed')==count
        assert page.locator('#diagRows .config-row').count()==3

        page.click('#readDiag');page.evaluate('replyDiag(4)')
        page.wait_for_function('taskProgress.diagnostics.completed===1')
        page.evaluate("diagWait.shift().reject(Error('板端拒绝专题'))")
        page.wait_for_function("document.getElementById('diagnosticsProgress').dataset.status==='error'")
        assert page.locator('#diagnosticsProgress [role=progressbar]').get_attribute('aria-valuenow')=='25'
        page.click('#readDiag');page.evaluate('replyDiag(4)')
        page.wait_for_function('taskProgress.diagnostics.completed===1&&diagWait.length===1')
        page.click('#stop')
        page.wait_for_function("document.getElementById('diagnosticsProgress').dataset.status==='cancelled'")
        assert page.locator('#diagnosticsProgress [role=progressbar]').get_attribute('aria-valuenow')=='25'
        assert page.evaluate('polishWrites.at(-1)')=='Stop'

        for theme in ('dark','light'):
            if page.locator('html').get_attribute('data-theme')!=theme:
                page.click('#themeToggle');page.wait_for_function('t=>document.documentElement.dataset.theme===t',arg=theme)
            for width in (375,768,1024,1540):
                page.set_viewport_size({'width':width,'height':1100})
                assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),(theme,width)
        ids=page.locator('[id]').evaluate_all('els=>els.map(e=>e.id)')
        assert len(ids)==len(set(ids))
        page.emulate_media(reduced_motion='reduce');page.wait_for_timeout(50)
        assert page.evaluate("document.getAnimations().filter(a=>a.playState==='running').length")==0
        page.locator('#settingsPanel').screenshot(path=str(OUT/'native-backed-forms.png'))
        page.locator('#diagnosticsPanel').screenshot(path=str(OUT/'measured-progress.png'))
        assert not errors,errors
        assert not [url for url in network if url.startswith(('http:','https:'))]
        print('PASS: polish: edge/panel/row feedback, native switch guards and radio-keyboard/events, no automatic commands, measured UTF-8 bytes and returned rows, 99% until validated EOF, parse failure, live-return/superseding-import cancellation, Stop cancellation, partial progress, themes/layouts/reduced motion, unique IDs and zero network requests')
    finally:
        browser.close()
