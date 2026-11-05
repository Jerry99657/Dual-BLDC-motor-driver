"""Actual single-HTML measurement UI, CSV, exports and live responsiveness.
Uses analytic samples and a command spy; never opens a hardware serial port.
"""
import csv
import io
import json
import math
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/measurement-html-20261105'
OUT.mkdir(exist_ok=True)

SEED="""()=>{
 clear();boardConfig=null;lastStatus={work:'Single M1',drive:'ENCODER',mode:['Speed','Speed'],flags:[255,0]};
 for(const p of plots){p.view.cacheSeconds=p.view.span=10}
 for(let i=0;i<=500;i++){const t=i*.01;add({kind:1,epoch:0,ms:i*10,values:[
  ['m1.target_speed','rad/s',t<1?0:10],['m1.feedback_speed','rad/s',t<1?0:10*(1-Math.exp(-(t-1)/.2))],
  ['m1.iq','A',.3],['vbus','V',12]]})}
 for(const key of ['m1.target_speed|rad/s','m1.feedback_speed|rad/s','m1.iq|A','vbus|V'])plots[0].selected.add(key);
 renderSignals();renderPlotChannels();draw();
 $('measureStart').value='0';$('measureEnd').value='5';$('measurePreset').value='speed1';refreshMeasurementChannels();$('measureMode').value='step';
}"""

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        page=browser.new_page(viewport={'width':1440,'height':1100},accept_downloads=True)
        errors=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        page.evaluate("()=>{window.measureCommands=[];window.send=async command=>{measureCommands.push(command)}}")
        page.evaluate(SEED)
        page.click('#measureAnalyze')
        page.wait_for_function('measurementCurrent?.result.step?.valid&&!measurementBusy')
        first=page.evaluate('measurementCurrent')
        assert abs(first['result']['step']['rise10to90']-.2*math.log(9))<.001
        assert first['meta']['electrical']=={'peakIqA':.3,'busMinV':12,'busMaxV':12}
        assert page.evaluate('measurementBytes')==0
        page.fill('#measureName','trial 2');page.click('#measureSave')
        assert page.locator('#measureHistory tr').count()==1
        with page.expect_download() as download:page.click('#measureExport')
        report=json.loads(Path(download.value.path()).read_text(encoding='utf-8'))
        assert report['format']=='FOC-control-measurement-report' and len(report['reports'])==2
        download.value.save_as(OUT/'measurement-report.json')
        with page.expect_download() as download:page.click('#measureCsv')
        rows=list(csv.reader(io.StringIO(Path(download.value.path()).read_text(encoding='utf-8-sig'))))
        assert len(rows)==3 and 'settling_s' in rows[0] and 'peak_iq_A' in rows[0]
        # Real pause snapshot stays unchanged while live cache receives more data.
        page.evaluate("freezePlot();for(let i=501;i<801;i++)add({kind:1,epoch:0,ms:i*10,values:[['m1.target_speed','rad/s',80],['m1.feedback_speed','rad/s',0]]})")
        page.click('#measureAnalyze');page.wait_for_function("measurementCurrent?.meta.source==='暂停快照'&&!measurementBusy")
        assert abs(page.evaluate('measurementCurrent.result.step.rise10to90')-first['result']['step']['rise10to90'])<1e-10
        assert page.evaluate('!!plots[0].view.frozen')
        page.evaluate("plots[0].resume();$('measurePreset').value='custom';refreshMeasurementChannels()")
        page.select_option('#measureReference','m1.target_speed|rad/s');page.select_option('#measureFeedback','vbus|V')
        page.click('#measureAnalyze');page.wait_for_function("$('measureState').textContent.includes('单位不一致')")
        assert page.evaluate('measurementCurrent.meta.source')=='暂停快照'
        # Import real console CSV with status metadata and no use of online config.
        text=io.StringIO();writer=csv.writer(text);writer.writerow(['host_time','board_ms','sequence','epoch','packet_kind','field','unit','value'])
        status={'work':'Single M1','drive':'ENCODER','mode':['Speed','Speed'],'flags':[255,0]}
        writer.writerow(['',0,0,7,16,'status','json',json.dumps(status)])
        for i in range(501):
            t=i*.01
            for name,value in [('m1.target_speed',0 if t<1 else 10),('m1.feedback_speed',0 if t<1 else 10*(1-math.exp(-(t-1)/.2)))]:
                writer.writerow(['',i*10,i+1,7,1,name,'rad/s',value])
        page.set_input_files('#file',{'name':'response.csv','mimeType':'text/csv','buffer':text.getvalue().encode()})
        page.wait_for_function("taskProgress.csv?.status==='done'")
        page.evaluate("$('measurePreset').value='speed1';refreshMeasurementChannels();$('measureStart').value='0';$('measureEnd').value='5'")
        page.click('#measureAnalyze');page.wait_for_function("measurementCurrent.meta.source==='CSV 回放'&&!measurementBusy")
        assert page.evaluate("measurementCurrent.meta.configuration===null&&measurementCurrent.meta.context.work==='Single M1'")
        assert abs(page.evaluate('measurementCurrent.result.step.rise10to90')-.2*math.log(9))<.001
        # Follow uses the actual firmware's signed group error, including origins.
        page.evaluate("""()=>{
         replaying=false;clear();lastStatus={work:'Master M2',drive:'ENCODER',mode:['Position','Position'],flags:[255,0]};
         for(let i=0;i<=500;i++){const t=i*.01,ref=.17*Math.sin(8*Math.PI*t),actual=.136*Math.sin(8*Math.PI*(t-.04));
          add({kind:3,epoch:9,ms:i*10,values:[['m1.position','rad',actual],['group.error','rad',actual-ref]]})}
         $('measurePreset').value='follow1';$('measureMode').value='sine';refreshMeasurementChannels();
         $('measureStart').value='0';$('measureEnd').value='5';
        }""")
        page.click('#measureAnalyze');page.wait_for_function("measurementCurrent?.result.sine?.valid&&!measurementBusy")
        assert abs(page.evaluate('measurementCurrent.result.sine.gain')-.8)<1e-8
        assert abs(page.evaluate('measurementCurrent.result.sine.equivalentLagSeconds')-.04)<1e-8
        page.locator('#measurePanel').screenshot(path=OUT/'measurement-desktop.png')
        page.set_viewport_size({'width':390,'height':844})
        page.locator('#measurePanel').scroll_into_view_if_needed()
        assert page.evaluate('document.documentElement.scrollWidth<=innerWidth+1'),'horizontal overflow'
        assert page.locator('.nav-item').count()==6
        page.screenshot(path=OUT/'measurement-mobile.png')
        page.set_viewport_size({'width':1440,'height':1100})
        # Different control modes in one numeric stream are not one experiment.
        page.evaluate("""()=>{
         clear();$('measurePreset').value='speed1';$('measureMode').value='tracking';
         for(let i=0;i<=500;i++){lastStatus={work:'Single M1',drive:'ENCODER',mode:[i<250?'Speed':'Position','Speed'],flags:[255,0]};
          add({kind:1,epoch:10,ms:i*10,values:[['m1.target_speed','rad/s',10],['m1.feedback_speed','rad/s',9]]})}
         refreshMeasurementChannels();
        }""")
        page.click('#measureAnalyze');page.wait_for_function("$('measureState').textContent.includes('跨越')")
        # Long analysis yields to real event handlers, live reception and Stop.
        page.evaluate("""()=>{
         clear();lastStatus={work:'Single M1',drive:'ENCODER',mode:['Speed','Speed'],flags:[255,0]};
         for(const p of plots){p.view.cacheSeconds=p.view.span=1200}
         for(let i=0;i<90000;i++)add({kind:1,epoch:11,ms:i*10,values:[['m1.target_speed','rad/s',Math.sin(i*.08)],['m1.feedback_speed','rad/s',.8*Math.sin(i*.08-.2)]]});
         $('measurePreset').value='speed1';$('measureMode').value='tracking';$('measureStart').value='0';$('measureEnd').value='899.99';refreshMeasurementChannels();
         window.beforeLive=runtimeStats.liveFrames;window.liveCounter=90000;
         window.measureFeed=setInterval(()=>add({kind:1,epoch:11,ms:liveCounter++*10,values:[['m1.target_speed','rad/s',0],['m1.feedback_speed','rad/s',0]]}),10);
        }""")
        page.click('#measureAnalyze');page.wait_for_function('measurementBusy')
        page.click('#stop');assert page.evaluate('measureCommands.at(-1)')=='Stop'
        page.click('#measureCancel');page.wait_for_function('!measurementBusy')
        page.evaluate('clearInterval(measureFeed)')
        assert page.evaluate('runtimeStats.liveFrames>beforeLive&&measurementBytes===0')
        assert '取消' in page.locator('#measureState').inner_text()
        assert page.evaluate('measureCommands')==['Stop'],page.evaluate('measureCommands')
        assert not errors,errors
        print('PASS: real Edge measurement controls, analytic step and Follow, immutable pause analysis, CSV metadata, JSON/CSV export, unit/config gates, 390px layout, bounded workspace, cancellation, live data and priority Stop during analysis; zero unsolicited commands')
    finally:browser.close()
