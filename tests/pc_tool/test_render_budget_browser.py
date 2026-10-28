"""Dense waveform rendering stays bounded without changing retained samples."""
import json
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/render-budget'
OUT.mkdir(exist_ok=True)

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        page=browser.new_page(viewport={'width':1540,'height':1200})
        errors=[];crashes=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('crash',lambda:crashes.append(True))
        page.goto((ROOT/'pc_tool/index.html').as_uri())
        page.wait_for_timeout(800)
        geometry=page.evaluate("""()=>{
            const points=new PlotSamples();
            for(let i=0;i<20000;i++)points.push([i/1000,i>=4000&&i<4500?null:i===10005?777:i===10006?-444:Math.sin(i)]);
            const paths=[];let path=[];
            const context={beginPath(){path=[]},moveTo(x,y){path.push(['M',x,y])},lineTo(x,y){path.push(['L',x,y])},stroke(){paths.push(path)}};
            strokePlotSamples(context,points,0,points.length,0,20,0,20,0,1221,-444,777,1);
            const vertices=paths.flat();
            let previous=null,bridged=false;
            for(const [op,x,y]of vertices){if(op==='L'&&previous<4&&x>=4.5)bridged=true;previous=x}
            const dense={vertices:vertices.length,peak:vertices.some(v=>Math.abs(v[1]-10.005)<1e-9&&v[2]===0),
                valley:vertices.some(v=>Math.abs(v[1]-10.006)<1e-9&&v[2]===1221),bridged,
                intact:points.length===20000&&points.valueAt(10005)===777&&points.valueAt(4000)===null};
            const sparse=new PlotSamples();for(let i=0;i<5000;i++)sparse.push([i,i]);paths.length=0;
            strokePlotSamples(context,sparse,0,5000,0,5000,0,10000,0,5000,0,5000,1);
            const batched={paths:paths.length,max:Math.max(...paths.map(p=>p.length)),
                continuous:paths.slice(1).every((p,i)=>p[0][0]==='M'&&p[0][1]===paths[i].at(-1)[1]&&p[0][2]===paths[i].at(-1)[2]),
                samples:new Set(paths.flat().map(v=>v[1])).size};
            return {dense,batched};
        }""")
        assert geometry['dense']['vertices']<=100,geometry
        assert geometry['dense']['peak'] and geometry['dense']['valley'],geometry
        assert not geometry['dense']['bridged'] and geometry['dense']['intact'],geometry
        assert geometry['batched']=={'paths':3,'max':2048,'continuous':True,'samples':5000},geometry

        page.evaluate("""()=>{
            document.getElementById('window').value='100';document.getElementById('window').onchange();
            window.budgetSample=0;window.budgetPaints=0;window.budgetCommands=[];
            window.send=async command=>{budgetCommands.push(command);return {result:1}};
            window.feedBudget=n=>{for(let i=0;i<n;i++){
                const ms=budgetSample++*5;add({kind:1,epoch:0,ms,values:[
                    ['m1.target_speed','rad/s',100],['m1.feedback_speed','rad/s',100+Math.sin(ms*.7)],
                    ['m1.position','rad',ms/1000],['m1.iq','A',.2],['m2.feedback_speed','rad/s',90],['vbus','V',20]]});
            }};
            feedBudget(20001);setPlotCount(6);
            for(let i=0;i<6;i++){
                plots[i].view.cacheSeconds=plots[i].view.span=100;
                assignPlotChannel(i,'m1.target_speed|rad/s',true);assignPlotChannel(i,'m1.feedback_speed|rad/s',true);
            }
            activatePlot(0);draw();
            const native=plots[0].draw;plots[0].draw=function(){budgetPaints++;return native()};
            const start=performance.now(),base=budgetSample;
            window.budgetTimer=setInterval(()=>feedBudget(Math.max(0,base+Math.floor((performance.now()-start)/5)-budgetSample)),5);
        }""")
        page.wait_for_timeout(1500)
        cdp=page.context.new_cdp_session(page)
        cdp.send('HeapProfiler.collectGarbage')
        before=cdp.send('Memory.getDOMCounters')
        memory_before=cdp.send('Runtime.getHeapUsage')
        start=page.evaluate('({samples:budgetSample,paints:budgetPaints,time:performance.now()})')
        # Keep a filled 100-second cache under continuous updates, rather than
        # only timing a synchronous seed that never exercises the compositor.
        for _ in range(30):
            page.wait_for_timeout(1000)
        finish=page.evaluate('({samples:budgetSample,paints:budgetPaints,time:performance.now()})')
        cdp.send('HeapProfiler.collectGarbage')
        after=cdp.send('Memory.getDOMCounters')
        memory_after=cdp.send('Runtime.getHeapUsage')
        rate=(finish['paints']-start['paints'])*1000/(finish['time']-start['time'])
        assert 8<=rate<=31,rate
        assert finish['samples']-start['samples']>4500,(start,finish)
        assert after['nodes']<=before['nodes']+100,(before,after)
        assert memory_after['usedSize']<=memory_before['usedSize']+8*1024*1024,(memory_before,memory_after)
        assert memory_after.get('backingStorageSize',0)<=memory_before.get('backingStorageSize',0)+1024*1024,(memory_before,memory_after)
        assert page.evaluate('[...series.values()].every(s=>s.points.length===20001)'), 'Full 100-second source is retained'
        assert page.evaluate('plots.every(p=>p.bounds().hi===latestTime&&p.view.frozen===null)')
        page.click('#stop');assert page.evaluate('budgetCommands')==['Stop']
        page.evaluate('clearInterval(budgetTimer)')
        assert not errors and not crashes,(errors,crashes)
        result={'geometry':geometry,'liveFPS':rate,'samplesAdded':finish['samples']-start['samples'],'DOMBefore':before,'DOMAfter':after,'memoryBefore':memory_before,'memoryAfter':memory_after}
        (OUT/'render-budget.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print('PASS: dense paths preserve extrema/null gaps and raw samples, sparse paths retain all points in bounded continuous batches, 30-second 200 Hz six-canvas soak, <=30 live FPS, stable retained DOM, full 100-second cache and responsive Stop; '+json.dumps(result))
    finally:
        browser.close()
