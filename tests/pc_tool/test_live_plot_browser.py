"""Exercise the recorded VOFA-style gestures while a 100 Hz source runs.

Uses the real single-file HTML/Canvas in Edge. No serial/motor is accessed.
"""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/live-plot-r101'
OUT.mkdir(exist_ok=True)

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge' if os.name=='nt' else 'chromium',headless=True)
    try:
        for dpr in (1,2):
            context=browser.new_context(viewport={'width':1540,'height':1200},device_scale_factor=dpr)
            page=context.new_page()
            errors=[]
            page.on('pageerror',lambda error:errors.append(str(error)))
            page.goto((ROOT/'pc_tool/index.html').as_uri())
            page.evaluate("""()=>{
                window.liveCommands=[];window.send=async command=>liveCommands.push(command);
                window.liveSample=0;window.livePaints=0;
                const clear=CanvasRenderingContext2D.prototype.clearRect;
                CanvasRenderingContext2D.prototype.clearRect=function(...args){
                    if(this.canvas.id==='plot')++livePaints;return clear.apply(this,args);
                };
                window.feedLive=n=>{for(let i=0;i<n;i++){
                    const t=liveSample++*.01;
                    add({kind:1,epoch:0,ms:t*1000,values:[['live.target','rad/s',50],
                        ['live.feedback','rad/s',50+2*Math.sin(t*9)+.15*Math.sin(t*271)]]});
                }};
                feedLive(2200);addAllPlotChannels();draw();window.liveTimer=setInterval(()=>feedLive(1),10);
            }""")
            assert page.evaluate("plotLayout.relative&&plotLayout.origin===latestTime&&plotView.frozen===null")

            def hit():
                page.locator('#plot').scroll_into_view_if_needed()
                rect=page.locator('#plot').bounding_box()
                layout=page.evaluate('({left:plotLayout.left,right:plotLayout.right,p:plotLayout.panels[0]})')
                x=rect['x']+(layout['left']+layout['right'])/2
                y=rect['y']+(layout['p']['top']+layout['p']['bottom'])/2
                return x,y,layout['right']-layout['left']

            x,y,width=hit();page.mouse.move(x,y);page.mouse.wheel(0,-180)
            page.wait_for_function('plotView.span<10&&plotView.y.size===1')
            assert page.evaluate('plotView.frozen===null&&plotView.end===null&&!paused')
            offset=page.evaluate('plotView.offset');before=page.evaluate('plotLayout.end')
            paints=page.evaluate('livePaints');pixels=page.locator('#plot').evaluate('c=>c.toDataURL()')
            page.wait_for_function('e=>plotLayout.end>e+.15',arg=before)
            assert abs(page.evaluate('plotView.offset')-offset)<1e-7
            assert page.evaluate('livePaints')>paints
            assert page.locator('#plot').evaluate('c=>c.toDataURL()')!=pixels, 'Canvas itself must keep painting new samples after zoom'

            # Incoming samples must not shift the drag origin or end the gesture.
            page.select_option('#plotAxis','xy');x,y,width=hit()
            page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+80,y+35,steps=5)
            held_offset=page.evaluate('plotView.offset');held_end=page.evaluate('plotLayout.end')
            page.wait_for_function('e=>plotLayout.end>e+.15',arg=held_end)
            assert page.evaluate('plotView.drag?.active&&plotView.frozen===null')
            assert abs(page.evaluate('plotView.offset')-held_offset)<1e-7
            page.mouse.move(x+120,y+45,steps=4);page.mouse.up()
            assert page.evaluate('plotView.frozen===null&&plotView.end===null&&!paused')

            # Like the recording: pan left and leave a future margin after t=0.
            page.click('#plotFollow');page.select_option('#plotAxis','x');x,y,width=hit()
            page.mouse.move(x,y);page.mouse.down();page.mouse.move(x-width*.25,y,steps=8);page.mouse.up()
            assert page.evaluate('plotView.offset>0&&plotLayout.end>latestTime&&plotLayout.start<latestTime&&plotView.frozen===null')
            margin=page.evaluate('plotView.offset');end=page.evaluate('plotLayout.end')
            page.wait_for_function('e=>plotLayout.end>e+.15',arg=end)
            assert abs(page.evaluate('plotView.offset')-margin)<1e-7
            if dpr==1:page.locator('#plotPanel').screenshot(path=str(OUT/'live-future-margin.png'))

            # Sub-second scale uses ms; board time remains available explicitly.
            page.evaluate("plotView.span=.3;plotView.offset=.04;draw()")
            assert page.evaluate('plotLayout.timeScale===1000&&plotLayout.relative')
            page.select_option('#plotTime','board')
            assert page.evaluate('!plotLayout.relative&&plotView.frozen===null')
            page.select_option('#plotTime','relative');page.click('#plotReset')

            # Box zoom changes a live view, not a copied snapshot.
            page.select_option('#plotTool','box');page.select_option('#plotAxis','xy');x,y,width=hit()
            page.mouse.move(x-width*.2,y-50);page.mouse.down();page.mouse.move(x+width*.2,y+50,steps=6);page.mouse.up()
            assert page.evaluate('plotView.span<10&&plotView.frozen===null&&plotView.y.size===1')
            end=page.evaluate('plotLayout.end');page.wait_for_function('e=>plotLayout.end>e+.15',arg=end)
            page.select_option('#plotTool','pan');page.click('#plotFollow')

            # Only explicit Pause retains old points across buffer rollover.
            page.click('#plotPause');assert page.evaluate('plotView.frozen!==null')
            frozen_end=page.evaluate('plotLayout.end');frozen_hi=page.evaluate('plotBounds().hi')
            page.evaluate('feedLive(3000)')
            assert page.evaluate('latestTime')>frozen_hi+20
            assert page.evaluate('plotLayout.end')==frozen_end
            assert page.evaluate('plotBounds().hi')==frozen_hi
            assert page.evaluate('[...series.values()].every(s=>s.points.length<=1001&&s.points.at(-1)[0]-s.points.at(0)[0]<=10.000000001)')
            page.click('#plotPause')
            page.wait_for_function('e=>plotView.frozen===null&&plotLayout.end>e+20',arg=frozen_end)

            # Per-canvas pause/zoom must leave the other canvas receiving live.
            page.select_option('#plotCount','2');page.select_option('#plotTarget','2')
            page.locator('#signals [data-channel="live.feedback|rad/s"] input').check()
            page.click('#plotPause')
            fixed=page.evaluate('plots[0].layout.end');second=page.evaluate('plots[1].layout.end')
            page.wait_for_function('e=>plots[1].layout.end>e+.15',arg=second)
            assert page.evaluate('plots[0].layout.end')==fixed
            assert page.evaluate('plots[0].view.frozen!==null&&plots[1].view.frozen===null')
            page.click('#plotFollow');page.click('#pause')
            assert page.evaluate('paused&&plots.slice(0,2).every(p=>p.view.frozen!==null)')
            page.click('#pause');assert page.evaluate('!paused&&plots.slice(0,2).every(p=>p.view.frozen===null)')
            if dpr==1:page.locator('#plotPanel').screenshot(path=str(OUT/'live-two-plots.png'))
            page.evaluate('clearInterval(liveTimer)')
            assert page.evaluate('liveCommands')==[]
            assert not errors,errors
            context.close()
            print(f'PASS: Edge DPR {dpr}: continuously rendered 100 Hz source through wheel/held pan/box, stable relative offsets/future margin/ms/board axes, explicit-only pause, rollover snapshots, independent plots and zero commands')
    finally:
        browser.close()
