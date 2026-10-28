"""Actual Canvas glyph bounds and mouse interaction, using synthetic telemetry.

No serial port is opened. Requires Playwright plus local Edge on Windows.
"""
import argparse
import os
from pathlib import Path
import re

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build/shared-xy-r100"
OUT.mkdir(exist_ok=True)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--channel", default="msedge" if os.name == "nt" else "chromium")
args = parser.parse_args()

SEED = """()=>{
    clear();let random=1;
    for(let i=0;i<2200;i++){
        random=(1664525*random+1013904223)>>>0;
        add({kind:1,epoch:0,ms:1804000+i*20,values:[
            ['m2.target_speed','rad/s',0],['m2.feedback_speed','rad/s',0],
            ['m2.target_position','rad',540.2],['m2.position','rad',540.2],
            ['m2.iq','A',0],['vbus','V',11.69+.06*(random/4294967296-.5)]]});
    }
    draw();
}"""


def intersects(a, b):
    return min(a[2], b[2]) > max(a[0], b[0]) + .1 and min(a[3], b[3]) > max(a[1], b[1]) + .1


with sync_playwright() as p:
    browser = p.chromium.launch(channel=args.channel, headless=True)
    try:
        for dpr in (1, 2):
            context = browser.new_context(viewport={"width": 1540, "height": 1100}, device_scale_factor=dpr)
            page = context.new_page()
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto((ROOT / "pc_tool/index.html").as_uri())
            page.evaluate("""()=>{
                window.chartCommands=[];window.send=async command=>{chartCommands.push(command)};
                const original=CanvasRenderingContext2D.prototype.fillText;
                CanvasRenderingContext2D.prototype.fillText=function(text,x,y,...args){
                    if(window.captureInk&&this.canvas.id==='plot'){
                        const m=this.measureText(String(text));
                        window.plotInk.push({text:String(text),box:[x-m.actualBoundingBoxLeft,y-m.actualBoundingBoxAscent,x+m.actualBoundingBoxRight,y+m.actualBoundingBoxDescent]});
                    }
                    return original.call(this,text,x,y,...args);
                };
            }""")
            page.evaluate(SEED)

            def check_ink():
                ink = page.evaluate("""()=>{window.plotInk=[];window.captureInk=true;draw();window.captureInk=false;return plotInk}""")
                units = [x for x in ink if x["text"] in ("rad/s", "rad", "A", "V", "数值 · 单位见通道", "数值")]
                numbers = [x for x in ink if re_number(x["text"])]
                assert len(units) == 1, "Mixed units always have one shared Y header"
                assert page.evaluate("plotLayout.panels.length===1 && plotLayout.panels[0].items.length===6")
                size = page.evaluate("({width:plotLayout.width,height:plotLayout.height})")
                for n in numbers:
                    assert n["box"][0] >= -.5 and n["box"][2] <= size["width"] + .5, n
                    assert n["box"][1] >= -.5 and n["box"][3] <= size["height"] + .5, n
                    assert all(not intersects(u["box"], n["box"]) for u in units), (units, n)
                for index, n in enumerate(numbers):
                    assert all(not intersects(n["box"], other["box"]) for other in numbers[index + 1:]), (n, numbers)

            def re_number(text):
                return bool(re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?", text))

            for width in (1540, 768, 390):
                page.set_viewport_size({"width": width, "height": 1100})
                check_ink()
                assert page.evaluate("document.body.scrollWidth<=innerWidth")
                assert page.evaluate("Math.abs($('plot').width/$('plot').getBoundingClientRect().width-devicePixelRatio)<.01")
            # Large DC offsets with small excursions must retain distinct,
            # in-bounds scientific ticks rather than identical rounded labels.
            page.evaluate("plotView.y.set('__xy',{lo:16777217,hi:16777217.01});plotView.span=.002;draw()")
            check_ink()
            page.evaluate("followPlot(true)")
            page.set_viewport_size({"width": 1540, "height": 1100})
            page.evaluate("draw()")
            if dpr == 1:
                page.locator("#plotPanel").screenshot(path=str(OUT / "plot-fixed-units.png"))

            def hit_position():
                page.locator("#plot").scroll_into_view_if_needed()
                bounds = page.locator("#plot").bounding_box()
                layout = page.evaluate("({left:plotLayout.left,right:plotLayout.right,panel:plotLayout.panels[0],bottom:plotLayout.bottom})")
                x = bounds["x"] + (layout["left"] + layout["right"]) / 2
                y = bounds["y"] + (layout["panel"]["top"] + layout["panel"]["bottom"]) / 2
                return bounds, layout, x, y

            bounds, layout, x, y = hit_position()
            page.select_option("#plotAxis", "x")
            page.mouse.move(x, y)
            page.mouse.wheel(0, -200)
            page.wait_for_function("plotView.span<10 && plotView.frozen===null && plotView.offset<0")
            assert page.evaluate("plotView.y.size") == 0
            span = page.evaluate("plotView.span")
            old_offset = page.evaluate("plotView.offset")
            page.mouse.down()
            page.mouse.move(x + 100, y, steps=8)
            page.mouse.up()
            page.wait_for_function("plotView.drag===null")
            assert page.evaluate("plotView.offset") < old_offset
            assert page.evaluate("plotView.span") == span
            live_last = page.evaluate("plotBounds().hi")
            live_offset = page.evaluate("plotView.offset")
            page.evaluate("for(let i=0;i<2300;i++)add({kind:1,epoch:0,ms:1848000+i*20,values:[['vbus','V',20]]});draw()")
            assert page.evaluate("plotBounds().hi") > live_last
            assert abs(page.evaluate("plotView.offset") - live_offset) < 1e-8
            assert page.evaluate("plotView.frozen===null && plotView.end===null")
            page.click("#plotFollow")
            assert page.evaluate("plotView.frozen===null && plotView.end===null")

            page.evaluate(SEED)
            bounds, layout, x, y = hit_position()
            page.mouse.move(bounds["x"] + layout["left"] - 12, y)
            page.mouse.wheel(0, -160)
            page.wait_for_function("plotView.y.size===1")
            assert page.evaluate("plotView.end===null && plotView.frozen===null && plotView.span===10")
            page.click("#plotReset")
            page.select_option("#plotAxis", "xy")
            page.select_option("#plotTool", "box")
            bounds, layout, x, y = hit_position()
            page.mouse.move(x - 160, y - 25)
            page.mouse.down()
            page.mouse.move(x + 160, y + 25, steps=10)
            page.mouse.up()
            assert page.evaluate("plotView.span<10 && plotView.y.size===1")
            page.click("#plotReset")
            bounds, layout, x, y = hit_position()
            page.mouse.move(x, y)
            page.wait_for_function("!$('plotTip').hidden")
            assert "m2.target_speed" in page.locator("#plotTip").inner_text()
            page.mouse.dblclick(x, y)
            assert page.evaluate("plotView.end===null && plotView.y.size===0")
            assert page.evaluate("plotLayout.panels.length") == 1
            if dpr == 1:
                page.locator("#plotPanel").screenshot(path=str(OUT / "plot-overlay.png"))
            page.click("#plotZoomIn")
            page.evaluate("replaying=true;replayEnd=latestTime;replayStart=latestTime-20;draw()")
            page.locator("#scrub").evaluate("el=>{el.value=0;el.dispatchEvent(new Event('input'))}")
            assert page.evaluate("replaying && plotLayout.end<latestTime && $('scrub').value==='0'")
            page.click("#plotFollow")
            assert page.evaluate("replaying && plotView.end===null"), "Chart follow must not exit protected CSV replay"
            assert page.evaluate("chartCommands") == []
            assert not errors, errors
            context.close()
            print(f"PASS: Edge DPR {dpr}: one shared XY pair, real relative-time/unit/tick ink bounds at 1540/768/390px, live wheel/axis-only zoom/pan/box while receiving, hover, reset, overlay, CSV scrub and zero motor commands")
    finally:
        browser.close()
