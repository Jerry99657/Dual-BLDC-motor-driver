"""Real drag/drop and independent canvases, without opening any serial port."""
import argparse
import os
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build/shared-xy-r100"
OUT.mkdir(exist_ok=True)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--channel", default="msedge" if os.name == "nt" else "chromium")
args = parser.parse_args()

with sync_playwright() as p:
    browser = p.chromium.launch(channel=args.channel, headless=True)
    try:
        for dpr in (1, 2):
            # Keep both source and target entirely visible for drag_to. Its
            # preparatory auto-scroll can otherwise move the source coordinates.
            context = browser.new_context(viewport={"width": 1540, "height": 1800}, device_scale_factor=dpr)
            page = context.new_page()
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto((ROOT / "pc_tool/index.html").as_uri())
            page.evaluate("""()=>{
                window.canvasCommands=[];window.send=async command=>{canvasCommands.push(command)};
                clear();for(let i=0;i<1200;i++)add({kind:1,epoch:0,ms:i*20,values:[
                    ['m1.target_speed','rad/s',50],['m1.feedback_speed','rad/s',49+Math.sin(i/20)],
                    ['m2.position','rad',.5*Math.sin(i/30)],['m2.iq','A',.2*Math.sin(i/20)],
                    ['vbus','V',20+.02*Math.sin(i/3)]]});draw();
            }""")
            assert page.evaluate("plots[0].layout.panels.length===1 && plots[0].layout.panels[0].items.length===5"), "Different units must overlay in one XY pair by default"
            assert page.locator("#plotMode, #plotDistribute").count() == 0, "No hidden unit-splitting or fixed allocation controls"
            page.select_option("#plotCount", "3")
            assert page.locator("#plotCanvases canvas").count() == 3
            assert page.evaluate("plots.map(p=>p.selected.size)") == [5, 0, 0]
            source = page.locator('#signals [data-channel="m1.target_speed|rad/s"]')
            source.drag_to(page.locator("#plot-2"))
            assert page.evaluate("[...plots[1].selected]") == ["m1.target_speed|rad/s"]
            assert page.evaluate("plots[0].selected.has('m1.target_speed|rad/s')")
            assert page.locator("#plotTarget").input_value() == "2"
            assert source.locator("input").is_checked()
            source.drag_to(page.locator("#plot-2"))
            assert page.evaluate("plots[1].selected.size") == 1
            page.locator('#signals [data-channel="m1.feedback_speed|rad/s"]').drag_to(page.locator("#plot-2"))
            page.locator('#signals [data-channel="m2.position|rad"]').drag_to(page.locator("#plot-2"))
            assert page.evaluate("plots[1].layout.panels.length===1 && plots[1].layout.panels[0].items.length===3"), "Speed setpoint, feedback and position freely share one XY pair"
            assert page.evaluate("plots[1].layout.panels[0].lo<0 && plots[1].layout.panels[0].hi>50"), "One numerical Y range includes every selected curve"
            page.select_option("#plotTarget", "3")
            page.locator('#signals [data-channel="vbus|V"] input').check()
            page.locator('#signals [data-channel="m2.iq|A"] input').check()
            assert page.evaluate("[...plots[2].selected]") == ["vbus|V", "m2.iq|A"]
            assert page.evaluate("activePlotIndex") == 2
            assert page.evaluate("plots[2].layout.panels.length===1 && plots[2].layout.panels[0].items.length===2"), "Current and bus voltage share a different, independently configured XY pair"
            page.evaluate("plots[2].view.y.set('__xy',{lo:-.1,hi:.1});plots[2].draw()")
            page.locator('#signals [data-channel="m1.target_speed|rad/s"] input').check()
            assert page.evaluate("plots[2].view.y.size===0 && plots[2].layout.panels[0].hi>50"), "Newly added curves are fitted into the shared Y range"
            page.locator('#signals [data-channel="m1.target_speed|rad/s"] input').uncheck()

            # Zoom only canvas 2; canvas 1 and 3 remain live and unmodified.
            page.select_option("#plotAxis-2", "x")
            page.locator("#plot-2").scroll_into_view_if_needed()
            bounds = page.locator("#plot-2").bounding_box()
            layout = page.evaluate("({left:plots[1].layout.left,right:plots[1].layout.right,p:plots[1].layout.panels[0]})")
            page.mouse.move(bounds["x"] + (layout["left"] + layout["right"]) / 2,
                            bounds["y"] + (layout["p"]["top"] + layout["p"]["bottom"]) / 2)
            page.mouse.wheel(0, -160)
            page.wait_for_function("plots[1].view.span<10")
            assert page.evaluate("plots[1].view.frozen===null && plots[1].view.offset<0 && plots[0].view.offset===0 && plots[2].view.offset===0")
            old_offset = page.evaluate("plots[1].view.offset")
            live_end = page.evaluate("plots[1].bounds().hi")
            page.evaluate("""for(let i=1200;i<3500;i++)add({kind:1,epoch:0,ms:i*20,values:[
                ['m1.target_speed','rad/s',50],['m1.feedback_speed','rad/s',49],['m2.position','rad',.2],['m2.iq','A',.1],['vbus','V',20]]});draw()""")
            assert abs(page.evaluate("plots[1].view.offset") - old_offset) < 1e-8
            assert page.evaluate("plots[1].bounds().hi") > live_end
            assert page.evaluate("plots[0].view.end===null && plots[2].view.end===null")
            page.select_option("#plotTarget", "3")
            page.click("#plotZoomIn-3")
            page.locator("#scrub").evaluate("el=>{el.value=0;el.dispatchEvent(new Event('input'))}")
            assert page.evaluate("plots[2].view.offset<0 && plots[2].view.frozen===null && plots[0].view.offset===0")
            assert abs(page.evaluate("plots[1].view.offset") - old_offset) < 1e-8
            page.select_option("#plotCount", "1")
            assert page.locator("#plot-2").is_hidden() and page.locator("#plot-3").is_hidden()
            page.select_option("#plotCount", "3")
            assert page.locator("#plot-2").is_visible() and page.locator("#plot-3").is_visible()
            assert page.evaluate("plots[1].selected.size===3 && plots[2].selected.size===2")
            assert abs(page.evaluate("plots[1].view.offset") - old_offset) < 1e-8
            page.locator('#plotChannels-2 [data-channel="m1.target_speed|rad/s"] button').click()
            assert page.evaluate("plots[1].selected.size===2 && !plots[1].selected.has('m1.target_speed|rad/s') && plots[0].selected.has('m1.target_speed|rad/s')")
            page.click("#plotClear-2")
            assert page.evaluate("plots[1].layout.panels.length===1 && plots[1].layout.panels[0].items.length===0"), "An empty canvas still has one XY pair"

            before = page.evaluate("plots.map(p=>[...p.selected])")
            transfer = page.evaluate_handle("(()=>{const d=new DataTransfer();d.setData('text/plain','<img src=x onerror=alert(1)>');return d})()")
            page.dispatch_event("#plot-3", "drop", {"dataTransfer": transfer})
            assert page.evaluate("plots.map(p=>[...p.selected])") == before
            page.select_option("#plotCount", "4")
            page.select_option("#plotTarget", "4")
            page.click("#plotAddAll")
            assert page.evaluate("plots.slice(0,4).map(p=>p.selected.size)") == [5, 0, 2, 5], "Bulk add overlays only on the selected canvas without redistributing others"
            assert page.evaluate("plots.slice(0,4).every(p=>p.layout.panels.length===1)")
            if dpr == 1:
                page.locator("#plotPanel").screenshot(path=str(OUT / "shared-xy-four-canvases.png"))
            page.click("#pause")
            assert page.evaluate("paused && plots.slice(0,4).every(p=>p.view.frozen!==null)")
            page.click("#plotFollow-2")
            assert page.evaluate("!paused && plots[1].view.frozen===null && plots[0].view.frozen!==null && plots[2].view.frozen!==null")
            page.select_option("#plotCount", "6")
            assert page.locator("#plotAdd").is_disabled()
            assert page.locator("#plotCanvases canvas").count() == 6
            page.select_option("#plotColumns", "1")
            assert not page.locator("#plotCanvases").evaluate("el=>el.classList.contains('two-columns')")
            page.select_option("#plotColumns", "2")
            for width in (1540, 768, 390):
                page.set_viewport_size({"width": width, "height": 1000})
                page.evaluate("draw()")
                assert page.evaluate("document.body.scrollWidth<=innerWidth"), width
                assert page.evaluate("plots.slice(0,visiblePlots).every(p=>p.layout.width>100&&Math.abs(p.control('plot').width/p.control('plot').getBoundingClientRect().width-devicePixelRatio)<.01)")
            page.select_option("#plotTarget", "6")
            page.locator('#signals [data-channel="vbus|V"] input').check()
            assert page.evaluate("plots[5].selected.has('vbus|V')"), "Touch/keyboard binding fallback works without drag/drop"
            page.select_option("#plotCount", "1")
            assert page.locator("#plotRemove").is_disabled()
            assert page.evaluate("activePlotIndex===0")
            ids = page.locator("[id]").evaluate_all("items=>items.map(x=>x.id)")
            assert len(ids) == len(set(ids)), "Cloned controls require unique DOM IDs"
            assert page.evaluate("canvasCommands") == []
            assert not errors, errors
            context.close()
            print(f"PASS: Edge DPR {dpr}: one XY pair per canvas, mixed-unit drag/overlay, automatic shared-Y refit, copy/dedup, selected-canvas checkboxes, independent zoom/snapshots/CSV, hide/restore/removal, invalid drops, bulk-add isolation, 1..6 plots, unique IDs, pause/follow isolation and responsive layouts")
    finally:
        browser.close()
