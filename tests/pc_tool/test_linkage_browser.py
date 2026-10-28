"""Real DOM/control-layout checks. No serial device is opened or commanded.

Requires Playwright and a local Edge browser on Windows (or installed Chromium
elsewhere). The HTML itself retains zero external runtime dependencies.
"""
import argparse
import os
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--channel", default="msedge" if os.name == "nt" else "chromium")
args = parser.parse_args()
OUT = ROOT / "build/console-linkage-r100"
OUT.mkdir(exist_ok=True)


with sync_playwright() as p:
    browser = p.chromium.launch(channel=args.channel, headless=True)
    try:
        page = browser.new_page(viewport={"width": 1440, "height": 1100})
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto((ROOT / "pc_tool/index.html").as_uri())
        assert page.title().endswith("R102")
        assert page.locator("#apply").is_disabled()
        assert not page.locator("#runPanel details").get_attribute("open")
        fixture = (ROOT / "build/console-r87/rpc-fixture.bin").read_bytes()[:106]
        page.evaluate("""bytes=>{
            window.testWrites=[];
            window.testConfig=decodeReply(new WireParser().feed(new Uint8Array(bytes))[0]);
            window.send=async command=>{window.testWrites.push(command);return {result:1}};
            window.testReadback=(work,drive=0,esc=false,focus=0)=>{
                const r=structuredClone(window.testConfig);
                r.work=work;r.drive=drive;r.focus=focus;r.profiles=[0,1];
                r.settings.control_config=(r.settings.control_config&~(7|8|16384|32768))|work|
                    (drive===1?8:drive===2?16384:0)|(esc?32768:0);
                r.settings.flags&=~(1<<24);r.settings.control_config|=16;
                connected=true;boardConfig=r;populateControls(r);renderConfig();
            };
            testReadback(0);
        }""", list(fixture))
        assert page.evaluate("testConfig.firmware") == 105
        assert page.locator(".monitor-side").evaluate("e=>Array.from(e.children,x=>x.id)") == ["statusPanel", "runPanel"]

        def values(selector):
            return page.locator(selector + " option").evaluate_all("items=>items.map(x=>x.value)")

        page.select_option("#workMode", "SingleM2")
        assert values("#motorAxis") == values("#axis") == ["M2"]
        assert values("#driveMode") == ["Encoder"]
        assert values("#motorModel") == ["2804", "4015"]
        assert page.locator("#motorModel").input_value() == "4015"
        assert page.locator("#setDrive").is_hidden()
        assert page.locator("#apply").is_disabled()
        assert page.evaluate("testWrites") == []
        page.click("#setWork")
        assert page.evaluate("testWrites") == ["Work:SingleM2"]
        assert page.locator("#apply").is_disabled()  # Await real board acceptance.
        page.evaluate("testReadback(1,0,false,1)")
        page.fill("#target", "12")
        page.click("#apply")
        assert page.evaluate("testWrites.at(-1)") == "M2:Speed:12"
        page.screenshot(path=str(OUT / "linked-m2-desktop.png"), full_page=True)

        # A select cannot choose an absent option. Build options before using
        # a newly read M2 focus; this is not modeled by the native DOM fixture.
        page.evaluate("testReadback(0);testReadback(2,0,false,1)")
        assert page.locator("#axis").input_value() == "M2"
        assert page.locator("#motorAxis").input_value() == "M2"
        assert page.locator("#motorModel").input_value() == "4015"
        page.select_option("#motorAxis", "M1")
        assert page.locator("#axis").input_value() == "M1"
        page.select_option("#axis", "M2")
        assert page.locator("#motorAxis").input_value() == "M2"
        assert page.locator("#motorModel").input_value() == "4015"

        for work, drive, esc, actions in (
            (0, 0, False, ["Speed", "Angle", "Torque"]),
            (0, 1, False, ["Speed"]),
            (0, 2, False, ["Speed", "Throttle"]),
            (0, 2, True, ["Throttle"]),
            (3, 0, False, ["Speed"]),
            (4, 0, False, ["Angle"]),
        ):
            page.evaluate("args=>testReadback(...args)", [work, drive, esc])
            assert values("#action") == actions
            assert page.locator("#follow").is_hidden()
            assert page.locator("#runTargetRow").is_visible()
        for work, follower in ((5, "M2"), (6, "M1")):
            page.evaluate("work=>testReadback(work)", work)
            assert values("#axis") == [follower]
            assert page.locator("#runTargetRow").is_hidden()
            assert page.locator("#follow").is_visible()
            page.click("#follow")
            assert page.evaluate("testWrites.at(-1)") == "Follow"

        # Commands stay reachable beside both the first and last waveform.
        page.evaluate("testReadback(2,0,false,1);setPlotCount(4)")
        page.select_option("#action", "Speed")
        for canvas, target in (("#plot", "12"), ("#plot-4", "18.5")):
            page.locator(canvas).scroll_into_view_if_needed()
            for control in ("#target", "#apply", "#stop"):
                box = page.locator(control).bounding_box()
                assert box["y"] >= 0 and box["y"] + box["height"] <= 1100, (canvas, control, box)
            page.fill("#target", target)
            page.click("#apply")
            assert page.evaluate("testWrites.at(-1)") == "M2:Speed:" + target
            graph = page.locator(canvas).bounding_box()
            assert graph["y"] < 1100 and graph["y"] + graph["height"] > 70
        page.screenshot(path=str(OUT / "run-beside-waveform.png"))

        for width in (1440, 768, 390):
            page.set_viewport_size({"width": width, "height": 1000})
            assert page.evaluate("document.body.scrollWidth<=innerWidth"), width
            assert page.locator("#stop").is_visible()
        page.screenshot(path=str(OUT / "linked-follow-mobile.png"), full_page=True)
        assert not errors, errors
        print("PASS: real Edge DOM: R105 fixture, Status above Run, target commands beside first/last waveform, all seven work modes, dependent options, M2 readback/focus, action/Follow routing, no preview commands, visible Stop and 1440/768/390px layouts")
    finally:
        browser.close()
