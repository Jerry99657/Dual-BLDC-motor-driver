"""Render framebuffer captures from run.py; requires Pillow (optional)."""
from pathlib import Path
from PIL import Image, ImageDraw

root = Path(__file__).resolve().parents[2] / "build/ui-validation"
frame_size = 80 * 160 // 8
for name in ("home-forward", "home-reverse", "home-rapid"):
    raw = (root / (name + ".frames")).read_bytes()
    assert len(raw) % frame_size == 0
    frames = []
    for offset in range(0, len(raw), frame_size):
        data = raw[offset:offset + frame_size]
        pixels = bytes(255 if data[((159-x)//8)*80+y] & (1 << ((159-x)%8)) else 0
                       for y in range(80) for x in range(160))
        frames.append(Image.frombytes("L", (160, 80), pixels).resize((640, 320), Image.Resampling.NEAREST))
    frames[0].save(root / (name + ".gif"), save_all=True, append_images=frames[1:],
                   duration=20, loop=0, disposal=2)
    chosen = [frames[min(i, len(frames)-1)] for i in (0, 11, 14, 18, 22, 31, 35, 39, 43, 50)]
    sheet = Image.new("RGB", (640*5, 344*2))
    painter = ImageDraw.Draw(sheet)
    for index, frame in enumerate(chosen):
        x, y = (index%5)*640, (index//5)*344
        sheet.paste(frame, (x, y+24))
        painter.text((x+8, y+5), str((0,11,14,18,22,31,35,39,43,50)[index]*20)+" ms", fill="white")
    sheet.save(root / (name + "-timeline.png"))
for path in root.glob("*.pgm"):
    Image.open(path).resize((640,320), Image.Resampling.NEAREST).save(path.with_suffix(".png"))
for output_name,names in (
    ("r54-preview", ("settings-hso-loop-on", "work-current-marker", "run-hso-start", "run-hso-control",
                     "run-edit-running", "run-4015-draft-range", "run-dual-m2", "run-hand-follow",
                     "run-steps", "settings-save-failed")),
    ("r55-preview", ("home-run-centered", "home-settings-centered", "home-status-centered", "home-about-centered",
                     "run-speed", "run-4015-draft-range", "run-current", "run-hso-control")),
    ("r56-preview", ("run-dual-m2", "run-dual-m2-apply", "run-dual-stopped", "run-speed")),
    ("r57-preview", ("r57-status-menu", "r57-drive", "r57-motion", "r57-hso-limits",
                     "r57-hso-min", "r57-run-rpm", "r57-run-deg", "run-hso-control")),
    ("r58-preview", ("r58-protection", "r58-ibus-editor", "r58-ibus-tripped", "r58-clear-confirm")),
    ("r60-preview", ("r60-system-sixstep", "r60-run-sixstep")),
    ("r63-preview", ("r63-run-throttle", "r60-run-sixstep")),
    ("r72-preview", ("r72-system", "r72-sensorless", "r72-sensorless-disabled",
                     "r72-sixstep", "r72-sixstep-tail")),
):
    sheet = Image.new("RGB", (1280, 344*((len(names)+1)//2)), (20,20,20))
    painter = ImageDraw.Draw(sheet)
    for index,name in enumerate(names):
        x,y = (index%2)*640, (index//2)*344
        sheet.paste(Image.open(root/(name+".png")), (x,y+24))
        painter.text((x+8,y+5), name, fill="white")
    sheet.save(root/(output_name+".png"))
print("Rendered home animations and R54-R72 settings/Run/status framebuffer previews.")
