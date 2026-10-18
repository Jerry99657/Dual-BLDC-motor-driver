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
for name in ("run-steps", "run-speed", "run-edit-stopped", "run-edit-running", "speed-limit", "run-speed-limit"):
    Image.open(root / (name + ".pgm")).resize((640,320), Image.Resampling.NEAREST).save(root / (name + ".png"))
print("Rendered home animations, timelines, Run confirmation states, and step settings.")
