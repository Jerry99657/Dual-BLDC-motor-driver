"""Record firmware size and verify the official/core source boundary.

python tests/flash_report.py baseline
python tests/flash_report.py ui
python tests/flash_report.py final

Reports and baseline firmware/source copies live under ignored build/size-baseline.
Symbol sizes describe the linked LTO result, not exclusive source-file costs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("label", nargs="?", default="current")
parser.add_argument("--directory", type=Path, default=Path("build/size-baseline"))
args = parser.parse_args()
OUT = ROOT / args.directory
LABEL = args.label


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def protected():
    paths = []
    for directory in ("Drivers", "Middlewares", "ThirdParty", "Core", "USB_Device",
                      "mycode/kk_ui", "mycode/kk_oled", "cmake"):
        paths.extend(p for p in (ROOT / directory).rglob("*") if p.is_file())
    paths += [ROOT / name for name in ("CMakeLists.txt", "CMakePresets.json", "STM32G431XX_FLASH.ld")]
    paths.extend(ROOT.glob("*.ioc"))
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}


def official_commands(mode):
    commands = json.loads((ROOT / "build" / mode / "compile_commands.json").read_text())
    result = {}
    for entry in commands:
        path = Path(entry["file"]).resolve().relative_to(ROOT).as_posix()
        if path.startswith(("Drivers/", "Middlewares/", "ThirdParty/", "Core/", "USB_Device/", "mycode/kk_")):
            result[path] = entry.get("command", entry.get("arguments"))
    return result


def image(mode):
    folder = ROOT / "build" / mode
    elf, binary = folder / "FOC_two.elf", folder / "FOC_two.bin"
    symbols = subprocess.check_output(["arm-none-eabi-nm", "-S", "--size-sort", "--radix=d", str(elf)], text=True)
    functions = []
    for line in symbols.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2] in "tT":
            functions.append({"name": parts[3], "bytes": int(parts[1])})
    sections = subprocess.check_output(["arm-none-eabi-objdump", "-h", str(elf)], text=True).splitlines()
    ram = 0
    for index, line in enumerate(sections[:-1]):
        parts = line.split()
        if len(parts) >= 7 and parts[0].isdigit() and "ALLOC" in sections[index + 1]:
            size, address = int(parts[2], 16), int(parts[3], 16)
            if 0x20000000 <= address < 0x20008000:
                ram += size
    return {"flash_bytes": binary.stat().st_size, "flash_free": 131072 - binary.stat().st_size,
            "ram_allocated": ram, "bin_sha256": sha(binary), "elf_sha256": sha(elf),
            "largest_functions": list(reversed(functions[-50:]))}


OUT.mkdir(parents=True, exist_ok=True)
report = {"protected": protected(), "commands": {}, "images": {}}
for mode in ("Debug", "Release"):
    report["commands"][mode] = official_commands(mode)
    report["images"][mode] = image(mode)
if LABEL == "baseline":
    if (OUT / "baseline.json").exists():
        raise SystemExit("Baseline already exists; do not overwrite the original reference.")
    for mode in ("Debug", "Release"):
        destination = OUT / mode
        destination.mkdir(exist_ok=True)
        for suffix in ("elf", "bin", "map"):
            shutil.copy2(ROOT / "build" / mode / ("FOC_two." + suffix), destination)
    for path in (ROOT / "mycode").glob("*.[ch]"):
        destination = OUT / "mycode" / path.name
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(path, destination)
else:
    baseline = json.loads((OUT / "baseline.json").read_text())
    if baseline["protected"] != report["protected"]:
        changed = [name for name in set(baseline["protected"]) | set(report["protected"])
                   if baseline["protected"].get(name) != report["protected"].get(name)]
        raise SystemExit("Protected files changed: " + ", ".join(sorted(changed)))
    if baseline["commands"] != report["commands"]:
        raise SystemExit("Official/core compilation commands changed.")
    for mode, current in report["images"].items():
        current["saved_bytes"] = baseline["images"][mode]["flash_bytes"] - current["flash_bytes"]
(OUT / (LABEL + ".json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
for mode, current in report["images"].items():
    print(mode, json.dumps({k: v for k, v in current.items() if k != "largest_functions"}))
print("Protected files:", len(report["protected"]),
      "; official/core compile commands", "recorded." if LABEL == "baseline" else "verified.")
