"""Use the same generated-code checks as every firmware build."""
from pathlib import Path
import subprocess, sys
root=Path(__file__).resolve().parents[2]
subprocess.run([sys.executable, str(root/'scripts/check_cubemx.py')], check=True)
