# ST HSO binary distribution

Extracted unchanged from the user's installed **X-CUBE-MCSDK 6.4.2**:

`Utilities/PC_Software/STMCWB/assets/MotorControlPack/MotorControl.zip`

`manifest.json` records original archive paths and SHA-256 hashes. `libHSO.a`
and `libImpedCorr.a` are the official Cortex-M4 / VFPv4-SP / hard-float libraries.
Their core C sources are **not supplied** in this package. They are not a new
open-source implementation and must not be relabeled under the application's
license. Copyright notices in the supplied headers are retained.

The component headers specify [SLA0044](SLA0044.txt), also available at
https://www.st.com/SLA0044. The installed package's SLA0048 is preserved in
`PACKAGE_LICENSE_SLA0048.txt`. Use is restricted to ST devices and the other
license conditions continue to apply.

Only `fixpmath.c` is compiled here; its CORDIC path supplies the math functions
required by the two archives. `mathlib.h/.c` are the original software fallback
dependency/reference, not linked into the firmware. `FIXPMATH_init()` is not
needed for the hardware CORDIC path. The CORDIC clock/platform bridge lives in
`mycode/st_hso_port`, outside the unchanged vendor files.

The modifiable application integration is `mycode/st_hso.c/.h`, plus the FOC,
protocol and UI application hooks. No ZeST, PolPulse, RsTemp or ST Motor Profiler
library is included. See `docs/ST_HSO_integration.md` for operation and limits.
