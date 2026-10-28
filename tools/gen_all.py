"""Regenerate every file derived from config/settings.yaml.

    python -m tools.gen_all            # firmware/include/pins.h, firmware/include/config.h, docs/CONFIG_REFERENCE.md
    python -m tools.gen_all --check    # write nothing; exit 1 when any of them is stale

Run it after every edit of config/settings.yaml, then commit the regenerated files.
"""

from __future__ import annotations

import sys
from typing import Optional, Sequence

from tools import gen_config_reference, gen_firmware_config


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(argv if argv is not None else sys.argv[1:])
    return max(gen_firmware_config.main(args), gen_config_reference.main(args))


if __name__ == "__main__":
    sys.exit(main())
