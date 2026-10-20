"""
delay_proxy.py – Script entrypoint for AISR TCP delay proxy.

Run with:
    python scripts/delay_proxy.py --delay-ms 300
    python scripts/delay_proxy.py --delay-ms 500 --mode round-trip
"""

import sys
from pathlib import Path

# Add project root to sys.path
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from src.delay_proxy import main

if __name__ == "__main__":
    main()
