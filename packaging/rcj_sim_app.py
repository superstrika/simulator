"""Entry point for the standalone executable (PyInstaller).

Run without arguments to open the match setup window; all command-line options
of `python -m rcj_sim` work too (e.g. `rcj-sim examples/match.yaml --headless`).
"""
from rcj_sim.__main__ import main

if __name__ == "__main__":
    main()
