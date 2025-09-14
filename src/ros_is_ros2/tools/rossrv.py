"""rossrv command - thin shim over 'ros2 interface'."""

import sys
from ..generic_tool import main_with_completion as generic_main_with_completion, main_generic


def main(args: list[str] | None = None) -> int:
    """Main entry point for rossrv command."""
    return main_generic("rossrv", args)


def main_with_completion():
    """Main entry point with argparse completion support."""
    return generic_main_with_completion("rossrv")


if __name__ == "__main__":
    sys.exit(main_with_completion())
