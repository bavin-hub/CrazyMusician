"""Velocity ADRC deployment with continuous streaming and RC mode selection."""

from acsi_music.adrc_vel_deploy import run_deployment


def main(args=None):
    """Start velocity control without checking modes or issuing arm/mode commands."""
    run_deployment(automatic=False, args=args)


if __name__ == '__main__':
    main()
