"""Velocity ADRC deployment with automatic arming and offboard selection."""

from acsi_music.adrc_vel_deploy import run_deployment


def main(args=None):
    """Start the automatic velocity controller."""
    run_deployment(automatic=True, args=args)


if __name__ == '__main__':
    main()
