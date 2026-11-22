from acsi_music.adrc_deploy import run_deployment


def main(args=None):
    run_deployment(automatic=True, args=args)


if __name__ == '__main__':
    main()
