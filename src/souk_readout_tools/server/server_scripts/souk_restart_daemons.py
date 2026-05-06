from souk_readout_tools.server.server_scripts._daemon_cli import main as daemon_main


def main():
    daemon_main(
        "restart_systemd_service.sh",
        "restart",
        default_pipelines=(0, 1),
        default_label="both",
        help_suffix="Use -p 0 or -p 1 for a single pipeline.",
    )


if __name__ == "__main__":
    main()
