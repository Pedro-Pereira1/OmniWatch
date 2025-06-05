#!/bin/bash

distrobox enter humble -- bash -c '
    echo "========== Starting Prosody =========="
    sudo prosodyctl start

    echo "========== Checking Prosody Processes =========="
    ps aux | grep prosody
'