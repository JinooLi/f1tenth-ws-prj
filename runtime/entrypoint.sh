#!/bin/bash
cd /workspace
source install/setup.bash
supervisord -c /etc/supervisord.conf