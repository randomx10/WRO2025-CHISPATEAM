#!/bin/bash
echo "📋 WRO Obstacle Avoidance System Logs (Press Ctrl+C to exit):"
echo "============================================================"
sudo journalctl -u wro-obstacle-avoidance.service -f
