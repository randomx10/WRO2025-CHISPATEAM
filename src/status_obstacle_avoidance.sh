#!/bin/bash
echo "📊 WRO Obstacle Avoidance System Status:"
echo "======================================="
sudo systemctl status wro-obstacle-avoidance.service --no-pager
echo ""
echo "📋 Recent logs:"
echo "==============="
sudo journalctl -u wro-obstacle-avoidance.service --no-pager -n 20
