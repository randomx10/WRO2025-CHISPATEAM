./#!/bin/bash
echo "🚀 Starting WRO Obstacle Avoidance System..."
sudo systemctl start wro-obstacle-avoidance.service
echo "✅ Service started"
echo "📊 To view logs: sudo journalctl -u wro-obstacle-avoidance.service -f"
