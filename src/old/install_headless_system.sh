#!/bin/bash
# Installation script for WRO Obstacle Avoidance System
# This script sets up the headless system to run on startup

set -e  # Exit on any error

echo "==============================================="
echo "WRO Obstacle Avoidance System - Setup Script"
echo "==============================================="

# Check if running as root for service installation
if [[ $EUID -eq 0 ]]; then
    echo "⚠️  Please run this script as the pi user, not as root"
    exit 1
fi

# Check required files
echo "🔍 Checking required files..."
SCRIPT_DIR="/home/pi/wro"
HEADLESS_SCRIPT="$SCRIPT_DIR/lidar_obstacle_avoidance_headless.py"
SERVICE_FILE="$SCRIPT_DIR/wro-obstacle-avoidance.service"
LIDAR_FLASK="$SCRIPT_DIR/lidar_flask.py"

if [[ ! -f "$HEADLESS_SCRIPT" ]]; then
    echo "❌ Headless script not found: $HEADLESS_SCRIPT"
    exit 1
fi

if [[ ! -f "$SERVICE_FILE" ]]; then
    echo "❌ Service file not found: $SERVICE_FILE"
    exit 1
fi

if [[ ! -f "$LIDAR_FLASK" ]]; then
    echo "❌ LiDAR Flask server not found: $LIDAR_FLASK"
    exit 1
fi

echo "✅ All required files found"

# Create log directory
echo "📁 Creating log directory..."
sudo mkdir -p /var/log/wro
sudo chown pi:pi /var/log/wro
echo "✅ Log directory created: /var/log/wro"

# Make script executable
echo "🔧 Making script executable..."
chmod +x "$HEADLESS_SCRIPT"
echo "✅ Script is now executable"

# Install Python dependencies (if needed)
echo "🐍 Checking Python dependencies..."
python3 -c "import RPi.GPIO, requests, flask, threading, logging, subprocess" 2>/dev/null && echo "✅ All Python dependencies available" || {
    echo "⚠️  Some Python dependencies missing. Installing..."
    pip3 install --user RPi.GPIO requests flask
}

# Install systemd service
echo "🔧 Installing systemd service..."
sudo cp "$SERVICE_FILE" /etc/systemd/system/
sudo systemctl daemon-reload
echo "✅ Service installed"

# Enable service
echo "🚀 Enabling service for automatic startup..."
sudo systemctl enable wro-obstacle-avoidance.service
echo "✅ Service enabled for automatic startup"

# Create manual control scripts
echo "📝 Creating control scripts..."

# Start script
cat > "$SCRIPT_DIR/start_obstacle_avoidance.sh" << 'EOF'
#!/bin/bash
echo "🚀 Starting WRO Obstacle Avoidance System..."
sudo systemctl start wro-obstacle-avoidance.service
echo "✅ Service started"
echo "📊 To view logs: sudo journalctl -u wro-obstacle-avoidance.service -f"
EOF

# Stop script
cat > "$SCRIPT_DIR/stop_obstacle_avoidance.sh" << 'EOF'
#!/bin/bash
echo "🛑 Stopping WRO Obstacle Avoidance System..."
sudo systemctl stop wro-obstacle-avoidance.service
echo "✅ Service stopped"
EOF

# Status script
cat > "$SCRIPT_DIR/status_obstacle_avoidance.sh" << 'EOF'
#!/bin/bash
echo "📊 WRO Obstacle Avoidance System Status:"
echo "======================================="
sudo systemctl status wro-obstacle-avoidance.service --no-pager
echo ""
echo "📋 Recent logs:"
echo "==============="
sudo journalctl -u wro-obstacle-avoidance.service --no-pager -n 20
EOF

# View logs script
cat > "$SCRIPT_DIR/logs_obstacle_avoidance.sh" << 'EOF'
#!/bin/bash
echo "📋 WRO Obstacle Avoidance System Logs (Press Ctrl+C to exit):"
echo "============================================================"
sudo journalctl -u wro-obstacle-avoidance.service -f
EOF

# Make control scripts executable
chmod +x "$SCRIPT_DIR"/*.sh

echo "✅ Control scripts created:"
echo "   • start_obstacle_avoidance.sh - Start the service"
echo "   • stop_obstacle_avoidance.sh - Stop the service"
echo "   • status_obstacle_avoidance.sh - Check service status"
echo "   • logs_obstacle_avoidance.sh - View live logs"

# Test script syntax
echo "🧪 Testing script syntax..."
python3 -m py_compile "$HEADLESS_SCRIPT"
echo "✅ Script syntax is valid"

echo ""
echo "==============================================="
echo "🎉 Installation Complete!"
echo "==============================================="
echo ""
echo "📚 Usage:"
echo "1. Start manually: ./start_obstacle_avoidance.sh"
echo "2. Stop manually: ./stop_obstacle_avoidance.sh"
echo "3. Check status: ./status_obstacle_avoidance.sh"
echo "4. View logs: ./logs_obstacle_avoidance.sh"
echo "5. Automatic startup: Will start on next reboot"
echo ""
echo "📁 Log files:"
echo "   • System journal: sudo journalctl -u wro-obstacle-avoidance.service"
echo "   • Application log: /var/log/wro/obstacle_avoidance.log"
echo ""
echo "🔧 Service management:"
echo "   • sudo systemctl start wro-obstacle-avoidance.service"
echo "   • sudo systemctl stop wro-obstacle-avoidance.service"
echo "   • sudo systemctl restart wro-obstacle-avoidance.service"
echo "   • sudo systemctl disable wro-obstacle-avoidance.service (disable autostart)"
echo ""
echo "⚠️  Important: The system will automatically start on next reboot!"
echo "   To test now, run: ./start_obstacle_avoidance.sh"
echo ""
