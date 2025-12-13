#!/usr/bin/env bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_msg() {
    echo -e "${2}${1}${NC}"
}

clear
echo "=========================================="
print_msg "TurtleBot4 SLAM Mapping System" "$BLUE"
echo "=========================================="
echo ""
print_msg "This will launch 3 terminals:" "$YELLOW"
echo "  1. Foxglove Bridge (visualization)"
echo "  2. SLAM Toolbox (mapping)"
echo "  3. Keyboard Teleop (robot control)"
echo ""
print_msg "Next Steps After Launch:" "$GREEN"
echo "  1. Open browser: https://app.foxglove.dev"
echo "  2. Connect to: ws://localhost:8765"
echo "  3. Add 3D panel, set Fixed frame to 'map'"
echo "  4. Check topics: /map, /scan, /tf"
echo "  5. Use keyboard to drive robot"
echo "  6. Run scripts/mapping/save_map.sh when done"
echo ""
read -p "Press Enter to launch..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v gnome-terminal >/dev/null 2>&1; then
    TERM_CMD="gnome-terminal"
elif command -v xterm >/dev/null 2>&1; then
    TERM_CMD="xterm -e"
elif command -v konsole >/dev/null 2>&1; then
    TERM_CMD="konsole -e"
else
    print_msg "ERROR: No terminal emulator found!" "$RED"
    print_msg "Please launch manually in 3 separate terminals:" "$YELLOW"
    echo "  Terminal 1: ${SCRIPT_DIR}/launch_foxglove.sh"
    echo "  Terminal 2: ${SCRIPT_DIR}/launch_slam.sh"
    echo "  Terminal 3: ${SCRIPT_DIR}/launch_teleop.sh"
    exit 1
fi

print_msg "Launching Foxglove Bridge..." "$YELLOW"
$TERM_CMD bash -c "${SCRIPT_DIR}/launch_foxglove.sh; exec bash" &
sleep 2

print_msg "Launching SLAM Toolbox..." "$YELLOW"
$TERM_CMD bash -c "${SCRIPT_DIR}/launch_slam.sh; exec bash" &
sleep 3

print_msg "Launching Keyboard Teleop..." "$YELLOW"
$TERM_CMD bash -c "${SCRIPT_DIR}/launch_teleop.sh; exec bash" &

echo ""
print_msg "✓ All systems launched!" "$GREEN"
echo ""
print_msg "Remember:" "$BLUE"
echo "  - Move slowly (0.2-0.3 m/s)"
echo "  - Drive along walls first"
echo "  - Return to starting point for loop closure"
echo "  - Save map with: scripts/mapping/save_map.sh"
echo ""
