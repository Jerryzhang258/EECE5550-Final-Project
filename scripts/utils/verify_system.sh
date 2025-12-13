#!/usr/bin/env bash
# =============================================================================
# TurtleBot4 SLAM System Verification Script
# 
# Description: Comprehensive system check for SLAM mapping components
# Author: Rongxuan Zhang
# License: MIT
# =============================================================================

set -u

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

declare -i CHECKS_PASSED=0
declare -i CHECKS_FAILED=0
declare -i CHECKS_WARNING=0

print_header() {
    echo ""
    echo "========================================"
    echo -e "${BLUE}$1${NC}"
    echo "========================================"
    echo ""
}

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((CHECKS_PASSED++))
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ((CHECKS_FAILED++))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    ((CHECKS_WARNING++))
}

check_ros2_environment() {
    print_header "ROS2 Environment"
    
    if [[ -n "${ROS_DISTRO:-}" ]]; then
        check_pass "ROS_DISTRO set to: ${ROS_DISTRO}"
    else
        check_fail "ROS_DISTRO not set"
        echo "  Run: source /opt/ros/jazzy/setup.bash"
    fi
    
    if command -v ros2 >/dev/null 2>&1; then
        check_pass "ros2 command available"
    else
        check_fail "ros2 command not found"
    fi
    
    if ros2 daemon status >/dev/null 2>&1; then
        check_pass "ROS2 daemon running"
    else
        check_warn "ROS2 daemon not running (this is OK)"
    fi
}

check_installed_packages() {
    print_header "Required Packages"
    
    local packages=(
        "ros-${ROS_DISTRO:-jazzy}-turtlebot4-navigation"
        "ros-${ROS_DISTRO:-jazzy}-slam-toolbox"
        "ros-${ROS_DISTRO:-jazzy}-foxglove-bridge"
        "ros-${ROS_DISTRO:-jazzy}-nav2-map-server"
        "ros-${ROS_DISTRO:-jazzy}-teleop-twist-keyboard"
    )
    
    for pkg in "${packages[@]}"; do
        if dpkg -l "${pkg}" 2>/dev/null | grep -q "^ii"; then
            check_pass "${pkg}"
        else
            check_fail "${pkg} not installed"
        fi
    done
}

check_ros2_nodes() {
    print_header "Active ROS2 Nodes"
    
    local nodes
    nodes=$(ros2 node list 2>/dev/null)
    
    if [[ -z "${nodes}" ]]; then
        check_warn "No ROS2 nodes currently running"
        echo "  This is expected if SLAM system is not launched yet"
        return
    fi
    
    local expected_nodes=(
        "slam_toolbox"
        "foxglove"
        "rplidar"
    )
    
    for node in "${expected_nodes[@]}"; do
        if echo "${nodes}" | grep -q "${node}"; then
            check_pass "${node} node detected"
        else
            check_warn "${node} node not found (OK if not launched)"
        fi
    done
    
    echo ""
    echo "Active nodes:"
    echo "${nodes}" | sed 's/^/  /'
}

check_ros2_topics() {
    print_header "ROS2 Topics"
    
    local topics
    topics=$(ros2 topic list 2>/dev/null)
    
    if [[ -z "${topics}" ]]; then
        check_warn "No topics available (system not launched)"
        return
    fi
    
    local critical_topics=(
        "/map"
        "/scan"
        "/tf"
        "/cmd_vel"
    )
    
    for topic in "${critical_topics[@]}"; do
        if echo "${topics}" | grep -q "^${topic}$"; then
            check_pass "${topic} topic exists"
        else
            check_warn "${topic} topic not found (OK if not launched)"
        fi
    done
}

check_topic_rates() {
    print_header "Topic Publishing Rates"
    
    if ! ros2 topic list 2>/dev/null | grep -q "/map"; then
        check_warn "No topics to check (system not launched)"
        return
    fi
    
    echo -n "Checking /map rate... "
    local map_rate
    map_rate=$(timeout 5s ros2 topic hz /map 2>&1 | grep "average rate" || echo "No data")
    if [[ "${map_rate}" != "No data" ]]; then
        check_pass "/map publishing: ${map_rate}"
    else
        check_warn "/map not publishing (may be building initial map)"
    fi
    
    echo -n "Checking /scan rate... "
    local scan_rate
    scan_rate=$(timeout 5s ros2 topic hz /scan 2>&1 | grep "average rate" || echo "No data")
    if [[ "${scan_rate}" != "No data" ]]; then
        check_pass "/scan publishing: ${scan_rate}"
    else
        check_warn "/scan not publishing"
    fi
}

check_network() {
    print_header "Network Configuration"
    
    if command -v netstat >/dev/null 2>&1; then
        if netstat -tuln 2>/dev/null | grep -q ":8765"; then
            check_pass "Foxglove Bridge port 8765 is listening"
        else
            check_warn "Port 8765 not in use (Foxglove not launched)"
        fi
    else
        check_warn "netstat not available, cannot check ports"
    fi
    
    if ping -c 1 -W 2 google.com >/dev/null 2>&1; then
        check_pass "Internet connectivity available"
    else
        check_warn "No internet connection (OK for local operation)"
    fi
}

check_transforms() {
    print_header "TF Transform Tree"
    
    if ! ros2 topic list 2>/dev/null | grep -q "/tf"; then
        check_warn "TF system not active (not launched)"
        return
    fi
    
    echo -n "Checking transform frames... "
    local frames
    frames=$(timeout 5s ros2 run tf2_ros tf2_echo map base_link 2>&1 || echo "")
    
    if echo "${frames}" | grep -q "At time"; then
        check_pass "map -> base_link transform available"
    else
        check_warn "Transform tree not fully established yet"
    fi
}

check_directories() {
    print_header "Directory Structure"
    
    local maps_dir="${HOME}/turtlebot4_maps"
    if [[ -d "${maps_dir}" ]]; then
        check_pass "Maps directory exists: ${maps_dir}"
        
        local map_count
        map_count=$(find "${maps_dir}" -name "*.pgm" 2>/dev/null | wc -l)
        if [[ ${map_count} -gt 0 ]]; then
            echo "  Found ${map_count} saved map(s)"
        fi
    else
        check_warn "Maps directory not found: ${maps_dir}"
        echo "  Run: mkdir -p ${maps_dir}"
    fi
    
    local script_base="${HOME}/EECE5550-Final-Project"
    if [[ -d "${script_base}" ]]; then
        check_pass "Project directory exists"
    else
        check_warn "Project directory not found at expected location"
    fi
}

check_hardware() {
    print_header "Hardware Status"
    
    if command -v lsusb >/dev/null 2>&1; then
        if lsusb | grep -qi "cp210"; then
            check_pass "LiDAR USB interface detected (CP210x)"
        else
            check_warn "LiDAR USB interface not detected (may be using different connection)"
        fi
    fi
    
    local available_space
    available_space=$(df -h "${HOME}" | awk 'NR==2 {print $4}')
    echo "  Available disk space: ${available_space}"
    
    if command -v free >/dev/null 2>&1; then
        local available_mem
        available_mem=$(free -h | awk '/^Mem:/ {print $7}')
        echo "  Available memory: ${available_mem}"
    fi
}

check_lidar_data() {
    print_header "LiDAR Data Quality"
    
    if ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
        check_warn "LiDAR not publishing (system not launched)"
        return
    fi
    
    echo -n "Checking LiDAR scan data... "
    local scan_data
    scan_data=$(timeout 3s ros2 topic echo /scan --once 2>&1 || echo "")
    
    if [[ -n "${scan_data}" ]] && echo "${scan_data}" | grep -q "ranges:"; then
        check_pass "LiDAR publishing valid scan data"
        
        local inf_count
        inf_count=$(echo "${scan_data}" | grep -o "inf" | wc -l)
        if [[ ${inf_count} -gt 100 ]]; then
            check_warn "High number of 'inf' values (${inf_count}) - check LiDAR lens"
        fi
    else
        check_warn "Unable to read LiDAR data"
    fi
}

perform_quick_diagnostics() {
    print_header "Quick Diagnostics"
    
    if ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then
        echo -n "Testing SLAM performance... "
        local slam_cpu
        slam_cpu=$(top -bn1 | grep slam_toolbox | awk '{print $9}' || echo "N/A")
        echo "CPU usage: ${slam_cpu}%"
    fi
    
    if command -v ros2 >/dev/null 2>&1; then
        echo -n "Checking topic statistics... "
        ros2 topic info /map 2>/dev/null | grep "Subscription count:" || echo "Not available"
    fi
}

print_summary() {
    print_header "Verification Summary"
    
    local total_checks=$((CHECKS_PASSED + CHECKS_FAILED + CHECKS_WARNING))
    
    echo "Results:"
    echo -e "  ${GREEN}✓ Passed:${NC}   ${CHECKS_PASSED}"
    echo -e "  ${RED}✗ Failed:${NC}   ${CHECKS_FAILED}"
    echo -e "  ${YELLOW}⚠ Warnings:${NC} ${CHECKS_WARNING}"
    echo "  ─────────────"
    echo "  Total:     ${total_checks}"
    echo ""
    
    if [[ ${CHECKS_FAILED} -eq 0 ]]; then
        if [[ ${CHECKS_WARNING} -eq 0 ]]; then
            echo -e "${GREEN}✓ System Status: EXCELLENT${NC}"
            echo "All components are verified and ready!"
        else
            echo -e "${YELLOW}⚠ System Status: GOOD${NC}"
            echo "System is functional with minor warnings."
        fi
    else
        echo -e "${RED}✗ System Status: ISSUES DETECTED${NC}"
        echo "Please address the failed checks above."
    fi
}

print_recommendations() {
    echo ""
    print_header "Recommendations"
    
    if [[ ${CHECKS_FAILED} -gt 0 ]]; then
        echo "To fix installation issues:"
        echo "  1. Re-run setup: ./scripts/setup/install_dependencies.sh"
        echo "  2. Source ROS2: source /opt/ros/jazzy/setup.bash"
        echo "  3. Install missing packages manually"
        echo ""
    fi
    
    if [[ ${CHECKS_WARNING} -gt 0 ]]; then
        echo "If system is not launched yet:"
        echo "  Start all components: ./scripts/launch/start_all.sh"
        echo "  Or launch individually:"
        echo "    - Foxglove: ./scripts/launch/launch_foxglove.sh"
        echo "    - SLAM: ./scripts/launch/launch_slam.sh"
        echo "    - Teleop: ./scripts/launch/launch_teleop.sh"
        echo ""
    fi
    
    echo "For more information:"
    echo "  Documentation: cat README.md"
    echo "  Quick Reference: cat docs/QUICK_REFERENCE.md"
    echo ""
    
    echo "Foxglove Studio:"
    echo "  URL: https://app.foxglove.dev"
    echo "  Connect: ws://localhost:8765"
    echo ""
}

main() {
    clear
    
    cat << "EOF"
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║          TurtleBot4 SLAM System Verification                   ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
EOF
#!/usr/bin/env bash
# =============================================================================
# TurtleBot4 SLAM System Verification Script
# 
# Description: Comprehensive system check for SLAM mapping components
# Author: Rongxuan Zhang
# License: MIT
# =============================================================================

set -u

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

declare -i CHECKS_PASSED=0
declare -i CHECKS_FAILED=0
declare -i CHECKS_WARNING=0

print_header() {
    echo ""
    echo "========================================"
    echo -e "${BLUE}$1${NC}"
    echo "========================================"
    echo ""
}

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((CHECKS_PASSED++))
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ((CHECKS_FAILED++))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    ((CHECKS_WARNING++))
}

check_ros2_environment() {
    print_header "ROS2 Environment"
    
    if [[ -n "${ROS_DISTRO:-}" ]]; then
        check_pass "ROS_DISTRO set to: ${ROS_DISTRO}"
    else
        check_fail "ROS_DISTRO not set"
        echo "  Run: source /opt/ros/jazzy/setup.bash"
    fi
    
    if command -v ros2 >/dev/null 2>&1; then
        check_pass "ros2 command available"
    else
        check_fail "ros2 command not found"
    fi
    
    if ros2 daemon status >/dev/null 2>&1; then
        check_pass "ROS2 daemon running"
    else
        check_warn "ROS2 daemon not running (this is OK)"
    fi
}

check_installed_packages() {
    print_header "Required Packages"
    
    local packages=(
        "ros-${ROS_DISTRO:-jazzy}-turtlebot4-navigation"
        "ros-${ROS_DISTRO:-jazzy}-slam-toolbox"
        "ros-${ROS_DISTRO:-jazzy}-foxglove-bridge"
        "ros-${ROS_DISTRO:-jazzy}-nav2-map-server"
        "ros-${ROS_DISTRO:-jazzy}-teleop-twist-keyboard"
    )
    
    for pkg in "${packages[@]}"; do
        if dpkg -l "${pkg}" 2>/dev/null | grep -q "^ii"; then
            check_pass "${pkg}"
        else
            check_fail "${pkg} not installed"
        fi
    done
}

check_ros2_nodes() {
    print_header "Active ROS2 Nodes"
    
    local nodes
    nodes=$(ros2 node list 2>/dev/null)
    
    if [[ -z "${nodes}" ]]; then
        check_warn "No ROS2 nodes currently running"
        echo "  This is expected if SLAM system is not launched yet"
        return
    fi
    
    local expected_nodes=(
        "slam_toolbox"
        "foxglove"
        "rplidar"
    )
    
    for node in "${expected_nodes[@]}"; do
        if echo "${nodes}" | grep -q "${node}"; then
            check_pass "${node} node detected"
        else
            check_warn "${node} node not found (OK if not launched)"
        fi
    done
    
    echo ""
    echo "Active nodes:"
    echo "${nodes}" | sed 's/^/  /'
}

check_ros2_topics() {
    print_header "ROS2 Topics"
    
    local topics
    topics=$(ros2 topic list 2>/dev/null)
    
    if [[ -z "${topics}" ]]; then
        check_warn "No topics available (system not launched)"
        return
    fi
    
    local critical_topics=(
        "/map"
        "/scan"
        "/tf"
        "/cmd_vel"
    )
    
    for topic in "${critical_topics[@]}"; do
        if echo "${topics}" | grep -q "^${topic}$"; then
            check_pass "${topic} topic exists"
        else
            check_warn "${topic} topic not found (OK if not launched)"
        fi
    done
}

check_topic_rates() {
    print_header "Topic Publishing Rates"
    
    if ! ros2 topic list 2>/dev/null | grep -q "/map"; then
        check_warn "No topics to check (system not launched)"
        return
    fi
    
    echo -n "Checking /map rate... "
    local map_rate
    map_rate=$(timeout 5s ros2 topic hz /map 2>&1 | grep "average rate" || echo "No data")
    if [[ "${map_rate}" != "No data" ]]; then
        check_pass "/map publishing: ${map_rate}"
    else
        check_warn "/map not publishing (may be building initial map)"
    fi
    
    echo -n "Checking /scan rate... "
    local scan_rate
    scan_rate=$(timeout 5s ros2 topic hz /scan 2>&1 | grep "average rate" || echo "No data")
    if [[ "${scan_rate}" != "No data" ]]; then
        check_pass "/scan publishing: ${scan_rate}"
    else
        check_warn "/scan not publishing"
    fi
}

check_network() {
    print_header "Network Configuration"
    
    if command -v netstat >/dev/null 2>&1; then
        if netstat -tuln 2>/dev/null | grep -q ":8765"; then
            check_pass "Foxglove Bridge port 8765 is listening"
        else
            check_warn "Port 8765 not in use (Foxglove not launched)"
        fi
    else
        check_warn "netstat not available, cannot check ports"
    fi
    
    if ping -c 1 -W 2 google.com >/dev/null 2>&1; then
        check_pass "Internet connectivity available"
    else
        check_warn "No internet connection (OK for local operation)"
    fi
}

check_transforms() {
    print_header "TF Transform Tree"
    
    if ! ros2 topic list 2>/dev/null | grep -q "/tf"; then
        check_warn "TF system not active (not launched)"
        return
    fi
    
    echo -n "Checking transform frames... "
    local frames
    frames=$(timeout 5s ros2 run tf2_ros tf2_echo map base_link 2>&1 || echo "")
    
    if echo "${frames}" | grep -q "At time"; then
        check_pass "map -> base_link transform available"
    else
        check_warn "Transform tree not fully established yet"
    fi
}

check_directories() {
    print_header "Directory Structure"
    
    local maps_dir="${HOME}/turtlebot4_maps"
    if [[ -d "${maps_dir}" ]]; then
        check_pass "Maps directory exists: ${maps_dir}"
        
        local map_count
        map_count=$(find "${maps_dir}" -name "*.pgm" 2>/dev/null | wc -l)
        if [[ ${map_count} -gt 0 ]]; then
            echo "  Found ${map_count} saved map(s)"
        fi
    else
        check_warn "Maps directory not found: ${maps_dir}"
        echo "  Run: mkdir -p ${maps_dir}"
    fi
    
    local script_base="${HOME}/EECE5550-Final-Project"
    if [[ -d "${script_base}" ]]; then
        check_pass "Project directory exists"
    else
        check_warn "Project directory not found at expected location"
    fi
}

check_hardware() {
    print_header "Hardware Status"
    
    if command -v lsusb >/dev/null 2>&1; then
        if lsusb | grep -qi "cp210"; then
            check_pass "LiDAR USB interface detected (CP210x)"
        else
            check_warn "LiDAR USB interface not detected (may be using different connection)"
        fi
    fi
    
    local available_space
    available_space=$(df -h "${HOME}" | awk 'NR==2 {print $4}')
    echo "  Available disk space: ${available_space}"
    
    if command -v free >/dev/null 2>&1; then
        local available_mem
        available_mem=$(free -h | awk '/^Mem:/ {print $7}')
        echo "  Available memory: ${available_mem}"
    fi
}

check_lidar_data() {
    print_header "LiDAR Data Quality"
    
    if ! ros2 topic list 2>/dev/null | grep -q "/scan"; then
        check_warn "LiDAR not publishing (system not launched)"
        return
    fi
    
    echo -n "Checking LiDAR scan data... "
    local scan_data
    scan_data=$(timeout 3s ros2 topic echo /scan --once 2>&1 || echo "")
    
    if [[ -n "${scan_data}" ]] && echo "${scan_data}" | grep -q "ranges:"; then
        check_pass "LiDAR publishing valid scan data"
        
        local inf_count
        inf_count=$(echo "${scan_data}" | grep -o "inf" | wc -l)
        if [[ ${inf_count} -gt 100 ]]; then
            check_warn "High number of 'inf' values (${inf_count}) - check LiDAR lens"
        fi
    else
        check_warn "Unable to read LiDAR data"
    fi
}

perform_quick_diagnostics() {
    print_header "Quick Diagnostics"
    
    if ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then
        echo -n "Testing SLAM performance... "
        local slam_cpu
        slam_cpu=$(top -bn1 | grep slam_toolbox | awk '{print $9}' || echo "N/A")
        echo "CPU usage: ${slam_cpu}%"
    fi
    
    if command -v ros2 >/dev/null 2>&1; then
        echo -n "Checking topic statistics... "
        ros2 topic info /map 2>/dev/null | grep "Subscription count:" || echo "Not available"
    fi
}

print_summary() {
    print_header "Verification Summary"
    
    local total_checks=$((CHECKS_PASSED + CHECKS_FAILED + CHECKS_WARNING))
    
    echo "Results:"
    echo -e "  ${GREEN}✓ Passed:${NC}   ${CHECKS_PASSED}"
    echo -e "  ${RED}✗ Failed:${NC}   ${CHECKS_FAILED}"
    echo -e "  ${YELLOW}⚠ Warnings:${NC} ${CHECKS_WARNING}"
    echo "  ─────────────"
    echo "  Total:     ${total_checks}"
    echo ""
    
    if [[ ${CHECKS_FAILED} -eq 0 ]]; then
        if [[ ${CHECKS_WARNING} -eq 0 ]]; then
            echo -e "${GREEN}✓ System Status: EXCELLENT${NC}"
            echo "All components are verified and ready!"
        else
            echo -e "${YELLOW}⚠ System Status: GOOD${NC}"
            echo "System is functional with minor warnings."
        fi
    else
        echo -e "${RED}✗ System Status: ISSUES DETECTED${NC}"
        echo "Please address the failed checks above."
    fi
}

print_recommendations() {
    echo ""
    print_header "Recommendations"
    
    if [[ ${CHECKS_FAILED} -gt 0 ]]; then
        echo "To fix installation issues:"
        echo "  1. Re-run setup: ./scripts/setup/install_dependencies.sh"
        echo "  2. Source ROS2: source /opt/ros/jazzy/setup.bash"
        echo "  3. Install missing packages manually"
        echo ""
    fi
    
    if [[ ${CHECKS_WARNING} -gt 0 ]]; then
        echo "If system is not launched yet:"
        echo "  Start all components: ./scripts/launch/start_all.sh"
        echo "  Or launch individually:"
        echo "    - Foxglove: ./scripts/launch/launch_foxglove.sh"
        echo "    - SLAM: ./scripts/launch/launch_slam.sh"
        echo "    - Teleop: ./scripts/launch/launch_teleop.sh"
        echo ""
    fi
    
    echo "For more information:"
    echo "  Documentation: cat README.md"
    echo "  Quick Reference: cat docs/QUICK_REFERENCE.md"
    echo ""
    
    echo "Foxglove Studio:"
    echo "  URL: https://app.foxglove.dev"
    echo "  Connect: ws://localhost:8765"
    echo ""
}

main() {
    clear
    
    cat << "EOF"
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║          TurtleBot4 SLAM System Verification                   ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
EOF
    
    echo ""
    echo "Running comprehensive system checks..."
    echo ""
    
    check_ros2_environment
    check_installed_packages
    check_directories
    check_network
    check_hardware
    check_ros2_nodes
    check_ros2_topics
    check_topic_rates
    check_transforms
    check_lidar_data
    perform_quick_diagnostics
    
    print_summary
    print_recommendations
    
    if [[ ${CHECKS_FAILED} -eq 0 ]]; then
        return 0
    else
        return 1
    fi
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
