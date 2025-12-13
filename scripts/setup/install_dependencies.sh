#!/usr/bin/env bash
# =============================================================================
# TurtleBot4 SLAM Mapping - Dependency Installation Script
# 
# Description: Automated setup script for TurtleBot4 SLAM mapping system
# Author: Rongxuan Zhang
# Course: EECE5550 Mobile Robotics, Northeastern University
# License: MIT
# =============================================================================

set -e
set -u

# =============================================================================
# Configuration
# =============================================================================
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
readonly ROS_DISTRO="${ROS_DISTRO:-jazzy}"
readonly REQUIRED_ROS_VERSION="jazzy"

readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

# =============================================================================
# Logging Functions
# =============================================================================
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

print_header() {
    echo ""
    echo "========================================"
    echo -e "${BLUE}$1${NC}"
    echo "========================================"
    echo ""
}

# =============================================================================
# Utility Functions
# =============================================================================
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

package_installed() {
    dpkg -l "$1" 2>/dev/null | grep -q "^ii"
}

check_ubuntu_version() {
    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        if [[ "${VERSION_ID}" != "24.04" ]]; then
            log_warning "Ubuntu ${VERSION_ID} detected. Recommended: 24.04 LTS"
            read -p "Continue anyway? (y/n): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 0
            fi
        fi
    fi
}

check_ros2() {
    if ! command_exists ros2; then
        log_error "ROS2 is not installed!"
        echo ""
        echo "Please install ROS2 ${REQUIRED_ROS_VERSION} from:"
        echo "https://docs.ros.org/en/${REQUIRED_ROS_VERSION}/Installation.html"
        echo ""
        exit 1
    fi
    
    if [[ "${ROS_DISTRO}" != "${REQUIRED_ROS_VERSION}" ]]; then
        log_error "Wrong ROS2 version detected: ${ROS_DISTRO}"
        log_error "Required: ${REQUIRED_ROS_VERSION}"
        exit 1
    fi
    
    log_success "ROS2 ${ROS_DISTRO} detected"
}

check_python_version() {
    local python_version
    python_version=$(python3 --version 2>&1 | awk '{print $2}')
    local major_minor
    major_minor=$(echo "${python_version}" | cut -d. -f1,2)
    
    if (( $(echo "${major_minor} < 3.10" | bc -l) )); then
        log_error "Python ${python_version} detected. Required: >= 3.10"
        exit 1
    fi
    
    log_success "Python ${python_version} detected"
}

check_network() {
    log_info "Checking network connectivity..."
    if ! ping -c 1 google.com >/dev/null 2>&1; then
        log_error "No internet connection detected"
        exit 1
    fi
    log_success "Network connection OK"
}

# =============================================================================
# Installation Functions
# =============================================================================
update_system() {
    print_header "Updating System Packages"
    
    log_info "Running apt update..."
    sudo apt update || {
        log_error "apt update failed"
        exit 1
    }
    
    log_info "Upgrading existing packages..."
    sudo apt upgrade -y || {
        log_warning "apt upgrade had some issues, but continuing..."
    }
    
    log_success "System update completed"
}

install_ros2_packages() {
    print_header "Installing ROS2 Packages"
    
    local packages=(
        "ros-${ROS_DISTRO}-turtlebot4-navigation"
        "ros-${ROS_DISTRO}-slam-toolbox"
        "ros-${ROS_DISTRO}-foxglove-bridge"
        "ros-${ROS_DISTRO}-nav2-map-server"
        "ros-${ROS_DISTRO}-teleop-twist-keyboard"
    )
    
    for pkg in "${packages[@]}"; do
        if package_installed "${pkg}"; then
            log_info "${pkg} already installed"
        else
            log_info "Installing ${pkg}..."
            if sudo apt install -y "${pkg}"; then
                log_success "${pkg} installed"
            else
                log_error "Failed to install ${pkg}"
                exit 1
            fi
        fi
    done
    
    log_success "All ROS2 packages installed"
}

install_utilities() {
    print_header "Installing Utility Tools"
    
    local utilities=(
        "net-tools"
        "curl"
        "git"
        "vim"
    )
    
    for util in "${utilities[@]}"; do
        if ! package_installed "${util}"; then
            log_info "Installing ${util}..."
            sudo apt install -y "${util}" || log_warning "Failed to install ${util}"
        fi
    done
    
    log_success "Utility tools installed"
}

setup_environment() {
    print_header "Setting Up Environment"
    
    if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
        log_info "Adding ROS2 to .bashrc..."
        echo "" >> ~/.bashrc
        echo "# ROS2 ${ROS_DISTRO}" >> ~/.bashrc
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
        log_success "ROS2 added to .bashrc"
    else
        log_info "ROS2 already configured in .bashrc"
    fi
    
    local scripts_path="${PROJECT_ROOT}/scripts"
    if [[ -d "${scripts_path}" ]]; then
        if ! grep -q "${scripts_path}" ~/.bashrc; then
            log_info "Adding project scripts to PATH..."
            echo "" >> ~/.bashrc
            echo "# TurtleBot4 SLAM Scripts" >> ~/.bashrc
            echo "export PATH=\"\${PATH}:${scripts_path}/launch\"" >> ~/.bashrc
            echo "export PATH=\"\${PATH}:${scripts_path}/mapping\"" >> ~/.bashrc
            echo "export PATH=\"\${PATH}:${scripts_path}/utils\"" >> ~/.bashrc
            log_success "Scripts added to PATH"
        fi
    fi
    
    local maps_dir="${HOME}/turtlebot4_maps"
    if [[ ! -d "${maps_dir}" ]]; then
        log_info "Creating maps directory: ${maps_dir}"
        mkdir -p "${maps_dir}"
        log_success "Maps directory created"
    else
        log_info "Maps directory already exists"
    fi
    
    log_success "Environment setup completed"
}

make_scripts_executable() {
    print_header "Making Scripts Executable"
    
    local script_dirs=(
        "${PROJECT_ROOT}/scripts/launch"
        "${PROJECT_ROOT}/scripts/mapping"
        "${PROJECT_ROOT}/scripts/utils"
        "${PROJECT_ROOT}/scripts/setup"
    )
    
    for dir in "${script_dirs[@]}"; do
        if [[ -d "${dir}" ]]; then
            log_info "Processing directory: ${dir}"
            find "${dir}" -type f -name "*.sh" -exec chmod +x {} \;
        fi
    done
    
    log_success "All scripts are now executable"
}

verify_installation() {
    print_header "Verifying Installation"
    
    local all_ok=true
    
    log_info "Checking installed ROS2 packages..."
    local required_packages=(
        "ros-${ROS_DISTRO}-turtlebot4-navigation"
        "ros-${ROS_DISTRO}-slam-toolbox"
        "ros-${ROS_DISTRO}-foxglove-bridge"
    )
    
    for pkg in "${required_packages[@]}"; do
        if package_installed "${pkg}"; then
            echo "  ✓ ${pkg}"
        else
            echo "  ✗ ${pkg}"
            all_ok=false
        fi
    done
    
    log_info "Checking ROS2 environment..."
    if [[ -n "${ROS_DISTRO}" ]]; then
        echo "  ✓ ROS_DISTRO: ${ROS_DISTRO}"
    else
        echo "  ✗ ROS_DISTRO not set"
        all_ok=false
    fi
    
    log_info "Checking directories..."
    if [[ -d "${HOME}/turtlebot4_maps" ]]; then
        echo "  ✓ Maps directory exists"
    else
        echo "  ✗ Maps directory missing"
        all_ok=false
    fi
    
    echo ""
    if [[ "${all_ok}" == true ]]; then
        log_success "Installation verified successfully!"
        return 0
    else
        log_error "Installation verification found issues"
        return 1
    fi
}

# =============================================================================
# Main Installation Flow
# =============================================================================
main() {
    clear
    
    cat << "EOF"
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║          TurtleBot4 SLAM Mapping System Setup                  ║
║                                                                ║
║  This script will install all required dependencies for        ║
║  the TurtleBot4 SLAM mapping system.                          ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
EOF
    
    echo ""
    log_info "Starting installation process..."
    log_warning "This script requires sudo privileges"
    echo ""
    
    read -p "Continue with installation? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "Installation cancelled by user"
        exit 0
    fi
    
    print_header "Pre-Installation Checks"
    check_ubuntu_version
    check_network
    check_ros2
    check_python_version
    
    update_system
    install_ros2_packages
    install_utilities
    setup_environment
    make_scripts_executable
    
    if verify_installation; then
        print_header "Installation Complete!"
        
        cat << EOF
${GREEN}
╔════════════════════════════════════════════════════════════════╗
║                    Installation Successful!                    ║
╚════════════════════════════════════════════════════════════════╝
${NC}

Next Steps:

1. Restart your terminal or run:
   ${YELLOW}source ~/.bashrc${NC}

2. Verify the system:
   ${YELLOW}./scripts/utils/verify_system.sh${NC}

3. Start mapping:
   ${YELLOW}./scripts/launch/start_all.sh${NC}

4. Open Foxglove Studio:
   ${YELLOW}https://app.foxglove.dev${NC}
   Connect to: ${YELLOW}ws://localhost:8765${NC}

Documentation:
  Quick Reference: ${YELLOW}cat docs/QUICK_REFERENCE.md${NC}
  Full README: ${YELLOW}cat README.md${NC}

Support:
  Issues: ${YELLOW}https://github.com/Jerryzhang258/EECE5550-Final-Project/issues${NC}

Happy Mapping!
EOF
        
        return 0
    else
        log_error "Installation completed with errors"
        echo ""
        echo "Please check the error messages above and try again."
        echo "If problems persist, please open an issue on GitHub."
        return 1
    fi
}

# =============================================================================
# Script Entry Point
# =============================================================================
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
