#!/bin/bash
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

ok()   { echo -e "  ${GREEN}[OK]${NC} $1"; }
warn() { echo -e "  ${YELLOW}[!]${NC} $1"; }
fail() { echo -e "  ${RED}[X]${NC} $1"; }
info() { echo -e "  ${CYAN}[*]${NC} $1"; }

echo ""
echo "============================================"
echo "  Physical AI Championship — Setup Script"
echo "============================================"
echo ""

# ─── 1. Check OS ───
echo "1. Checking OS..."
if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    ok "$NAME $VERSION_ID ($(uname -m))"
    if [[ "$VERSION_ID" == "24.04" ]]; then
        fail "Ubuntu 24.04 is not supported. Please use 20.04 or 22.04."
        exit 1
    fi
else
    warn "Cannot detect OS, continuing anyway"
fi

# ─── 2. System packages ───
echo ""
echo "2. Checking system packages..."

PACKAGES=(
    build-essential
    cmake
    git
    libopencv-dev
    libglfw3-dev
    libboost-program-options-dev
    libyaml-cpp-dev
    libfmt-dev
)

MISSING=()
for pkg in "${PACKAGES[@]}"; do
    if dpkg -s "$pkg" &>/dev/null; then
        ok "$pkg"
    else
        fail "$pkg — not installed"
        MISSING+=("$pkg")
    fi
done

if [ ${#MISSING[@]} -gt 0 ]; then
    echo ""
    info "Installing missing packages: ${MISSING[*]}"
    sudo apt update -qq
    sudo apt install -y "${MISSING[@]}"
    ok "All packages installed"
fi

# ─── 3. unitree_sdk2 ───
echo ""
echo "3. Checking unitree_sdk2..."

SDK_FOUND=0
for path in /opt/unitree_robotics /usr/local; do
    if [ -f "$path/include/unitree/robot/channel/channel_publisher.hpp" ] || \
       [ -f "$path/lib/libunitree_sdk2.a" ] || \
       [ -d "$path/include/unitree" ]; then
        ok "unitree_sdk2 found in $path"
        SDK_FOUND=1
        break
    fi
done

if [ $SDK_FOUND -eq 0 ]; then
    # Check via cmake
    if pkg-config --exists unitree_sdk2 2>/dev/null; then
        ok "unitree_sdk2 found via pkg-config"
        SDK_FOUND=1
    fi
fi

if [ $SDK_FOUND -eq 0 ]; then
    warn "unitree_sdk2 not found, installing..."
    TMPDIR=$(mktemp -d)
    cd "$TMPDIR"
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
    cd unitree_sdk2
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
    sudo make install -j$(nproc)
    cd "$ROOT_DIR"
    rm -rf "$TMPDIR"
    ok "unitree_sdk2 installed to /opt/unitree_robotics"
fi

# ─── 4. Build simulator ───
echo ""
echo "4. Building simulator..."
info "MuJoCo will be downloaded automatically if needed"

if [ -f "$ROOT_DIR/simulate/build/unitree_mujoco" ]; then
    warn "Simulator binary already exists"
    read -p "  Delete build and rebuild? [y/N]: " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        rm -rf "$ROOT_DIR/simulate/build"
        info "Build directory removed, rebuilding..."
    else
        ok "Skipping simulator build"
    fi
fi

if [ ! -f "$ROOT_DIR/simulate/build/unitree_mujoco" ]; then
    cd "$ROOT_DIR/simulate"
    mkdir -p build && cd build
    cmake .. 2>&1 | grep -E "MuJoCo|error|warning" || true
    make -j$(nproc) 2>&1 | tail -5

    if [ -f unitree_mujoco ]; then
        ok "Simulator built: simulate/build/unitree_mujoco"
    else
        fail "Simulator build failed!"
        exit 1
    fi
fi

# ─── 5. Build controller ───
echo ""
echo "5. Building controller..."

if [ -f "$ROOT_DIR/g1_ctrl/build/g1_ctrl" ]; then
    warn "Controller binary already exists"
    read -p "  Delete build and rebuild? [y/N]: " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        rm -rf "$ROOT_DIR/g1_ctrl/build"
        info "Build directory removed, rebuilding..."
    else
        ok "Skipping controller build"
    fi
fi

if [ ! -f "$ROOT_DIR/g1_ctrl/build/g1_ctrl" ]; then
    cd "$ROOT_DIR/g1_ctrl"
    mkdir -p build && cd build
    cmake .. 2>&1 | grep -E "onnxruntime|error|warning" || true
    make -j$(nproc) 2>&1 | tail -5

    if [ -f g1_ctrl ]; then
        ok "Controller built: g1_ctrl/build/g1_ctrl"
    else
        fail "Controller build failed!"
        exit 1
    fi
fi

cd "$ROOT_DIR"

# ─── Done ───
echo ""
echo "============================================"
echo -e "  ${GREEN}Setup complete!${NC}"
echo "============================================"
echo ""
echo "  To run (single command, fully autonomous):"
echo ""
echo -e "    ${CYAN}./simulate/build/unitree_mujoco${NC}"
echo ""
echo "  The robot will automatically:"
echo "    1. Stand up (FixStand)"
echo "    2. Enter walking mode (Velocity)"
echo "    3. Search for the red cube"
echo "    4. Approach and stop at ≤ 0.5m"
echo ""
echo -e "  ${YELLOW}Tip:${NC} Cyclic testing mode is available!"
echo "  In simulate/config.yaml set:"
echo ""
echo "    auto_respawn: 1"
echo "    auto_respawn_delay: 2.0"
echo ""
echo "  Robot and cube will respawn to new random"
echo "  positions after each successful stop."
echo ""
