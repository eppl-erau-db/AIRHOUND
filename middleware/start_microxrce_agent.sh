#!/bin/bash
# Helper script to start MicroXRCE-DDS Agent
# Works with both system-wide and local installations

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}Starting MicroXRCE-DDS Agent...${NC}"

# Check if agent is installed system-wide
if command -v MicroXRCEAgent &> /dev/null; then
    echo -e "${GREEN}Using system-wide MicroXRCEAgent${NC}"
    MicroXRCEAgent udp4 -p 8888
else
    # Try local build
    LOCAL_AGENT="$HOME/.local/airhound_tools/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
    if [ -f "$LOCAL_AGENT" ]; then
        echo -e "${YELLOW}Using local MicroXRCEAgent build${NC}"
        echo -e "${YELLOW}(To install system-wide, run: cd ~/.local/airhound_tools/Micro-XRCE-DDS-Agent/build && sudo make install)${NC}"
        "$LOCAL_AGENT" udp4 -p 8888
    else
        echo -e "${RED}ERROR: MicroXRCEAgent not found!${NC}"
        echo -e "Please run: ${GREEN}./airhound/setup_dependencies.sh${NC}"
        exit 1
    fi
fi
