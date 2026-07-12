#!/bin/bash

# ========================================
# Quick Experiment Mode Scripts
# ========================================

# 색상 정의
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Setting EXPERIMENT_MODE environment variable...${NC}"

case "$1" in
    "static"|"baseline"|"0")
        export EXPERIMENT_MODE=0
        echo -e "${GREEN}Mode 0: Static Lattice Planning (Baseline)${NC}"
        ;;
    "va"|"mds"|"1")
        export EXPERIMENT_MODE=1
        echo -e "${GREEN}Mode 1: VA Lattice Planning with MDS${NC}"
        ;;
    "fusion"|"2")
        export EXPERIMENT_MODE=2
        echo -e "${GREEN}Mode 2: VA + Fusion Costmap${NC}"
        ;;
    "original"|"dynamic"|"3")
        export EXPERIMENT_MODE=3
        echo -e "${GREEN}Mode 3: Original Dynamic LD${NC}"
        ;;
    *)
        echo "Usage: source $0 [static|va|fusion|original] or [0|1|2|3]"
        echo "Example: source $0 va"
        echo "         source $0 1"
        return 1
        ;;
esac

echo -e "${BLUE}EXPERIMENT_MODE = $EXPERIMENT_MODE${NC}"
echo "Now you can run: rosrun PlanningControl lattice_test_node"