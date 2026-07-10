#!/bin/bash
set -e
pkill -f "app/demo_server.py" || true
pkill -f "scripts/lost_item_visual_api.py" || true
