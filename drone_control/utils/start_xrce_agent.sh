#!/bin/bash
# start_xrce_agent.sh
# Runs the MicroXRCE-DDS Agent in a detached screen session for easy inspection.

AGENT_CMD="MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600"

# Kill existing agent screen session if it exists
screen -S xrce_agent -X quit 2>/dev/null

echo "🚀 Starting MicroXRCE-DDS Agent in screen session 'xrce_agent'..."
screen -dmS xrce_agent bash -c "$AGENT_CMD; exec bash"

echo "--------------------------------------------------------"
echo "✅ Agent is running in the background."
echo "   To inspect the agent logs, type: screen -r xrce_agent"
echo "   To detach and keep it running, press: Ctrl+A then D"
echo "--------------------------------------------------------"
