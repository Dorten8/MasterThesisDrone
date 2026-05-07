#!/bin/bash

echo "🧹 Cleaning up Copilot Chat broken state..."

# Close VS Code if running
pkill -f "code" 2>/dev/null && sleep 2

# Remove corrupted Copilot Chat caches (local)
echo "Removing local Copilot Chat data..."
rm -rf ~/.config/Code/User/globalStorage/github.copilot-chat 2>/dev/null
rm -rf ~/.config/Code/User/workspaceStorage 2>/dev/null

# If working on remote SSH, also clean there
if [ ! -z "$SSH_CONNECTION" ]; then
    echo "Removing remote Copilot Chat data..."
    rm -rf ~/.vscode-server/data/User/globalStorage/github.copilot-chat 2>/dev/null
    rm -rf ~/.vscode-server/data/User/workspaceStorage 2>/dev/null
fi

# Remove any project-level Copilot files (if in a project folder)
if [ -d ".vscode" ]; then
    echo "Removing project-level Copilot files..."
    rm -rf .vscode/github-copilot-chat* 2>/dev/null
fi

echo "✅ Cleanup complete!"
echo "👉 You can now reopen VS Code."