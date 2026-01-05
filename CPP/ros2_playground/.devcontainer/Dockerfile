FROM ros:jazzy-perception-noble

# Install Node.js and Claude Code
RUN apt-get update && apt-get install -y \
    nodejs \
    npm \
    && npm install -g @anthropic-ai/claude-code \
    && rm -rf /var/lib/apt/lists/*
