#!/bin/bash

# Exit on error
set -e

# You may also direclly insall ollama, firefox and webui directly through the dockerfile,
curl -fsSL https://ollama.com/install.sh | sh

apt-get update && apt-get install -y firefox

# Install Miniconda if not already installed
if [ ! -d "/opt/conda" ]; then
    echo "Installing Miniconda..."
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh
    chmod +x miniconda.sh
    ./miniconda.sh -b -p /opt/conda
    rm miniconda.sh
fi

# Add conda to path and initialize
export PATH="/opt/conda/bin:$PATH"
eval "$(/opt/conda/bin/conda shell.bash hook)"

# Create conda environment if it doesn't exist
if ! conda env list | grep -q "llm_env"; then
    echo "Creating conda environment llm_env..."
    conda create -y -n llm_env python=3.11
fi

# Activate environment
echo "Activating conda environment..."
conda activate llm_env

# Install OpenWebUI
echo "Installing OpenWebUI..."
pip install open-webui

# Run OpenWebUI
echo "Starting OpenWebUI server..."
open-webui serve