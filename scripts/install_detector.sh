#!/bin/bash

git -C mad_detector pull || git clone https://github.com/KaiL4eK/mad_detector.git

# Install models in local folder
cd mad_detector && ./get_models.sh
