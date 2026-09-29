#!/bin/bash
set -e

# Load environment variables from .env file if it exists
if [ -f .env ]; then
    source .env
fi

# Login to Artifactory if credentials are provided
if [ -n "$ARTIFACTORY_USER" ] && [ -n "$ARTIFACTORY_PASS" ]; then
    echo "Logging in to deloitte.jfrog.io..."
    echo "$ARTIFACTORY_PASS" | docker login -u "$ARTIFACTORY_USER" --password-stdin deloitte.jfrog.io
fi

# Run docker-compose with any additional arguments
docker-compose up "$@"