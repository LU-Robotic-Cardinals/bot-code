#!/bin/bash

# Check for uncommitted changes (both staged and unstaged)
git diff --quiet --exit-code HEAD > /dev/null 2>&1

# Exit if there are no changes
if [ $? -eq 0 ]; then
  echo "No changes detected. Skipping commit."
  exit 0
fi

# Get a list of changed files
changed_files=$(git diff --cached --name-only)

# Print a condensed list of changes (if any)
if [[ ! -z "$changed_files" ]]; then
  echo "Changes detected:"
  echo "$changed_files"
fi

# Stage all unstaged changes
git add -A

# Check for uncommitted changes
git diff --cached --name-only > /dev/null 2>&1
# Commit with timestamp message
git commit -m "$(date +'%Y-%m-%d %H:%M:%S')"

echo "Committed changes with timestamp!"
