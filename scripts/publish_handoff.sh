#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

DEFAULT_BRANCH="feature/full-singulator-handoff"
TARGET_BRANCH="${TARGET_BRANCH:-$DEFAULT_BRANCH}"
COMMIT_MESSAGE="${COMMIT_MESSAGE:-Integrate vision, forward-only singulation and roller throat}"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not a Git repository: $PROJECT_ROOT" >&2
  exit 1
fi

if ! git remote get-url origin >/dev/null 2>&1; then
  echo "Remote 'origin' is not configured." >&2
  exit 1
fi

current_branch="$(git branch --show-current)"
if [[ -z "$current_branch" ]]; then
  echo "Detached HEAD is not supported by this script." >&2
  exit 1
fi

if [[ "$current_branch" == "main" || "$current_branch" == "master" ]]; then
  echo "Creating handoff branch: $TARGET_BRANCH"
  git switch -c "$TARGET_BRANCH"
  current_branch="$TARGET_BRANCH"
fi

echo "Repository: $(git remote get-url origin)"
echo "Branch: $current_branch"
echo

echo "Running static checks..."
python3 tools/validate_project.py

git diff --check

if [[ -f scripts/build.sh ]]; then
  echo
  echo "Static checks passed. Build is not run automatically."
  echo "Recommended before publication: ./scripts/build.sh"
fi

echo

git status --short

if git diff --quiet && git diff --cached --quiet && [[ -z "$(git ls-files --others --exclude-standard)" ]]; then
  echo "Nothing to commit. Pushing current branch only."
else
  git add -A
  git commit -m "$COMMIT_MESSAGE"
fi

git push -u origin "$current_branch"

echo
echo "Published branch: $current_branch"
echo "Open GitHub and create a pull request into main after review."
