#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BASE_BRANCH="${BASE_BRANCH:-main}"
REPORT_DIR="${REPORT_DIR:-release_audit}"
mkdir -p "$REPORT_DIR"

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  echo "ERROR: not a Git repository" >&2
  exit 1
fi

if ! git show-ref --verify --quiet "refs/heads/$BASE_BRANCH" && \
   ! git show-ref --verify --quiet "refs/remotes/origin/$BASE_BRANCH"; then
  echo "ERROR: base branch '$BASE_BRANCH' is unavailable" >&2
  exit 1
fi

BASE_REF="$BASE_BRANCH"
if ! git show-ref --verify --quiet "refs/heads/$BASE_BRANCH"; then
  BASE_REF="origin/$BASE_BRANCH"
fi

printf 'Repository audit\n'
printf 'base=%s\n' "$BASE_REF"
printf 'head=%s\n' "$(git rev-parse --short HEAD)"
printf 'generated=%s\n\n' "$(date --iso-8601=seconds)"

{
  echo -e "branch\tmerge_base\tunique_commits\ttree_equal\tdecision_hint"
  while IFS= read -r branch; do
    [[ "$branch" == "$BASE_REF" ]] && continue
    [[ "$branch" == "origin/$BASE_BRANCH" ]] && continue
    [[ "$branch" == "chore/final-project-packaging" ]] && continue
    [[ "$branch" == "origin/chore/final-project-packaging" ]] && continue

    merge_base="$(git merge-base "$BASE_REF" "$branch" 2>/dev/null || true)"
    unique="$(git rev-list --count "$BASE_REF..$branch" 2>/dev/null || echo '?')"

    tree_equal="no"
    if [[ "$(git rev-parse "$BASE_REF^{tree}")" == "$(git rev-parse "$branch^{tree}")" ]]; then
      tree_equal="yes"
    fi

    hint="review"
    if [[ "$tree_equal" == "yes" ]]; then
      hint="safe-tree-duplicate"
    elif [[ "$unique" == "0" ]]; then
      hint="safe-ancestor"
    fi

    echo -e "${branch}\t${merge_base:0:12}\t${unique}\t${tree_equal}\t${hint}"
  done < <(
    git for-each-ref \
      --format='%(refname:short)' \
      refs/heads refs/remotes/origin \
      | grep -v 'origin/HEAD' \
      | sort -u
  )
} | tee "$REPORT_DIR/branches.tsv"

{
  echo "# Tracked backup/generated candidates"
  git ls-files \
    | grep -E '(^|/)(build|install|log|__pycache__)(/|$)|\.before_|\.backup$|\.bak$|^src_before_|^scripts_before_' \
    || true
} | tee "$REPORT_DIR/tracked_candidates.txt"

{
  echo "# Historical branch commands in jury-facing documentation"
  grep -RInE \
    'git[[:space:]]+switch.*(feat/kty|fix/kty|archive/kty|fix-separator|codex)' \
    README.md docs \
    --include='*.md' \
    || true
} | tee "$REPORT_DIR/historical_branch_references.txt"

{
  echo "# Potentially outdated matrix values in jury-facing documentation"
  grep -RInE \
    'mu2.{0,30}0[.,]8|rows[[:space:]]*==[[:space:]]*14|target_speed_mps.{0,30}56|56 (ROS|Gazebo)' \
    README.md docs \
    --include='*.md' \
    || true
} | tee "$REPORT_DIR/outdated_matrix_references.txt"

cat <<EOF

Audit written to: $REPORT_DIR/
No branches or files were deleted.
Review branches.tsv before running any git push --delete command.
EOF
