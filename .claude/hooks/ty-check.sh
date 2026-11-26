#!/usr/bin/env bash
# Claude Code PostToolUse hook (Edit|Write, .claude/settings.json), beside ruff-check.sh: `ty check` on the .py file
# Claude just wrote, with the ty and the [tool.ty] of its uv project (software/control/ or cad/; any other .py is
# skipped). Only that file: a whole-project check would flag the other files of a change half made - the git
# pre-commit hook checks the whole project. Findings go back to Claude (exit 2). Tooling trouble (no uv, a venv not
# synced yet, a ty config error) never blocks an edit: that exits 0. macOS bash 3.2.
set -u

input="$(cat)"
if command -v jq >/dev/null 2>&1; then
  file="$(printf '%s' "$input" | jq -r '.tool_input.file_path // empty' 2>/dev/null)"
else
  file="$(printf '%s' "$input" | python3 -c 'import json, sys; print(json.load(sys.stdin).get("tool_input", {}).get("file_path", ""))' 2>/dev/null)"
fi
case "${file:-}" in *.py) ;; *) exit 0 ;; esac
[ -f "$file" ] || exit 0

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
case "$file" in
  "$repo"/cad/*)              project="$repo/cad" ;;
  "$repo"/software/control/*) project="$repo/software/control" ;;
  *)                          exit 0 ;;   # in neither uv project, or not in this repo
esac
command -v uv >/dev/null 2>&1 || exit 0

out="$(cd "$project" && uv run --frozen --quiet ty check --force-exclude --output-format concise "$file" 2>&1)"
rc=$?
if [ $rc -eq 1 ]; then   # 1 = findings (a warning too: [tool.ty.terminal] error-on-warning); 0 = clean; else ty failed
  printf 'ty check %s:\n%s\n' "${file#"$repo"/}" "$out" >&2
  exit 2
fi
exit 0
