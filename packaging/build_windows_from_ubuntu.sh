#!/usr/bin/env bash
# Build the current pushed commit on GitHub's native Windows runner.
set -euo pipefail

cd "$(dirname "$0")/.."
root="$(pwd)"
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Run this script from Ubuntu; use build_windows.ps1 on Windows." >&2
    exit 1
fi
if [[ -n "$(git status --porcelain)" ]]; then
    echo "Commit and push your source changes before requesting a Windows build." >&2
    exit 1
fi

if command -v gh >/dev/null 2>&1; then
    gh_bin="$(command -v gh)"
else
    gh_bin="$root/build/gh-cli/usr/bin/gh"
    if [[ ! -x "$gh_bin" ]]; then
        command -v apt-get >/dev/null || { echo "Install GitHub CLI (gh) first." >&2; exit 1; }
        command -v dpkg-deb >/dev/null || { echo "Install dpkg-deb first." >&2; exit 1; }
        mkdir -p "$root/build/gh-cli"
        (
            cd "$root/build/gh-cli"
            apt-get download gh
            packages=(gh_*.deb)
            dpkg-deb -x "${packages[0]}" .
        )
    fi
fi

"$gh_bin" auth status >/dev/null || {
    echo "Sign in first with: $gh_bin auth login -h github.com --web" >&2
    exit 1
}
repo="$("$gh_bin" repo view --json nameWithOwner --jq .nameWithOwner)"
branch="$(git branch --show-current)"
sha="$(git rev-parse HEAD)"
remote_sha="$("$gh_bin" api "repos/$repo/git/ref/heads/$branch" --jq .object.sha)"
if [[ -z "$branch" || "$remote_sha" != "$sha" ]]; then
    echo "Push the current branch before requesting a Windows build." >&2
    exit 1
fi

started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
"$gh_bin" workflow run desktop-release.yml -R "$repo" --ref "$branch"
echo "Waiting for the native Windows build of $sha..."
run_id=""
for attempt in {1..30}; do
    run_id="$("$gh_bin" api \
        "repos/$repo/actions/workflows/desktop-release.yml/runs?event=workflow_dispatch&per_page=20" \
        --jq ".workflow_runs[] | select(.head_sha == \"$sha\" and .created_at >= \"$started_at\") | .id" \
        | head -n 1)"
    [[ -n "$run_id" ]] && break
    sleep 5
done
if [[ -z "$run_id" ]]; then
    echo "The workflow did not appear. Check GitHub Actions for $repo." >&2
    exit 1
fi

"$gh_bin" run watch "$run_id" -R "$repo" --exit-status
artifact_dir="$root/dist/windows-ci-$run_id"
"$gh_bin" run download "$run_id" -R "$repo" -n windows-release -D "$artifact_dir"
archives=("$artifact_dir"/LSS_NCAP_Tool-v*-windows-x86_64.zip)
if [[ ! -f "${archives[0]}" || "${#archives[@]}" -ne 1 ]]; then
    echo "Expected one versioned Windows ZIP in $artifact_dir." >&2
    exit 1
fi
archive="${archives[0]}"
python3 - "$archive" "$root/dist/$(basename "$archive")" <<'PY'
import shutil
import sys
import zipfile
from pathlib import Path

source, target = map(Path, sys.argv[1:])
with zipfile.ZipFile(source) as archive:
    names = set(archive.namelist())
    required = {
        "LSS_NCAP_Tool/LSS_NCAP_Tool.exe",
        "LSS_NCAP_Tool/_internal/lss_ncap/assets/vinfast_app_icon.png",
    }
    if not required <= names or archive.testzip() is not None:
        raise SystemExit("Windows artifact is incomplete or corrupt")
shutil.copyfile(source, target)
print(f"Windows package: {target}")
print("Extract the ZIP on Windows and run LSS_NCAP_Tool.exe with _internal beside it.")
PY
