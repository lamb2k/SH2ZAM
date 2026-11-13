#!/usr/bin/env bash
# Publish the SH2ZAM branch to a NEW standalone repository, preserving ancestry.
# Requires Git and an authenticated GitHub CLI account with access to lamb2k.
# Does not rename, push to, or delete the existing Dreamcast fork.
set -euo pipefail
export GH_HOST=github.com

target="lamb2k/SH2ZAM"
source_repo="https://github.com/lamb2k/sh4zam.git"
source_branch="feature/sh2zam-saturn"
prototype_commit="523f789ce7110d6dce5d8586fa157b2bf03df8bb"
workdir="${1:-SH2ZAM}"

for command_name in git gh; do
    command -v "$command_name" >/dev/null 2>&1 || {
        printf 'Required command not found: %s\n' "$command_name" >&2
        exit 1
    }
done
gh auth status --hostname github.com >/dev/null
if [[ "$(gh api --hostname github.com user --jq .login)" != "lamb2k" ]]; then
    printf 'Authenticate GitHub CLI as lamb2k before running this script.\n' >&2
    exit 1
fi
if [[ "$workdir" == -* || -e "$workdir" ]]; then
    printf 'Choose a new, non-option directory path; refusing: %s\n' "$workdir" >&2
    exit 1
fi
if gh repo view "$target" --json name >/dev/null 2>&1; then
    printf '%s already exists; refusing to overwrite it.\n' "$target" >&2
    exit 1
fi

git clone --single-branch --no-tags --branch "$source_branch" "$source_repo" "$workdir"
cd "$workdir"
git merge-base --is-ancestor "$prototype_commit" HEAD
grep -q '^# SH2ZAM$' README.md || {
    printf 'The SH2ZAM landing-page README is missing; stopping.\n' >&2
    exit 1
}
test -f LICENSE
test -f README.SH4ZAM.md

git branch -m main
git remote rename origin migration-source
git remote set-url --push migration-source DISABLED
git remote add upstream https://github.com/gyrovorbis/sh4zam.git
git remote set-url --push upstream DISABLED

printf 'Publishing commit %s to NEW public repository %s\n' "$(git rev-parse HEAD)" "$target"
gh repo create "$target" --public \
    --description 'Experimental Saturn-native fixed-point math inspired by SH4ZAM. Original authors and MIT license credited.' \
    --disable-wiki --source . --remote origin --push
gh repo edit "$target" --default-branch main --enable-issues
printf '\nCreated %s; original repository unchanged.\n' "$target"
printf 'Local checkout: %s\n' "$PWD"
gh repo view "$target" --json url --jq .url
