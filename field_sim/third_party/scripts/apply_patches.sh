#!/bin/sh
# Usage: apply_patches.sh <visualizer-src dir>. Makes absolute asset URLs relative so the app works under /visualizer/, and disables the service worker.
S="$1"
cd "$S" || exit 1
git apply ../patches/0001-relative-asset-paths.patch && echo applied
