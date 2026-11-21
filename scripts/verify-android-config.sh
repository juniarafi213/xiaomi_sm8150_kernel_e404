#!/usr/bin/env bash
set -euo pipefail

config=${1:-out/.config}
test -r "$config" || { echo "missing kernel config: $config" >&2; exit 1; }

get_config() {
    local key=$1
    sed -n "s/^${key}=//p" "$config" | tail -n 1
}

require_config() {
    local key=$1 expected=$2 actual
    actual=$(get_config "$key")
    if [[ "$actual" != "$expected" ]]; then
        echo "config mismatch: $key=$actual (expected $expected)" >&2
        return 1
    fi
}

require_config CONFIG_CGROUPS y
require_config CONFIG_MEMCG y
require_config CONFIG_MEMCG_SWAP y
require_config CONFIG_MEMCG_SWAP_ENABLED y
require_config CONFIG_PSI y
require_config CONFIG_ANDROID_LOW_MEMORY_KILLER y
require_config CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES y
require_config CONFIG_HZ 100
require_config CONFIG_LITTLE_CPU_MASK 15
require_config CONFIG_BIG_CPU_MASK 112
require_config CONFIG_PRIME_CPU_MASK 128
require_config CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE y
require_config CONFIG_LRU_GEN y
require_config CONFIG_LRU_GEN_ENABLED y

# This kernel tree has no SCHED_WALT Kconfig symbol; don't require stale
# defconfig lines that Kconfig silently drops.

echo "Android HyperOS config verified: $config"
