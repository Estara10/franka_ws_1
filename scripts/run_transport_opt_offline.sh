#!/usr/bin/env bash
# S2 transport-segment projection comparison: PLANNING ONLY.
#
# Starts the same single simulation stack as the contact probe, but the task
# stops inside the prepare step and never closes the gripper, lifts, welds or
# sends any arm target. The transport segment is built from the reconstructed
# nominal grasp/lift state and evaluated twice: the current object-space seed
# path, and the closed-chain projected path.
#
# Timing is the frozen comparison baseline: transport stretch 1x (the existing
# CONTACT_TRANSPORT_STRETCH mechanism), internal compliance off. Nothing in the
# timing pipeline is changed by this run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export CONTACT_TRANSPORT_PLAN_ONLY=1
export CONTACT_TRANSPORT_STRETCH=1.0
export CONTACT_PROBE_RUN_ID="${CONTACT_PROBE_RUN_ID:-lidio_transport_opt_$(date +%Y%m%d_%H%M%S)}"

exec bash "${HERE}/run_contact_probe_local.sh" "$@"
