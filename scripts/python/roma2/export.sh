#!/usr/bin/env bash
# Export, check and describe every graph an openMVS model directory publishes.
#
#   ./export.sh --images A.jpg B.jpg --out-dir ~/virginia/models/roma2-onnx/<export-id>
#   ./export.sh --images A.jpg B.jpg --settings base --out-dir <dir>          # one preset
#   ./export.sh --images A.jpg B.jpg --out-dir <dir> --fixtures ~/openMVS/apps/Tests/data/roma2
#
# The two images are a real overlapping pair from a capture, and they are not optional: the matcher head is
# checked against a reference built from them, because its warp is an argmax over a correlation volume and
# random descriptors give it no peak to find. The descriptor is checked against the traced reference too,
# which is all it needs, and then against the real one, which is what its pooled retrieval descriptors are
# judged on.
#
# Per setting: trace both stages, check the descriptor against its own traced reference, build the two real
# references, check both stages against those, run the two extra descriptor checks (the CPU execution
# provider, and the model's own bf16 noise floor), and write the manifest. There is no engine step —
# openMVS consumes the ONNX through onnxruntime, and the graphs are the shipped artifact.
#
# Everything is teed into $OUT_DIR/export.log, so a shipped model directory carries the log of the run that
# produced it and re-running this script reproduces that log.
set -euo pipefail

SETTINGS="base fast turbo"
STAGES="descriptor match_coarse"
IMAGES=()
OUT_DIR="."
FIXTURES=""
REPEAT=100
CPU_REPEAT=5        # the CPU provider runs about a second per descriptor at base; 5 is enough to time it

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [ $# -gt 0 ]; do
  case "$1" in
    --images) IMAGES=("$2" "$3"); shift 3 ;;
    --settings) SETTINGS="$2"; shift 2 ;;
    --stages) STAGES="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --fixtures) FIXTURES="$2"; shift 2 ;;
    --repeat) REPEAT="$2"; shift 2 ;;
    -h|--help) usage 0 ;;
    *) echo "unknown argument: $1" >&2; usage 1 ;;
  esac
done

if [ "${#IMAGES[@]}" -ne 2 ]; then
  echo "--images needs two overlapping frames from a capture" >&2
  usage 1
fi
for image in "${IMAGES[@]}"; do
  [ -f "$image" ] || { echo "no such image: $image" >&2; exit 1; }
done

project="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

# Everything runs in polyml's export project env: torch, onnx and onnxscript come from its lock file, and
# onnxruntime-gpu is added on top so that the trace and the check of it happen in one process family.
ROMA2_PROJECT="${ROMA2_PROJECT:-$HOME/polyml/romav2}"
ORT="${ROMA2_ORT:-onnxruntime-gpu==1.23.2}"
run() { uv run --project "$ROMA2_PROJECT" --with "$ORT" python "$project/$1" "${@:2}"; }

export_all() {
echo "exporting into $OUT_DIR on $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"

failures=0
for setting in $SETTINGS; do
  for stage in $STAGES; do
    coarse=""
    graph_stage="$stage"
    if [ "$stage" = "match_coarse" ]; then
      coarse="--coarse"
      graph_stage="match"
    fi
    graph="$OUT_DIR/roma_${setting}_${stage}_fp32.onnx"
    reference="$OUT_DIR/real_${setting}_${stage}.reference"

    echo ""
    echo "===== ${setting}/${stage}"
    run export.py onnx --stage "$graph_stage" $coarse --setting "$setting" --out-dir "$OUT_DIR"
    [ -f "$graph" ] || { echo "  missing $graph"; failures=$((failures + 1)); continue; }

    if [ "$graph_stage" = "descriptor" ]; then
      # The traced reference first: random input is what catches a graph that computes nothing at all.
      run export.py check --onnx "$graph" --repeat "$REPEAT" || failures=$((failures + 1))
    fi

    fixtures=()
    if [ -n "$FIXTURES" ] && [ "$stage" = "descriptor" ]; then
      fixtures=(--fixtures "$FIXTURES")
    fi
    run parity.py "${IMAGES[0]}" "${IMAGES[1]}" --stage "$graph_stage" $coarse --setting "$setting" \
        --out-dir "$reference" ${fixtures[@]+"${fixtures[@]}"}
    run export.py check --onnx "$graph" --reference "$reference" --repeat "$REPEAT" \
        || failures=$((failures + 1))
  done

  # The two checks that are about context rather than about shipping: the CPU execution provider is the
  # tolerance floor every non-CUDA platform sees (and the only provider some of them have), and the noise
  # floor says how far the model's own bf16 autocast lands from the same eager fp32 reference — which is
  # what makes a cosine of 0.999999 mean something.
  descriptor="$OUT_DIR/roma_${setting}_descriptor_fp32.onnx"
  descriptor_reference="$OUT_DIR/real_${setting}_descriptor.reference"
  if [ -f "$descriptor" ] && [ -d "$descriptor_reference" ]; then
    echo ""
    echo "===== ${setting}/descriptor extra checks"
    run export.py check --onnx "$descriptor" --reference "$descriptor_reference" \
        --provider cpu --repeat "$CPU_REPEAT" || failures=$((failures + 1))
    run export.py check --onnx "$descriptor" --reference "$descriptor_reference" \
        --repeat 0 --noise-floor || failures=$((failures + 1))
  fi

  echo ""
  echo "===== ${setting}/manifest"
  run export.py manifest --out-dir "$OUT_DIR" --setting "$setting" || failures=$((failures + 1))
done

echo ""
echo "===== $OUT_DIR"
ls -l "$OUT_DIR"/roma_*.onnx "$OUT_DIR"/roma_*.json | awk '{printf "%10.1f MB  %s\n", $5 / 1048576, $9}'
if [ "$failures" -ne 0 ]; then
  echo "$failures stage(s) failed" >&2
  return 1
fi
echo "all stages checked"
}

# The log belongs to the model directory rather than to whoever ran the script: a shipped set of graphs
# carries the checks that passed on it. PIPESTATUS, because tee is the process that exits last.
export_all 2>&1 | tee "$OUT_DIR/export.log"
exit "${PIPESTATUS[0]}"
