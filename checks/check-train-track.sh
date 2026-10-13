#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
example="$root/examples/train-track"
map="$example/source-map.tsv"
postscript="$example/straight-track.ps"
gcode="$example/straight-track-pen.gcode"

fail() {
  printf '%s\n' "check-train-track: $*" >&2
  exit 1
}

for file in "$map" "$postscript" "$gcode"; do
  [ -f "$file" ] || fail "missing $file"
done

component_count=$(awk -F '\t' 'NR > 1 && $3 != "train-track" { count++ } END { print count + 0 }' "$map")
[ "$component_count" -eq 10 ] || fail "expected 10 semantic strokes, found $component_count"

awk -F '\t' 'NR > 1 && $3 != "train-track" { print $1 }' "$map" |
while IFS= read -r semantic_id; do
  ps_count=$(grep -F -c "% @source $semantic_id" "$postscript" || true)
  gc_count=$(grep -F -c "(@source $semantic_id)" "$gcode" || true)
  [ "$ps_count" -eq 1 ] || fail "$semantic_id occurs $ps_count times in PostScript"
  [ "$gc_count" -eq 1 ] || fail "$semantic_id occurs $gc_count times in G-code"
done

awk '
function bad(message) {
  print "check-train-track: " message > "/dev/stderr"
  failed = 1
}
function close_source() {
  if (source != "" && working_moves != 1) {
    bad(source " has " working_moves " working XY moves; expected 1")
  }
}
/^\(@source / {
  close_source()
  source = $0
  working_moves = 0
  next
}
/^\(@endobject / {
  close_source()
  source = ""
  next
}
/^G0 Z5([.]0+)?([[:space:]]|$)/ {
  pen = "up"
  next
}
/^G1 Z0([.]0+)?([[:space:]]|$)/ {
  pen = "down"
  next
}
/^G0 / && /X/ && /Y/ {
  if (pen != "up") bad("XY travel occurred while pen was not raised: " $0)
  next
}
/^G1 / && /X/ && /Y/ {
  if (source == "") bad("working move has no semantic source: " $0)
  if (pen != "down") bad("working move occurred while pen was not down: " $0)
  working_moves++
  total_working_moves++
  next
}
END {
  close_source()
  if (total_working_moves != 10) {
    bad("expected 10 working XY moves, found " total_working_moves)
  }
  exit failed
}
' "$gcode" || exit 1

if command -v gs >/dev/null 2>&1; then
  gs -q -dNOPAUSE -dBATCH -sDEVICE=nullpage "$postscript"
fi

printf '%s\n' "train-track checks passed"
