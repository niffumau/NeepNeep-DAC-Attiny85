#!/bin/bash
OUTPUT_FILE="output.bin"
TMP_DATA="data.tmp"
> "$OUTPUT_FILE"
> "$TMP_DATA"

# Collect all WAV files
wav_files=( *.wav )
num_files=${#wav_files[@]}

# Gather sizes and accumulate
declare -a data_sizes
declare -a offsets
current_offset=0

# Strip headers and save combined data
for wav_file in "${wav_files[@]}"; do
  if [[ -f "$wav_file" ]]; then
    size=$(stat -c%s "$wav_file" 2>/dev/null || stat -f%z "$wav_file")
    data_size=$((size - 44))
    if (( data_size > 0 )); then
      tail -c +45 "$wav_file" >> "$TMP_DATA" 2>/dev/null
      data_sizes+=( "$data_size" )
      offsets+=( "$current_offset" )
      current_offset=$((current_offset + data_size))
    fi
  fi
done

# Add the end offset
offsets+=( "$current_offset" )

# Compute where data starts (after offset table and two nulls)
offset_table_count=${#offsets[@]}
offset_table_size=$(( (offset_table_count + 2) * 4 ))

# Adjust offsets to be absolute (starting from beginning of file)
for ((i=0; i<${#offsets[@]}; i++)); do
  offsets[$i]=$((offsets[$i] + offset_table_size))
done

# Write offsets as little-endian 32-bit unsigned integers
for off in "${offsets[@]}"; do
  printf "%08x" "$off" | tac -rs .. | xxd -r -p >> "$OUTPUT_FILE"
done

# Write two null uint32_t placeholders
printf "\x00\x00\x00\x00\x00\x00\x00\x00" >> "$OUTPUT_FILE"

# Append the actual waveform data
cat "$TMP_DATA" >> "$OUTPUT_FILE"

# Cleanup
rm "$TMP_DATA"

total_size=$(stat -c%s "$OUTPUT_FILE" 2>/dev/null || stat -f%z "$OUTPUT_FILE")
echo "Generated $OUTPUT_FILE ($total_size bytes)"
echo "Offsets (${#offsets[@]} entries + 2 nulls): ${offsets[*]}"

