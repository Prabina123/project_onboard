#!/bin/bash

directory="missions"

# Use the find command to search for all JSON files in the specified directory and its subdirectories.
# The "-type f" flag ensures we only search for files, not directories.
# The "-name" flag specifies the pattern to match filenames (in this case, '*.json' for JSON files).
# The results will be stored in the variable 'json_files'.
json_files=$(find "$directory" -type f -name '*.json')

# Loop through the JSON files found and process each file as needed.
for file in $json_files; do
    echo "Processing MISSION: $file"
    # Add your commands here...
    python src/dr_onboard_autonomy/verify_mission.py $file
    if [ $? -ne 0 ]; then
        echo "Error in mission: $file"
        exit 1
    fi
    echo "DONE Processing MISSION: $file"
done
