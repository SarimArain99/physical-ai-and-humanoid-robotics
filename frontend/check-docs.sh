#!/bin/bash

echo "Checking for common documentation issues..."

# Check for broken links (absolute paths that don't exist)
echo "Checking docs folder structure..."
find docs -type f -name "*.md" | while read -r file; do
    if grep -qE '\]\(/blog|/docs|http://|https://)' "$file"; then
        echo "  Potential link in: $file"
    fi
done

# Check for markdown syntax issues
echo ""
echo "Checking for common markdown issues..."
find docs -type f -name "*.md" | while read -r file; do
    # Check for unclosed code blocks
    open_backticks=$(grep -c '```' "$file")
    if [ $((open_backticks % 2)) -ne 0 ]; then
        echo "  Odd number of backticks in: $file"
    fi
done

# Check for empty frontmatter
echo ""
echo "Checking frontmatter..."
find docs -type f -name "*.md" | while read -r file; do
    if head -1 "$file" | grep -q '---'; then
        # Has frontmatter, check if it's closed
        if ! awk '/^---$/ {count++; if(count==2) exit} END {exit (count==2?0:1)}' "$file"; then
            echo "  Unclosed frontmatter in: $file"
        fi
    fi
done

echo ""
echo "Doc check complete."
