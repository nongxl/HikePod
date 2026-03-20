import sys

with open('d:/workspace/HikePod/src/KMLParser.cpp', 'r', encoding='utf-8') as f:
    lines = f.readlines()

stack = []
for i, line in enumerate(lines):
    line_num = i + 1
    if line_num < 105: continue
    for char in line:
        if char == '{':
            stack.append(line_num)
        elif char == '}':
            if not stack:
                print(f"Extra closing brace at line {line_num}")
            else:
                stack.pop()
    
    if line_num > 400: break

if stack:
    print(f"Unclosed braces starting at lines: {stack}")
else:
    print("All braces matched!")
