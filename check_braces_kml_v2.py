import sys

def check_braces(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()

    stack = []
    in_string = False
    in_char = False
    in_comment = False
    in_block_comment = False
    
    i = 0
    while i < len(content):
        char = content[i]
        next_char = content[i+1] if i+1 < len(content) else ''
        
        if in_block_comment:
            if char == '*' and next_char == '/':
                in_block_comment = False
                i += 1
        elif in_comment:
            if char == '\n':
                in_comment = False
        elif in_string:
            if char == '\\':
                i += 1
            elif char == '"':
                in_string = False
        elif in_char:
            if char == '\\':
                i += 1
            elif char == "'":
                in_char = False
        else:
            if char == '/' and next_char == '/':
                in_comment = True
                i += 1
            elif char == '/' and next_char == '*':
                in_block_comment = True
                i += 1
            elif char == '"':
                in_string = True
            elif char == "'":
                in_char = True
            elif char == '{':
                line_info = content[:i].count('\n') + 1
                stack.append(line_info)
            elif char == '}':
                line_info = content[:i].count('\n') + 1
                if not stack:
                    print(f"Extra closing brace at line {line_info}")
                else:
                    stack.pop()
        i += 1
    
    if stack:
        print(f"Unclosed braces starting at lines: {stack}")
    else:
        print("All braces matched!")

check_braces('d:/workspace/HikePod/src/KMLParser.cpp')
