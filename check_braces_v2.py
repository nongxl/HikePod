
def check_braces(filename, start_line, end_line):
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    balance = 0
    for i in range(0, start_line - 1):
        line = lines[i]
        for char in line:
            if char == '{': balance += 1
            elif char == '}': balance -= 1
    
    print(f"Balance at line {start_line}: {balance}")
    
    for i in range(start_line - 1, end_line):
        line = lines[i]
        old_balance = balance
        for char in line:
            if char == '{': balance += 1
            elif char == '}': balance -= 1
        print(f"{i+1:4d}: {old_balance:2d} -> {balance:2d} | {line.strip()}")
    
    print(f"Final Balance: {balance}")

check_braces('d:/workspace/HikePod/src/main.cpp', 2180, 2520)
