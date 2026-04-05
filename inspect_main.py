from pathlib import Path
text = Path('main.py').read_text(encoding='utf-8')
for i, line in enumerate(text.splitlines(), 1):
    if 'duration' in line or 'simulation' in line:
        print(i, repr(line))
