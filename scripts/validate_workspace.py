from pathlib import Path
import ast
import sys
import xml.etree.ElementTree as ET
try:
    import yaml
except ImportError:
    yaml = None
root = Path(__file__).resolve().parents[1]
errors = []
for path in root.rglob('package.xml'):
    try:
        ET.parse(path)
    except Exception as exc:
        errors.append(f'{path}: XML: {exc}')
for path in root.rglob('*.xml'):
    try:
        ET.parse(path)
    except Exception as exc:
        errors.append(f'{path}: XML: {exc}')
for path in root.rglob('*.py'):
    try:
        ast.parse(path.read_text())
    except Exception as exc:
        errors.append(f'{path}: Python: {exc}')
if yaml:
    for path in root.rglob('*.yaml'):
        try:
            yaml.safe_load(path.read_text())
        except Exception as exc:
            errors.append(f'{path}: YAML: {exc}')
if errors:
    print('\n'.join(errors))
    sys.exit(1)
print('Static validation passed: XML, Python syntax and YAML.')
