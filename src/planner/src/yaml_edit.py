import yaml
import sys
import pdb

# argv[1]: yaml file path
# argv[2]: map file path
argv = sys.argv

with open(argv[1], 'r') as fin:
    file = yaml.safe_load(fin)

if '/' in argv[2]:
    argv[2] = argv[2].split('/')[-1]

file['image'] = argv[2]

with open(argv[1], 'w') as fout:
    yaml.safe_dump(file, fout)