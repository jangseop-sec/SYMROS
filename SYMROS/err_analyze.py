import glob
import sys

def parse_err_files(content):
  err_type = ""
  err_loc = ""
  asan_detect = False
  if "runtime_error" in content:
    print("runtime error")
    return
  for line in content.splitlines():
    if "ERROR: AddressSanitizer:" in line:
      asan_detect = True
      err_type = line.split(': ')[2]
    if asan_detect and "#0" in line:
      err_loc = line.split('#0 ')[1].split(' ')[0]
  print(f"{err_type} / {err_loc}")
      


folder = sys.argv[1]

for filepath in glob.glob(folder + "/*.err"):
  with open(filepath, "r") as f:
    content = f.read()
    print(f"[{filepath}]: ", end='')
    parse_err_files(content)