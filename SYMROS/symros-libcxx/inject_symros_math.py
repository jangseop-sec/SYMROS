"""
Automatically replaces std::xxx / bare C math.h calls with symros_xxx equivalents,
and supports reverting back.

Usage:
  python replace_std_math.py <file_or_dir>                    -- forward (std::xxx -> symros_xxx), in-place
  python replace_std_math.py <file.cpp> <out.cpp>             -- forward, write to out
  python replace_std_math.py --revert <file_or_dir>           -- revert (symros_xxx -> std::xxx), in-place
  python replace_std_math.py --revert <file.cpp> <out>        -- revert, write to out

  Add --c-math to also handle bare C math.h calls (sin, sqrt, ... without std:: prefix):
  python replace_std_math.py --c-math <file_or_dir>           -- forward (std::xxx + bare C -> symros_xxx)
  python replace_std_math.py --revert --c-math <file_or_dir>  -- revert (symros_xxx -> std::xxx + bare C)
"""

import re
import sys
from pathlib import Path

# Target functions to replace (based on my_cmath.hpp)
MATH_FUNCS = [
  # classification
  "isnan", "isinf", "isfinite", "signbit",
  # absolute value / sign
  "abs", "fabs", "copysign",
  # rounding
  "trunc", "floor", "ceil", "round", "lround", "llround", "rint",
  # min / max / dim
  "fmin", "fmax", "fdim",
  # remainder / decomposition
  "fmod", "remainder", "modf", "frexp", "ldexp",
  # roots
  "sqrt", "cbrt", "hypot",
  # exponential / logarithm
  "exp", "exp2", "log", "log2", "log10",
  # power
  "pow",
  # trigonometry
  "sin", "cos", "tan", "asin", "acos", "atan", "atan2",
  # misc
  "fma",
]

# Sort longest first so log2/log10 match before log, asin/acos/atan before sin/cos/tan
MATH_FUNCS_SORTED = sorted(MATH_FUNCS, key=len, reverse=True)
FUNC_PATTERN = "|".join(re.escape(f) for f in MATH_FUNCS_SORTED)

# C math.h functions that are safe to replace as bare identifiers.
# We intentionally exclude "abs" here because it is also used heavily outside
# math contexts (e.g. integer abs from <cstdlib>). Add it back if needed.
C_MATH_FUNCS = [f for f in MATH_FUNCS if f != "abs"]
C_MATH_FUNCS_SORTED = sorted(C_MATH_FUNCS, key=len, reverse=True)
C_FUNC_PATTERN = "|".join(re.escape(f) for f in C_MATH_FUNCS_SORTED)


# ---------------------------------------------------------------------------
# Skip-range helpers (comments + string literals)
# ---------------------------------------------------------------------------

def get_skip_ranges(code: str) -> list[tuple[int, int]]:
  """
  Return list of (start, end) byte ranges that must not be modified
  (i.e. comments and string literals).
  """
  skip_ranges = []
  i = 0
  n = len(code)
  while i < n:
    if code[i:i+2] == "//":
      end = code.find("\n", i)
      end = end if end != -1 else n
      skip_ranges.append((i, end))
      i = end
    elif code[i:i+2] == "/*":
      end = code.find("*/", i+2)
      end = (end + 2) if end != -1 else n
      skip_ranges.append((i, end))
      i = end
    elif code[i] in ('"', "'"):
      q = code[i]
      j = i + 1
      while j < n:
        if code[j] == "\\":
          j += 2
        elif code[j] == q:
          j += 1
          break
        else:
          j += 1
      skip_ranges.append((i, j))
      i = j
    else:
      i += 1
  return skip_ranges


def is_in_skip(pos: int, skip_ranges: list[tuple[int, int]]) -> bool:
  for (s, e) in skip_ranges:
    if s <= pos < e:
      return True
  return False


# ---------------------------------------------------------------------------
# Forward pass
# ---------------------------------------------------------------------------

def replace_std_math(code: str, prefix: str = "symros") -> tuple[str, list]:
  """
  Forward pass: replace std::xxx with {prefix}_xxx.
  Returns (transformed_code, list_of_changes).
  """
  skip_ranges = get_skip_ranges(code)

  # Match std::<func> only when followed by '(', whitespace, or '<'
  # to avoid matching non-math identifiers like std::string
  pattern = re.compile(r'\bstd::(' + FUNC_PATTERN + r')(?=[(\s<])')

  changes = []
  result = []
  last = 0

  for m in pattern.finditer(code):
    if is_in_skip(m.start(), skip_ranges):
      result.append(code[last:m.end()])
      last = m.end()
      continue

    func_name = m.group(1)
    replacement = f"{prefix}_{func_name}"
    result.append(code[last:m.start()])
    result.append(replacement)
    last = m.end()
    changes.append({
      "line":        code[:m.start()].count("\n") + 1,
      "original":    m.group(0),
      "replacement": replacement,
    })

  result.append(code[last:])
  return "".join(result), changes


def replace_c_math(code: str, prefix: str = "symros") -> tuple[str, list]:
  """
  Forward pass for C math.h bare calls: replace xxx( with {prefix}_xxx(.

  A bare call is recognised when the function name is:
    - preceded by a word boundary and NOT preceded by '::' or a word char
      that would make it part of a longer identifier
    - followed by '(' optionally preceded by whitespace

  We deliberately skip matches that are already prefixed (e.g. symros_sin)
  by requiring a non-word, non-colon character (or start-of-string) before
  the function name.
  """
  skip_ranges = get_skip_ranges(code)

  # (?<![:\w]) ensures we don't match inside std::sin or already-replaced symros_sin
  pattern = re.compile(r'(?<![:\w])(' + C_FUNC_PATTERN + r')(?=\s*\()')

  changes = []
  result = []
  last = 0

  for m in pattern.finditer(code):
    if is_in_skip(m.start(), skip_ranges):
      result.append(code[last:m.end()])
      last = m.end()
      continue

    func_name = m.group(1)
    replacement = f"{prefix}_{func_name}"
    result.append(code[last:m.start()])
    result.append(replacement)
    last = m.end()
    changes.append({
      "line":        code[:m.start()].count("\n") + 1,
      "original":    m.group(0),
      "replacement": replacement,
      "source":      "c_math",
    })

  result.append(code[last:])
  return "".join(result), changes


# ---------------------------------------------------------------------------
# Reverse pass
# ---------------------------------------------------------------------------

def revert_std_math(code: str, prefix: str = "symros") -> tuple[str, list]:
  """
  Reverse pass: replace {prefix}_xxx back to std::xxx.
  Returns (transformed_code, list_of_changes).
  """
  skip_ranges = get_skip_ranges(code)

  pattern = re.compile(r'\b' + re.escape(prefix) + r'_(' + FUNC_PATTERN + r')(?=[(\s])')

  changes = []
  result = []
  last = 0

  for m in pattern.finditer(code):
    if is_in_skip(m.start(), skip_ranges):
      result.append(code[last:m.end()])
      last = m.end()
      continue

    func_name = m.group(1)
    replacement = f"std::{func_name}"
    result.append(code[last:m.start()])
    result.append(replacement)
    last = m.end()
    changes.append({
      "line":        code[:m.start()].count("\n") + 1,
      "original":    m.group(0),
      "replacement": replacement,
    })

  result.append(code[last:])
  return "".join(result), changes


def revert_c_math(code: str, prefix: str = "symros") -> tuple[str, list]:
  """
  Reverse pass for C math.h bare calls: replace {prefix}_xxx back to xxx.
  Only reverts functions that were originally bare C math calls, i.e.
  functions in C_MATH_FUNCS (abs excluded).
  """
  skip_ranges = get_skip_ranges(code)

  pattern = re.compile(r'\b' + re.escape(prefix) + r'_(' + C_FUNC_PATTERN + r')(?=\s*\()')

  changes = []
  result = []
  last = 0

  for m in pattern.finditer(code):
    if is_in_skip(m.start(), skip_ranges):
      result.append(code[last:m.end()])
      last = m.end()
      continue

    func_name = m.group(1)
    replacement = func_name          # bare function name, no std:: prefix
    result.append(code[last:m.start()])
    result.append(replacement)
    last = m.end()
    changes.append({
      "line":        code[:m.start()].count("\n") + 1,
      "original":    m.group(0),
      "replacement": replacement,
      "source":      "c_math",
    })

  result.append(code[last:])
  return "".join(result), changes


# ---------------------------------------------------------------------------
# File / directory processing
# ---------------------------------------------------------------------------

def process_file(
  input_path: str,
  output_path: str | None = None,
  prefix: str = "symros",
  revert: bool = False,
  c_math: bool = False,
):
  with open(input_path, "r", encoding="utf-8") as f:
    original = f.read()

  all_changes = []

  if revert:
    # In revert mode with --c-math, symros_xxx -> bare xxx for C funcs.
    # We run c_math revert first (more specific: no std:: prefix back),
    # then std revert for anything remaining.
    if c_math:
      transformed, changes = revert_c_math(original, prefix)
      all_changes.extend(changes)
      transformed, changes2 = revert_std_math(transformed, prefix)
      all_changes.extend(changes2)
      direction = f"{prefix}_xxx  ->  xxx / std::xxx"
    else:
      transformed, changes = revert_std_math(original, prefix)
      all_changes.extend(changes)
      direction = f"{prefix}_xxx  ->  std::xxx"
  else:
    # Forward: std::xxx first, then bare C math (if requested).
    transformed, changes = replace_std_math(original, prefix)
    all_changes.extend(changes)
    if c_math:
      transformed, changes2 = replace_c_math(transformed, prefix)
      all_changes.extend(changes2)
      direction = f"std::xxx / bare C  ->  {prefix}_xxx"
    else:
      direction = f"std::xxx  ->  {prefix}_xxx"

  out = output_path or input_path
  with open(out, "w", encoding="utf-8") as f:
    f.write(transformed)

  print(f"[replace_std_math] ({direction}) {len(all_changes)} replacement(s) -> {out}")
  for c in sorted(all_changes, key=lambda x: x["line"]):
    tag = " [C]" if c.get("source") == "c_math" else ""
    print(f"  line {c['line']:4d}{tag}: {c['original']}  ->  {c['replacement']}")

  return all_changes


def process_dir(
  dir_path: str,
  prefix: str = "symros",
  revert: bool = False,
  c_math: bool = False,
):
  """
  Recursively find all C/C++ source files under dir_path and replace in-place.
  """
  extensions = {".cpp", ".hpp", ".cc", ".cxx", ".h", ".c"}
  files = [p for p in Path(dir_path).rglob("*") if p.suffix in extensions]

  if not files:
    print(f"[replace_std_math] No source files found in '{dir_path}'")
    return

  total_changes = 0
  for f in sorted(files):
    changes = process_file(str(f), output_path=None, prefix=prefix,
                           revert=revert, c_math=c_math)
    total_changes += len(changes)

  mode = "revert" if revert else "forward"
  print(f"\n[replace_std_math] Done ({mode}). "
        f"{len(files)} file(s) processed, {total_changes} total replacement(s).")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
  args = sys.argv[1:]

  revert = False
  if "--revert" in args:
    revert = True
    args = [a for a in args if a != "--revert"]

  c_math = False
  if "--c-math" in args:
    c_math = True
    args = [a for a in args if a != "--c-math"]

  if not args:
    print("Usage:")
    print("  python replace_std_math.py <file_or_dir>                    -- forward (std::xxx -> symros_xxx)")
    print("  python replace_std_math.py --revert <file_or_dir>           -- revert  (symros_xxx -> std::xxx)")
    print("  python replace_std_math.py <file.cpp> <out.cpp>             -- forward, write to out")
    print("  python replace_std_math.py --revert <file.cpp> <out>        -- revert,  write to out")
    print()
    print("  Add --c-math to also handle bare C math.h calls (e.g. sin(...), sqrt(...)):")
    print("  python replace_std_math.py --c-math <file_or_dir>")
    print("  python replace_std_math.py --revert --c-math <file_or_dir>")
    sys.exit(1)

  target = args[0]

  if Path(target).is_dir():
    if len(args) >= 2:
      print("Error: output path cannot be specified when target is a directory.")
      sys.exit(1)
    process_dir(target, revert=revert, c_math=c_math)
  else:
    out = args[1] if len(args) >= 2 else None
    process_file(target, out, revert=revert, c_math=c_math)