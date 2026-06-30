# bitmap method
def set_bit(bitmap, bit):
  byte = bit >> 3
  off  = bit & 7
  bitmap[byte] |= (1 << off)

def clear_bit(bitmap, bit):
  byte = bit >> 3
  off  = bit & 7
  bitmap[byte] &= ~(1 << off)

def test_bit(bitmap, bit):
  byte = bit >> 3
  off  = bit & 7
  return (bitmap[byte] >> off) & 1
# bitmap_utils.py

# =========================
# Internal helpers
# =========================

def _normalize(index, width, bitmap):
    total_bits = len(bitmap) * 8
    if total_bits == 0:
        raise ValueError("bitmap size is 0")
    return (index * width) % total_bits, total_bits


def _get_bit(bitmap, bit_index):
    byte = bit_index >> 3
    off  = bit_index & 7
    return (bitmap[byte] >> off) & 1


def _set_bit(bitmap, bit_index, val):
    byte = bit_index >> 3
    off  = bit_index & 7
    mask = 1 << off

    if val:
        bitmap[byte] |= mask
    else:
        bitmap[byte] &= ~mask


# =========================
# Public API
# =========================

def set_bits(bitmap, index, value, width):
    base, total_bits = _normalize(index, width, bitmap)

    for i in range(width):
        bit_index = (base + i) % total_bits
        _set_bit(bitmap, bit_index, (value >> i) & 1)


def get_bits(bitmap, index, width):
    base, total_bits = _normalize(index, width, bitmap)

    result = 0
    for i in range(width):
        bit_index = (base + i) % total_bits
        if _get_bit(bitmap, bit_index):
            result |= (1 << i)

    return result


def inc_bits(bitmap, index, width):
    base, total_bits = _normalize(index, width, bitmap)

    curr = 0
    for i in range(width):
        bit_index = (base + i) % total_bits
        if _get_bit(bitmap, bit_index):
            curr |= (1 << i)

    max_val = (1 << width) - 1
    if curr < max_val:
        curr += 1

    for i in range(width):
        bit_index = (base + i) % total_bits
        _set_bit(bitmap, bit_index, (curr >> i) & 1)


def clear_bits(bitmap, index, width):
    base, total_bits = _normalize(index, width, bitmap)

    for i in range(width):
        bit_index = (base + i) % total_bits
        _set_bit(bitmap, bit_index, 0)


def test_bits(bitmap, index, value, width):
    return get_bits(bitmap, index, width) == value

def update_path_hash(path_hash: int, curr_location: str) -> int:
  
  # hashing
  MASK = (64 * 1024 * 8) - 1   # 64KB bit-level → 524287 (0x7FFFF)
  BASE = 131                  # any odd number

  h = path_hash
  for b in curr_location.encode():
      h = ((h * BASE) ^ b) & MASK
  return h

def trim_pattern_loops(seq, max_repeat=2, key=None):
    def get_key(x):
        return x[key] if key is not None else x

    result = []
    result_keys = []

    for x in seq:
        result.append(x)
        result_keys.append(get_key(x))

        n = len(result)
        removed = False
        for l in range(1, n // 2 + 1):
            needed = l * (max_repeat + 1)
            if n < needed:
                continue

            tail = result_keys[-needed:]
            pattern = tail[:l]

            if all(tail[i] == pattern[i % l] for i in range(needed)):
                del result[-l:]
                del result_keys[-l:]
                removed = True
                break

    return result