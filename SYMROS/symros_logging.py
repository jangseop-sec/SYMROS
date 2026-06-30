import sys

def symros_log_info(msg, thread_id=0):
    print(f"\033[32m[SYMROS_INFO.{thread_id}] {msg}\033[0m", file=sys.stderr)

def symros_log_warn(msg, thread_id=0):
    print(f"\033[33m[SYMROS_WARN.{thread_id}] {msg}\033[0m", file=sys.stderr)

def symros_log_error(msg, thread_id=0):
    print(f"\033[31m[SYMROS_ERROR.{thread_id}] {msg}\033[0m", file=sys.stderr)

def symros_log_debug(msg, thread_id=0):
    print(f"\033[36m[SYMROS_DEBUG.{thread_id}] {msg}\033[0m", file=sys.stderr)