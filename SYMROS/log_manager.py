import sys
import time
import os

from context import GlobalContext


def log_once(ctx: GlobalContext, worker_id: int):
  wid = worker_id
  qi = ctx.queue_index[wid]
  sl = ctx.solver_state[wid]
  ex = ctx.executor_state[wid]
  rc = ctx.recover_state[wid]

  def trunc(s, w):
    s = str(s) if s else "-"
    return s[-w:].ljust(w) if len(s) > w else s.ljust(w)

  SLV_INP = 7;  SLV_METH = 10; SLV_ST = 10; SLV_REM = 2
  EXE_INP = 7;  EXE_LOC  = 80; EXE_ST =  8; EXE_REM = 2
  RCV_INP = 7;  RCV_LOC  = 80; RCV_CHK= 16; RCV_REM = 2

  slv = (f"SOL id={trunc((sl['input_id'] if sl['input_id'] != -1 else 'none'), SLV_INP)}"
         f" meth={trunc(sl['method'],     SLV_METH)}"
         f" st={trunc(sl['status'],       SLV_ST)}"
         f" rem={trunc(sl['remain_constraints'], SLV_REM)}")

  exe = (f"EXE id={trunc((ex['input_id'] if ex['input_id'] != -1 else 'none'), EXE_INP)}"
         f" loc={trunc(ex['source_location'], EXE_LOC)}"
         f" st={trunc(ex['status'],       EXE_ST)}"
         f" rem={trunc(ex['remain_input'], EXE_REM)}")

  rcv = (f"CHK id={trunc((rc['input_id'] if rc['input_id'] != -1 else 'none'), RCV_INP)}"
         f" loc={trunc(rc['bug_location'], RCV_LOC)}"
         f" chk={trunc(rc['status'],      RCV_CHK)}"
         f" rem={trunc(rc['remain_input'], RCV_REM)}")

  state = trunc(ctx.state_source[qi], 7)
  line  = f"| W{wid:>02d}[{state}] {slv} | {exe} | {rcv} |"
  sep   = "|" + "-" * (len(line) - 2) + "|"

  return [sep, line, sep] 

def log_manager(ctx):
  # os.system("clear")
  time.sleep(1)
  BLOCK_HEIGHT = 2
  cnt = 0
  for i in range(ctx.num_process):
    for _ in range(BLOCK_HEIGHT):
      print()

  while True:
    cnt += 1
    if cnt == 100:
      os.system("clear")
      for i in range(ctx.num_process):
        for _ in range(BLOCK_HEIGHT):
          print()
      cnt = 0
      
    for worker_id in range(ctx.num_process):

      start_line = worker_id * BLOCK_HEIGHT + 1

      block = log_once(ctx, worker_id)

      for i, line in enumerate(block):

        sys.stdout.write(f"\033[{start_line + i};1H")
        sys.stdout.write(line)

      sys.stdout.flush()
    time.sleep(0.2)