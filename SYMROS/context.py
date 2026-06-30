from multiprocessing import JoinableQueue, Manager, Array, Value
import time

class GlobalContext:
    
    def __init__(self, eid, grater_enable=True, num_process=16, num_process_per_state=1, max_constraint=300, target_offset=0, timeout=3600):
        # basic setting
        self.max_constraint = max_constraint
        self.grater_enable = grater_enable
        self.eid = eid
        self.num_process = num_process
        self.num_process_per_state = num_process_per_state
        self.target_offset = target_offset
        
        # state transition
        self.is_state_mode = False
        
        # work queue
        self.queue_index = Array('i', [0] + [(x - 1) // num_process_per_state + 1 for x in range(1, num_process + 1)])
        self.constraint_queues = [JoinableQueue() for _ in range(num_process)]
        self.input_queues = [JoinableQueue() for _ in range(num_process)]
        self.bug_input_queues = [JoinableQueue() for _ in range(num_process)]
        self.new_state_queue = JoinableQueue()
        
        # symros state manager
        manager = Manager()
        self.executor_state = manager.list([manager.dict() for _ in range(num_process)])
        self.solver_state = manager.list([manager.dict() for _ in range(num_process)])
        self.fp_solver_state = manager.list([manager.dict() for _ in range(num_process)])
        self.recover_state = manager.list([manager.dict() for _ in range(num_process)])
        self.state_source = manager.list([""] * (num_process + 1))
        
        # adaptive solving timeout
        self.recent_new_found = Value('d', time.perf_counter())
        
        # timeout
        self.base_time = time.perf_counter()
        self.timeout = timeout
        