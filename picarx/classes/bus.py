class Bus():
    def __init__(self, init_message=list()):
        self.message = init_message

    def write(self, message):
        self.message = message

    def read(self):
        return(self.message)



















--------------------------


from time import sleep
from concurrent.futures import ThreadPoolExecutor
from threading import Event

# Define shutdown event
shutdown_event = Event()

# Exception handle function
def handle_exception(future):
    exception = future.exception()
    if exception:
        print(f"Exception in worker thread: {exception}")


# Define robot task
def robot_task(i):
    print("Starting robot task", i)
    while not shutdown_event.is_set():
        # Run some robot task...
        print("Running robot task", i)
        sleep(1)
    
    # Print shut down message
    print("Shutting down robot task", i)
    # Test exception
    if i == 1:
        raise Exception("Robot task 1 raised an exception")

  
if __name__ == "__main__":
    futures = []
    with ThreadPoolExecutor(max_workers=3) as executor:
        for i in range(3):
            # Spawn task threads
            future = executor.submit(robot_task, i)
            # Add exception call back
            future.add_done_callback(handle_exception)
            futures.append(future)
    
    try:
        # Keep the main thread running to response for the kill signal
        while not shutdown_event.is_set():
            sleep(1)
    
    except KeyboardInterrupt:
        # Trigger the shutdown event when receive the kill signal
        print("Shutting down")
        shutdown_event.set()
    
    finally:
        # Ensures all threads finish
        executor.shutdown()