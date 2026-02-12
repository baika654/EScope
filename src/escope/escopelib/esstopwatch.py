import time

class Stopwatch:
    def __init__(self):
        self.start_time = None
        self.elapsed_time = 0

    def Start(self):
        if self.start_time is None:
            self.start_time = time.time()
            #print("Stopwatch started.")
        #else:
            #print("Stopwatch is already running.")

    def Stop(self):
        if self.start_time is not None:
            self.elapsed_time += time.time() - self.start_time
            self.start_time = None
            #print(f"Stopwatch stopped. Elapsed time: {self.elapsed_time:.2f} seconds.")
        #else:
            #print("Stopwatch is not running.")

    def Restart(self):
        self.start_time = None
        self.elapsed_time = 0
        #print("Stopwatch reset.")

    def ElapsedMilliseconds(self):
        if self.start_time is not None:
            return self.elapsed_time + (time.time() - self.start_time)
        return self.elapsed_time*1000