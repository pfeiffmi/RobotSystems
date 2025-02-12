from readerwriterlock import rwlock

class Bus():
    def __init__(self, init_message=list()):
        self.lock = rwlock.RWLockWriteD()
        with self.lock.gen_wlock():
            self.message = init_message

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return(message)

