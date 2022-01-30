from readerwriterlock import rwlock

class Bus(object):
    def __init__(self):
        self.lock = rwlock.RWLockWriteD()
        self.message = None

    def write(self, data):
        with self.lock.gen_wlock():
            self.message = data

    def read(self):
        with self.lock.gen_rlock():
            return self.message