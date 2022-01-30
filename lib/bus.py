from readerwriterlock import rwlock

class Bus(object):
    def __init__(self, default):
        self.lock = rwlock.RWLockWriteD()
        self.message = default

    def write(self, data):
        with self.lock.gen_wlock():
            self.message = data

    def read(self):
        with self.lock.gen_rlock():
            return self.message