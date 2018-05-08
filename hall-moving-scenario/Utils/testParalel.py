from multiprocessing import Process, Queue
import time

# Nu merge cu asta pt ca primesc eroarea: RuntimeError: Cannot re-initialize
# CUDA in forked subprocess. To use CUDA with multiprocessing, you must use
# Python 3.4+ and the 'spawn' start method

class Main(object):
    def __init__(self):
        pass

    def method1(self, s, queue):
        time.sleep(10)
        queue.put("method1 %s" % (s))
        return "method1"

    def method2(self, s, queue):
        time.sleep(5)
        queue.put("method2 %s" % (s))
        return "method2"

if __name__ == "__main__":

    m = Main()

    queue1 = Queue()
    proc1 = Process(target=m.method1, args=('Miruna', queue1))
    proc1.start()

    queue2 = Queue()
    proc2 = Process(target=m.method2, args=('cineva', queue2))
    proc2.start()

    proc1.join()
    result1 = queue1.get()
    print('Result of first method: %s' % (result1))

    proc2.join()
    result2 = queue2.get()
    print('Result of second method: %s' % (result2))