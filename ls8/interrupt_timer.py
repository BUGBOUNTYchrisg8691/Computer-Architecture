"""
InterruptTimer class
"""
from threading import Timer
from functools import partial


class InterruptTimer:
    """
    Accepts a func and an interval and run that function
    every n seconds, specified by the interval.
    """
    def __init__(self, interval, func, args=None, kwargs=None):
        """
        Runs the function at a specified interval with given args.
        """
        if args is None:
            args = []
        if kwargs is None:
            kwargs = {}
        self.interval = interval
        self.func = partial(func, *args, **kwargs)
        self.running = False
        self.__timer = None

    def __call__(self):
        """
        Handler func for calling the partial and continuing.
        """
        self.running = False
        self.start()
        self.func()

    def start(self):
        """
        Starts the interval and lets it run.
        """
        if self.running:
            return

        self.__timer = Timer(self.interval, self)
        self.__timer.start()
        self.running = True

    def stop(self):
        """
        Cancel the interval (no more func calls).
        """
        if self.__timer:
            self.__timer.cancel()

        self.running = False
        self.__timer = None
