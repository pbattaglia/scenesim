"""Combomethod class."""
from functools import wraps


class combomethod(object):
    """ Descriptor class that can be used as a decorator inside class
    definitions to make a method both a class method and instance
    method.

    A typical use case is:

    @combomethod
    def func(combo):
        if isinstance(combo, type):
            # Do class stuff.
        else:
            # Do instance stuff.

    Originally found at:

    http://stackoverflow.com/questions/2589690/creating-a-method-that-is-simultaneously-an-instance-and-class-method
    """

    def __init__(self, method):
        self.method = method

    def __get__(self, obj, objtype):
        @wraps(self.method)
        def _wrapper(*args, **kwargs):
            if obj is not None:
                return self.method(obj, *args, **kwargs)
            else:
                return self.method(objtype, *args, **kwargs)
        return _wrapper
