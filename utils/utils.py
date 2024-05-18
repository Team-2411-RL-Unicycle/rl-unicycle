"""Generic utils."""

# Decorator
def call_super_first(method):
    def wrapper(self, *args, **kwargs):
        super_class = super(self.__class__, self) # get the super class
        getattr(super_class, method.__name__)(*args, **kwargs) # execute the method
        return method(self, *args, **kwargs)
    return wrapper