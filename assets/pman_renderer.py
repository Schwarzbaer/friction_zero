
import functools
def get_renderer():
    modname = 'pman.basicrenderer'
    attrs = ('BasicRenderer',)
    module =  __import__(modname, fromlist=['__name__'], level=0)
    return functools.reduce(getattr, attrs, module)
