import inspect

class command_append_method:
    def __init__(self, command_cls):
        self._command_cls = command_cls
        self.__doc__ = command_cls._append_method_doc

    def __get__(self, obj, cls=None):
        assert obj is not None

        def command_append_func(*args, **kwargs):
            cmd = self._command_cls(*args, **kwargs)
            obj._append_command(cmd)
            return cmd

        ret = command_append_func
        ret.__doc__ = self._command_cls._append_method_doc
        sig = inspect.signature(self._command_cls.__init__)
        sig = sig.replace(parameters=tuple(sig.parameters.values())[1:])
        ret.__signature__ = sig
        return ret

class CommandBase:
    pass