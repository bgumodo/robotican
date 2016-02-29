__author__ = 'tom1231'

class Device:
    def __init__(self, name, output):
        self._name = name
        self._output = output

    def publish(self, data): raise NotImplementedError

    def getType(self): raise NotImplementedError
