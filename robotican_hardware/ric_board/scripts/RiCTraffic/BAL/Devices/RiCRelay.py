from BAL.Header.Response.ParamBuildResponse import Rely

__author__ = 'tom1231'
from BAL.Header.Requests.relayRequest import RelayRequest
from BAL.Interfaces.Device import Device
from rospy import Service
from ric_board.srv import Relay, RelayResponse

class RiCRelay(Device):
    def __init__(self, param, relayNum, output):
        Device.__init__(self, param.getRelayName(relayNum), output)
        self._relayNum = relayNum
        Service('%s/setRelay' % self._name, Relay, self.setRelayCallBack)

    def publish(self, data): pass

    def setRelayCallBack(self, req):
        if req.req:
            self._output.write(RelayRequest(self._relayNum, 1).dataTosend())
        else:
            self._output.write(RelayRequest(self._relayNum, 0).dataTosend())
        return RelayResponse(True)

    def getType(self): return Rely
