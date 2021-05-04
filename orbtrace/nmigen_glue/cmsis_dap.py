from migen import *

import nmigen
from nmigen.hdl.rec import DIR_FANIN, DIR_FANOUT, DIR_NONE

from luna.gateware.stream import StreamInterface

from ..nmigen import cmsis_dap

from litex.soc.interconnect.stream import Endpoint

class CMSIS_DAP(Module):
    def __init__(self, pads, wrapper):
        self.source = Endpoint([('data', 8)])
        self.sink = Endpoint([('data', 8)])
        self.is_v2 = Signal()
        self.can = Signal()

        dbgpins = nmigen.Record([
            ('tck_swclk', 1, DIR_FANOUT),
            ('nvdriveen', 1, DIR_FANOUT),
            ('swdwr', 1, DIR_FANOUT),
            ('reseten', 1, DIR_FANOUT),
            ('nvsen', 1, DIR_FANOUT),
            ('tdi', 1, DIR_FANOUT),
            ('tms_swdio', [('i', 1, DIR_FANIN), ('o', 1, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
            ('tdo_swo', [('i', 1, DIR_FANIN)]),
            ('nreset_sense', [('i', 1, DIR_FANIN)]),
        ])

        stream_in = StreamInterface()
        stream_out = StreamInterface()

        is_v2 = nmigen.Signal()

        dap = cmsis_dap.CMSIS_DAP(stream_in, stream_out, dbgpins, is_v2)
        wrapper.m.submodules += dap

        wrapper.connect(self.source.data, stream_in.payload)
        wrapper.connect(self.source.first, stream_in.first)
        wrapper.connect(self.source.last, stream_in.last)
        wrapper.connect(self.source.valid, stream_in.valid)
        wrapper.connect(self.source.ready, stream_in.ready)

        wrapper.connect(self.sink.data, stream_out.payload)
        wrapper.connect(self.sink.first, stream_out.first)
        wrapper.connect(self.sink.last, stream_out.last)
        wrapper.connect(self.sink.valid, stream_out.valid)
        wrapper.connect(self.sink.ready, stream_out.ready)

        wrapper.connect(self.is_v2, is_v2)

        wrapper.connect(self.can, dap.can)

        jtms = TSTriple(8)
        self.specials += jtms.get_tristate(pads.jtms)

        wrapper.connect(pads.jtck, dbgpins.tck_swclk)

        wrapper.connect(jtms.i, dbgpins.tms_swdio.i)
        wrapper.connect(jtms.o, dbgpins.tms_swdio.o)
        wrapper.connect(jtms.oe, dbgpins.tms_swdio.oe)

        wrapper.connect(pads.jtms_dir, dbgpins.swdwr)

        wrapper.connect(pads.reseten, dbgpins.reseten)
        wrapper.connect(pads.nreset_sense, dbgpins.nreset_sense.i)