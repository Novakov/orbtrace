from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.stream import Endpoint, CombinatorialActor
from litex.build.io import DDRInput

class SWOManchPHY(Module):
    def __init__(self, pads):
        self.source = source = Endpoint([('data', 8)])

        swo_a = Signal()
        swo_b = Signal()

        self.specials += DDRInput(
            clk = ClockSignal(),
            i = pads.swo,
            o1 = swo_a,
            o2 = swo_b,
        )

        self.edgeOutput = Signal()

        byteavail = Signal()
        byteavail_last = Signal()
        byte = Signal(8)

        self.sync += byteavail_last.eq(byteavail)

        self.comb += [
            source.data.eq(byte),
            source.valid.eq(byteavail != byteavail_last),
            #source.first.eq(1),
            #source.last.eq(1),
        ]

        swomanchif = Instance('swoManchIF',
            i_rst = ResetSignal(),
            i_SWOina = swo_a,
            i_SWOinb = swo_b,
            i_clk = ClockSignal(),
            o_edgeOutput = self.edgeOutput,

            o_byteAvail = byteavail,
            o_completeByte = byte,
        )

        self.specials += swomanchif

class PulseLengthCapture(Module):
    def __init__(self, pads, n_bits):
        self.source = source = Endpoint([('count', n_bits), ('level', 1)])

        swo_a = Signal()
        swo_b = Signal()

        self.specials += DDRInput(
            clk = ClockSignal(),
            i = pads.swo,
            o1 = swo_a,
            o2 = swo_b,
        )

        state = Signal(3)
        self.sync += state.eq(Cat(swo_b, swo_a, state[0]))

        add_2 = Signal()
        output_0 = Signal()
        output_1 = Signal()

        self.comb += Case(state, {
            # Two more samples equal to prev.
            0b000: add_2.eq(1),
            0b111: add_2.eq(1),

            # Two samples opposite of prev.
            0b011: output_0.eq(1),
            0b100: output_0.eq(1),

            # One sample equal to prev and one opposite.
            0b001: output_1.eq(1),
            0b110: output_1.eq(1),

            # Glitch or short pulse, treat it as two samples opposite of prev.
            0b010: output_0.eq(1),
            0b101: output_0.eq(1),
        })

        count = Signal(n_bits)

        self.sync += [
            source.level.eq(state[2]),
            source.valid.eq(0),

            If(add_2 & ~count[-1],
                count.eq(count + 2),
            ),

            If(output_0,
                source.count.eq(count),
                source.valid.eq(1),
                count.eq(2),
            ),

            If(output_1,
                source.count.eq(count + 1),
                source.valid.eq(1),
                count.eq(1),
            ),
        ]

class CountToByte(CombinatorialActor):
    def __init__(self, n_bits):
        self.sink = sink = Endpoint([('count', n_bits), ('level', 1)])
        self.source = source = Endpoint([('data', 8)])

        self.comb += source.data.eq(sink.count)

        super().__init__()


class ManchesterDecoder(Module):
    def __init__(self, n_bits):
        self.sink = sink = Endpoint([('count', n_bits), ('level', 1)])
        self.source = source = Endpoint([('data', 1)])

        self.submodules.fsm = fsm = FSM()

        short_threshold = Signal(n_bits) # 3/4 bit time
        long_threshold = Signal(n_bits)  # 5/4 bit time

        fsm.act('IDLE',
            sink.ready.eq(1),

            If(sink.valid & sink.level & ~sink.count[-1],
                NextState('CENTER'),
                NextValue(short_threshold, sink.count + (sink.count >> 1)),
                NextValue(long_threshold, (sink.count << 1) + (sink.count >> 1)),
                NextValue(source.first, 1),
            ),
        )

        short = Signal()
        long = Signal()
        extra_long = Signal()

        capture = Signal()

        self.comb += [
            short.eq(sink.count <= short_threshold),
            extra_long.eq(sink.count > long_threshold),
            long.eq(~short & ~extra_long),
        ]

        fsm.act('CENTER',
            sink.ready.eq(1),

            # Long pulse from bit center takes us to the next bit center; capture.
            If(sink.valid & long,
                capture.eq(1),
            ),

            # Short pulse from bit center takes us to bit edge.
            If(sink.valid & short,
                NextState('EDGE'),
            ),

            # Extra long pulse is either end bit or error.
            If(sink.valid & extra_long,
                NextState('IDLE'),
            ),
        )

        fsm.act('EDGE',
            sink.ready.eq(1),

            # Short pulse from bit edge takes us to bit center; capture.
            If(sink.valid & short,
                capture.eq(1),
                NextState('CENTER'),
            ),

            # Long or extra long pulse from bit edge is either end bit or error.
            If(sink.valid & (long | extra_long),
                NextState('IDLE'),
            )
        )

        self.comb += If(capture,
            source.data.eq(sink.level),
            source.valid.eq(1),
        )

        self.sync += If(source.ready & source.valid,
            source.first.eq(0),
        )

class BitsToBytes(Module):
    def __init__(self):
        self.sink = sink = Endpoint([('data', 1)])
        self.source = source = Endpoint([('data', 8)])

        sr = Signal(9)

        self.comb += [
            source.valid.eq(sr[0]),
            source.data.eq(sr[1:]),
            sink.ready.eq(~source.valid),
        ]

        self.sync += [
            If(sink.valid & sink.ready,
                sr.eq(Cat(sr[1:], sink.data)),

                If(sink.first,
                    sr.eq(Cat(C(0x80, 8), sink.data)),
                ),
            ),

            If(source.valid & source.ready,
                sr.eq(0x100),
            )
        ]
