from base import *

adapter = Adapter(int(sys.argv[1]))

BADVADDR = 0x08
ENTRYHI = 0x0a
CAUSE = 0x0d
EPC = 0x0e
CONFIG = 0x10
LLADDR = 0x11

for csr in (BADVADDR, ENTRYHI, CAUSE, EPC, CONFIG, LLADDR):
    adapter.send(WriteCsrRequest, DEFAULT_CPU, csr, 0x00).expect_response(StatusOk)
    adapter.send(ReadCsrRequest, DEFAULT_CPU, csr).expect_response(StatusOk, 0x00)
    adapter.send(WriteCsrRequest, DEFAULT_CPU, csr, 0x12345678).expect_response(StatusOk)
    adapter.send(WriteCsrRequest, DEFAULT_CPU, csr, 0x12345678).expect_response(StatusOk)  # Write twice
    adapter.send(ReadCsrRequest, DEFAULT_CPU, csr).expect_response(StatusOk, 0x12345678)

# Step a bit and check that the PC is updated, but registers are unchanged
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + 2 * INSTR_LEN, StoppedReasonStep)
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + 4 * INSTR_LEN, StoppedReasonStep)

for csr in (BADVADDR, ENTRYHI, CAUSE, EPC, CONFIG, LLADDR):
    adapter.send(ReadCsrRequest, DEFAULT_CPU, csr).expect_response(StatusOk, 0x12345678)  # CSRs should be unchanged

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
