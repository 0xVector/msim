from base import *

adapter = Adapter(int(sys.argv[1]))

SSCRATCH = 0x140
MSCRATCH = 0x340
MTVEC = 0x305
MEPC = 0x341

for csr in (SSCRATCH, MSCRATCH, MTVEC, MEPC):
    adapter.send(ReadCsrRequest, DEFAULT_CPU, csr).expect_response(StatusOk, 0x00)  # CSRs are 0 at reset
    adapter.send(WriteCsrRequest, DEFAULT_CPU, csr, 0x12345678).expect_response(StatusOk)
    adapter.send(WriteCsrRequest, DEFAULT_CPU, csr, 0x12345678).expect_response(StatusOk)  # Write twice
    adapter.send(ReadCsrRequest, DEFAULT_CPU, csr).expect_response(StatusOk, 0x12345678)

# Step a bit and check that the PC is updated, but registers are unchanged
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC + 2 * INSTR_LEN, StoppedReasonStep)
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC + 4 * INSTR_LEN, StoppedReasonStep)

for csr in (SSCRATCH, MSCRATCH, MTVEC, MEPC):
    adapter.send(ReadCsrRequest, DEFAULT_CPU, csr).expect_response(StatusOk, 0x12345678)  # CSRs should be unchanged

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
