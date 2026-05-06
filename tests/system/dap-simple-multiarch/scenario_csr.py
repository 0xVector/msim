from base import *

adapter = Adapter(int(sys.argv[1]))

SSCRATCH = 0x140
MSCRATCH = 0x340
MTVEC = 0x305
MEPC = 0x341

for csr in (SSCRATCH, MSCRATCH, MTVEC, MEPC):
    adapter.send(WriteCsrRequest, CPU0, csr, 0x12345678).expect_response(StatusOk)
    adapter.send(WriteCsrRequest, CPU1, csr, 0x12000678).expect_response(StatusOk)
    adapter.send(ReadCsrRequest, CPU0, csr).expect_response(StatusOk, 0x12345678)
    adapter.send(ReadCsrRequest, CPU1, csr).expect_response(StatusOk, 0x12000678)

# Step a bit and check that the PC is updated, but registers are unchanged
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC + 2 * INSTR_LEN, StoppedReasonStep)

for csr in (SSCRATCH, MSCRATCH, MTVEC, MEPC):
    adapter.send(ReadCsrRequest, CPU0, csr).expect_response(StatusOk, 0x12345678)  # CSRs should be unchanged
    adapter.send(ReadCsrRequest, CPU1, csr).expect_response(StatusOk, 0x12000678)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
