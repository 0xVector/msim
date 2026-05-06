from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(ReadPCRequest, DEFAULT_CPU).expect_response(StatusOk, RST_VEC_VIRT)

# Advance the PC by one instruction and check that it has advanced correctly
adapter.send(WritePCRequest, DEFAULT_CPU, RST_VEC_VIRT + INSTR_LEN).expect_response(StatusOk)
adapter.send(ReadPCRequest, DEFAULT_CPU).expect_response(StatusOk, RST_VEC_VIRT + INSTR_LEN)

# Step once and check that the PC has advanced by another instruction
adapter.send(StepRequest, DEFAULT_CPU, 1).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + 2 * INSTR_LEN, StoppedReasonStep)
adapter.send(ReadPCRequest, DEFAULT_CPU).expect_response(StatusOk, RST_VEC_VIRT + 2 * INSTR_LEN)

# Rollback the PC
adapter.send(WritePCRequest, DEFAULT_CPU, RST_VEC_VIRT).expect_response(StatusOk)
adapter.send(ReadPCRequest, DEFAULT_CPU).expect_response(StatusOk, RST_VEC_VIRT)

# Step through the entire program and check that the PC advances correctly each time, then reset it back to the start
half = PROGRAM_LEN // 2
rest = PROGRAM_LEN - half
for _ in range(5):
    # Step through the first half of the program and check that the PC has advanced correctly
    adapter.send(StepRequest, DEFAULT_CPU, half).expect_response(StatusOk)
    adapter.send(ResumeRequest).expect_response()
    adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + half * INSTR_LEN, StoppedReasonStep)
    # Step through the rest of the program and check that the PC has advanced correctly
    adapter.send(StepRequest, DEFAULT_CPU, rest).expect_response(StatusOk)
    adapter.send(ResumeRequest).expect_response()
    adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + PROGRAM_LEN * INSTR_LEN, StoppedReasonStep)
    # Rollback the PC
    adapter.send(WritePCRequest, DEFAULT_CPU, RST_VEC_VIRT).expect_response(StatusOk)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
