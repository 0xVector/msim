from base import *

adapter = Adapter(int(sys.argv[1]))

# Advance the PC by one instruction and check that it has advanced correctly only for CPU0, not CPU1
adapter.send(WritePCRequest, CPU0, RST_VEC + INSTR_LEN).expect_response(StatusOk)
adapter.send(ReadPCRequest, CPU0).expect_response(StatusOk, RST_VEC + INSTR_LEN)
adapter.send(ReadPCRequest, CPU1).expect_response(StatusOk, RST_VEC)

# Step once CPU1
adapter.send(StepRequest, CPU1, 4).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, CPU1, RST_VEC + 4 * INSTR_LEN, StoppedReasonStep)
adapter.send(ReadPCRequest, CPU0).expect_response(StatusOk, RST_VEC + 5 * INSTR_LEN)
adapter.send(ReadPCRequest, CPU1).expect_response(StatusOk, RST_VEC + 4 * INSTR_LEN)

# Rollback the PC
adapter.send(WritePCRequest, CPU0, RST_VEC).expect_response(StatusOk)
adapter.send(WritePCRequest, CPU1, RST_VEC + 2 * INSTR_LEN).expect_response(StatusOk)

#
adapter.send(StepRequest, CPU0, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, CPU0, RST_VEC + 2 * INSTR_LEN, StoppedReasonStep)
adapter.send(ReadPCRequest, CPU0).expect_response(StatusOk, RST_VEC + 2 * INSTR_LEN)
adapter.send(ReadPCRequest, CPU1).expect_response(StatusOk, RST_VEC + 4 * INSTR_LEN)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
