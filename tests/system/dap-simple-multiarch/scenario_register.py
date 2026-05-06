from base import *

adapter = Adapter(int(sys.argv[1]))

# Check that write does not modify other CPU's registers
adapter.send(ReadGeneralRegisterRequest, arg0=CPU1, arg1=1).expect_response(StatusOk, 0x00)  # x1 (ra) is 0 at reset
adapter.send(WriteGeneralRegisterRequest, arg0=CPU0, arg1=1, arg2=0x12345678).expect_response(StatusOk)
adapter.send(ReadGeneralRegisterRequest, arg0=CPU0, arg1=1).expect_response(StatusOk, 0x12345678)
adapter.send(ReadGeneralRegisterRequest, arg0=CPU1, arg1=1).expect_response(StatusOk, 0x00)

# Invalid register index
for i in (REG_COUNT, REG_COUNT + 4546, REG_COUNT + 144104):
    adapter.send(ReadGeneralRegisterRequest, CPU0, REG_COUNT).expect_response(StatusUnspecifiedError)
    adapter.send(ReadGeneralRegisterRequest, CPU1, REG_COUNT).expect_response(StatusUnspecifiedError)

# Write and read all registers with a known pattern
for i in range(1, REG_COUNT):
    adapter.send(WriteGeneralRegisterRequest, CPU0, i, i * 0x01234567).expect_response(StatusOk)
    adapter.send(ReadGeneralRegisterRequest, CPU0, i).expect_response(StatusOk, i * 0x01234567)
    adapter.send(WriteGeneralRegisterRequest, CPU1, i, i * 0x0123456789abcdef).expect_response(StatusOk)
    adapter.send(ReadGeneralRegisterRequest, CPU1, i).expect_response(StatusOk, i * 0x0123456789abcdef)

# Write all registers with a different pattern
for i in range(1, REG_COUNT):
    adapter.send(WriteGeneralRegisterRequest, CPU0, i, i * 0x07654321).expect_response(StatusOk)
    adapter.send(WriteGeneralRegisterRequest, CPU1, i, i * 0x00edcba987654321).expect_response(StatusOk)

# Step a bit and check that the PC is updated, but registers are unchanged
adapter.send(StepRequest, CPU0, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, CPU0, RST_VEC + 2 * INSTR_LEN, StoppedReasonStep)
adapter.send(ReadPCRequest, CPU0).expect_response(StatusOk, RST_VEC + 2 * INSTR_LEN)
adapter.send(ReadPCRequest, CPU1).expect_response(StatusOk, RST_VEC + 2 * INSTR_LEN)

# Check that registers still have the same value after stepping
for i in range(1, REG_COUNT):
    adapter.send(ReadGeneralRegisterRequest, CPU0, i).expect_response(StatusOk, i * 0x07654321)
    adapter.send(ReadGeneralRegisterRequest, CPU1, i).expect_response(StatusOk, i * 0x00edcba987654321)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
