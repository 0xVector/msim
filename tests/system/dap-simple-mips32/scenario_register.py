from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, 0).expect_response(StatusOk, 0x00)  # x0 is always 0
# Writing 0 to x0 should succeed but have no effect
adapter.send(WriteGeneralRegisterRequest, DEFAULT_CPU, 0).expect_response(StatusOk)
adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, 0).expect_response(StatusOk, 0x00)

adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, 1).expect_response(StatusOk, 0x00)  # x1 (ra) is 0 at reset
adapter.send(WriteGeneralRegisterRequest, DEFAULT_CPU, 1, 0x12345678).expect_response(StatusOk)
adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, 1).expect_response(StatusOk, 0x12345678)

# Invalid register index
for i in range(REG_COUNT, REG_COUNT + 50, 10):
    adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, i).expect_response(StatusRegisterNotFoundError, arg0=i)

# Write and read all registers with a known pattern
for i in range(1, REG_COUNT):
    adapter.send(WriteGeneralRegisterRequest, DEFAULT_CPU, i, i * 0x01234567).expect_response(StatusOk)
    adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, i).expect_response(StatusOk, i * 0x01234567)

# Write all registers with a different pattern
for i in range(1, REG_COUNT):
    adapter.send(WriteGeneralRegisterRequest, DEFAULT_CPU, i, i * 0x07654321).expect_response(StatusOk)

# Step a bit and check that the PC is updated, but registers are unchanged
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + 2 * INSTR_LEN, StoppedReasonStep)
adapter.send(StepRequest, DEFAULT_CPU, 2).expect_response(StatusOk)
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC_VIRT + 4 * INSTR_LEN, StoppedReasonStep)

# Check that registers still have the same value after stepping
for i in range(1, REG_COUNT):
    adapter.send(ReadGeneralRegisterRequest, DEFAULT_CPU, i).expect_response(StatusOk, i * 0x07654321)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
