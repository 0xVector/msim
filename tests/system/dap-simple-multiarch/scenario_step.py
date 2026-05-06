from base import *

adapter = Adapter(int(sys.argv[1]))

# Step with count 0 should clear the stepping state, but not resume execution
adapter.send(StepRequest, CPU0, 0).expect_response()
adapter.send(StepRequest, CPU1, 0).expect_response()

# Step CPU0 by 1 instruction and CPU1 by 2 instructions, then resume both
for i in range(1, PROGRAM_LEN + 1, 2):
    adapter.send(StepRequest, CPU0, 1).expect_response()
    adapter.send(StepRequest, CPU1, 2).expect_response()
    adapter.send(ResumeRequest).expect_response(StatusOk)
    adapter.expect_event(StoppedAtEvent, CPU0, RST_VEC + i * INSTR_LEN, StoppedReasonStep)
    adapter.send(ResumeRequest).expect_response(StatusOk)
    adapter.expect_event(StoppedAtEvent, CPU1, RST_VEC + (i + 1) * INSTR_LEN, StoppedReasonStep)

adapter.send(StepRequest, CPU0, 0).expect_response()
adapter.send(StepRequest, CPU1, 0).expect_response()

# Resume to consume the halt instruction and terminate
adapter.send(ResumeRequest).expect_response(StatusOk)
adapter.expect_event(TerminatedEvent)
adapter.close()
