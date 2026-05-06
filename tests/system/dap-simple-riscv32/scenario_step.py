from base import *

adapter = Adapter(int(sys.argv[1]))

cpu = DEFAULT_CPU

# Step with count 0 should clear the stepping state, but not resume execution
adapter.send(StepRequest, cpu, 0).expect_response()
adapter.send(StepRequest, cpu, 0).expect_response()

for i in range(1, PROGRAM_LEN + 1):
    adapter.send(StepRequest, cpu, 1).expect_response()
    adapter.send(ResumeRequest).expect_response()
    adapter.expect_event(StoppedAtEvent, cpu, RST_VEC + i * INSTR_LEN, StoppedReasonStep)

adapter.send(StepRequest, cpu, 0).expect_response()

# Resume to consume the halt instruction and terminate
adapter.send(ResumeRequest).expect_response(StatusOk)
adapter.expect_event(TerminatedEvent)
adapter.close()
