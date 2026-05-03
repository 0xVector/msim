from base import *

adapter = Adapter(int(sys.argv[1]))

# Set breakpoints at the end, we will remove later
adapter.send(SetCodeBreakpointRequest, RST_VEC + 7 * INSTR_LEN).expect_response()
adapter.send(SetCodeBreakpointRequest, RST_VEC + 8 * INSTR_LEN).expect_response()

# Remove a non-existent breakpoint
adapter.send(RemoveCodeBreakpointRequest, RST_VEC + 6 * INSTR_LEN).expect_response(StatusUnspecifiedError)

adapter.send(SetCodeBreakpointRequest, RST_VEC + 3 * INSTR_LEN).expect_response()
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC + 3 * INSTR_LEN, StoppedReasonBreakpoint, )

adapter.send(SetCodeBreakpointRequest, RST_VEC).expect_response()  # BP should not be hit

adapter.send(SetCodeBreakpointRequest, RST_VEC + 5 * INSTR_LEN).expect_response()
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, DEFAULT_CPU, RST_VEC + 5 * INSTR_LEN, StoppedReasonBreakpoint, )

# Remove the breakpoints we set at the start, should not get hit
adapter.send(RemoveCodeBreakpointRequest, RST_VEC + 7 * INSTR_LEN).expect_response()
adapter.send(RemoveCodeBreakpointRequest, RST_VEC + 8 * INSTR_LEN).expect_response()

adapter.send(ResumeRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
