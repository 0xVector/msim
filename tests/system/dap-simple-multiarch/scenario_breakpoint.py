from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(SetCodeBreakpointRequest, RST_VEC + 3 * INSTR_LEN).expect_response()
adapter.send(ResumeRequest).expect_response()
# resumed both, as both are at the same PC, should get two events (two hits)
adapter.expect_event(StoppedAtEvent, arg0=CPU0, arg1=RST_VEC + 3 * INSTR_LEN, arg2=StoppedReasonBreakpoint)
adapter.expect_event(StoppedAtEvent, arg0=CPU1, arg1=RST_VEC + 3 * INSTR_LEN, arg2=StoppedReasonBreakpoint)

adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 5 * INSTR_LEN).expect_response()
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 6 * INSTR_LEN).expect_response()

adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, arg0=CPU0, arg1=RST_VEC + 5 * INSTR_LEN, arg2=StoppedReasonBreakpoint)
adapter.expect_event(StoppedAtEvent, arg0=CPU1, arg1=RST_VEC + 5 * INSTR_LEN, arg2=StoppedReasonBreakpoint)

adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, arg0=CPU0, arg1=RST_VEC + 6 * INSTR_LEN, arg2=StoppedReasonBreakpoint)
adapter.expect_event(StoppedAtEvent, arg0=CPU1, arg1=RST_VEC + 6 * INSTR_LEN, arg2=StoppedReasonBreakpoint)

adapter.send(ResumeRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
