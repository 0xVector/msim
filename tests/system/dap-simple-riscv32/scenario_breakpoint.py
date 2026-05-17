from base import *

adapter = Adapter(int(sys.argv[1]))

# Set breakpoints at the end of the program, we will remove later
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 7 * INSTR_LEN).expect_response()
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 8 * INSTR_LEN).expect_response()

# Remove a non-existent breakpoint
adapter.send(RemoveCodeBreakpointRequest, arg0=RST_VEC + 6 * INSTR_LEN).expect_response(StatusBreakpointNotFoundError,
                                                                                        arg0=RST_VEC + 6 * INSTR_LEN)

# Set a breakpoint, run the program, and check that we stop at the right place
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 3 * INSTR_LEN).expect_response()
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, arg0=DEFAULT_CPU, arg1=RST_VEC + 3 * INSTR_LEN, arg2=StoppedReasonBreakpoint)
adapter.send(ReadPCRequest, arg0=DEFAULT_CPU).expect_response(StatusOk, RST_VEC + 3 * INSTR_LEN)

# Set a breakpoint behind current PC, should never get hit
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC).expect_response()

# Set two breakpoints in front of PC
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 5 * INSTR_LEN).expect_response()
adapter.send(SetCodeBreakpointRequest, arg0=RST_VEC + 6 * INSTR_LEN).expect_response()

# Remove the first breakpoint, should not get hit
adapter.send(RemoveCodeBreakpointRequest, arg0=RST_VEC + 5 * INSTR_LEN).expect_response()
adapter.send(ResumeRequest).expect_response()
adapter.expect_event(StoppedAtEvent, arg0=DEFAULT_CPU, arg1=RST_VEC + 6 * INSTR_LEN, arg2=StoppedReasonBreakpoint)

# Remove the breakpoints we set at the start, should not get hit
adapter.send(RemoveCodeBreakpointRequest, arg0=RST_VEC + 7 * INSTR_LEN).expect_response()
adapter.send(RemoveCodeBreakpointRequest, arg0=RST_VEC + 8 * INSTR_LEN).expect_response()

adapter.send(ResumeRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
