from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(ReadPCRequest).expect_response(StatusOk, arg0=RST_VEC)
# Make sure its actually stopped and not running yet
adapter.send(ReadPCRequest).expect_response(StatusOk, arg0=RST_VEC)
adapter.send(PauseRequest).expect_response(StatusOk)
# We don't expect a StoppedAtEvent here, because the CPU is already stopped and shouldn't generate a new event
adapter.send(ReadPCRequest).expect_response(StatusOk, arg0=RST_VEC)

adapter.send(ResumeRequest).expect_response(StatusOk)

adapter.expect_event(TerminatedEvent)
adapter.close()
