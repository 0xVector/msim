from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
