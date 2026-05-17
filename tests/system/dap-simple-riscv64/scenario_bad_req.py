from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(0xff).expect_response(StatusUnsupportedRequestError, 0xff)
adapter.send(0xf0).expect_response(StatusUnsupportedRequestError, 0xf0)
adapter.send(0xff).expect_response(StatusUnsupportedRequestError, 0xff)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
