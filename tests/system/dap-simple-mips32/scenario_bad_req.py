from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(0xff).expect_response(StatusUnsupportedRequestError)
adapter.send(0xf0).expect_response(StatusUnsupportedRequestError)
adapter.send(0xff).expect_response(StatusUnsupportedRequestError)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
