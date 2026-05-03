from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(GetCpuInfoRequest).expect_response(StatusOk, 0x03)  # 0x02 is RISCV64 identifier
adapter.send(GetConfigRequest).expect_response(StatusOk, 1)  # Expect 1 CPU in the system

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
