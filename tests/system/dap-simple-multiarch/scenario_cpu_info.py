from base import *

adapter = Adapter(int(sys.argv[1]))

adapter.send(GetCpuInfoRequest, arg0=CPU0).expect_response(StatusOk, 0x02)  # 0x02 is RISCV32 identifier
adapter.send(GetCpuInfoRequest, arg0=CPU1).expect_response(StatusOk, 0x03)  # 0x03 is RISCV64 identifier
adapter.send(GetConfigRequest).expect_response(StatusOk, 2)  # Expect 2 CPUs in the system

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
