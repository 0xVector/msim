from base import *

adapter = Adapter(int(sys.argv[1]))

nop = struct.pack("<I", NOP_INSTR)  # NOP instruction
assert len(nop) == INSTR_LEN

nops_per_word = 8 // INSTR_LEN
nop_word = int.from_bytes(nop * nops_per_word, byteorder='little')

adapter.send(ReadPhysMemoryRequest, RST_VEC_PHYS).expect_response(StatusOk, nop_word, nop_word, nop_word)
adapter.send(ReadPhysMemoryRequest, RST_VEC_PHYS + 2 * INSTR_LEN).expect_response(StatusOk, nop_word, nop_word, nop_word)

# Unaligned access should also work
nop_bytes = nop * (nops_per_word + 1)  # extra byte of room
shifted = nop_bytes[1:9]  # 8 bytes starting at offset 1
nop_word_shifted = int.from_bytes(shifted, byteorder='little')
adapter.send(ReadPhysMemoryRequest, RST_VEC_PHYS + 1).expect_response(StatusOk, nop_word_shifted, nop_word_shifted,
                                                                 nop_word_shifted)

adapter.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adapter.close()
