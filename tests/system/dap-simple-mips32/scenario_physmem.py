from base import *

adp = Adapter(int(sys.argv[1]))

nop = struct.pack("<I", NOP_INSTR)  # NOP instruction
assert len(nop) == INSTR_LEN

nops_per_word = 8 // INSTR_LEN
nop_word = int.from_bytes(nop * nops_per_word, byteorder='little')

adp.send(ReadPhysMemoryRequest, arg0=at_phys(0)).expect_response(StatusOk, arg0=nop_word, arg1=nop_word, arg2=nop_word)
adp.send(ReadPhysMemoryRequest, arg0=at_phys(2)).expect_response(StatusOk, arg0=nop_word, arg1=nop_word, arg2=nop_word)

# Unaligned access should also work
nop_bytes = nop * (nops_per_word + 1)  # extra byte of room
shifted = nop_bytes[1:9]  # 8 bytes starting at offset 1
nop_word_shifted = int.from_bytes(shifted, byteorder='little')
adp.send(ReadPhysMemoryRequest, arg0=RST_VEC_PHYS + 1).expect_response(StatusOk, arg0=nop_word_shifted,
                                                                        arg1=nop_word_shifted, arg2=nop_word_shifted)

adp.send(TerminateRequest).expect_response().expect_event(TerminatedEvent)
adp.close()