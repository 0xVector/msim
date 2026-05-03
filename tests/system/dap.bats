#!/usr/bin/env bats

load "common"

@test "DAP RISC-V32: Terminate" {
    msim_dap_run "dap-simple-riscv32" "scenario_terminate.py"
}

@test "DAP RISC-V32: Step" {
    msim_dap_run "dap-simple-riscv32" "scenario_step.py"
}

@test "DAP RISC-V32: Resume" {
    msim_dap_run "dap-simple-riscv32" "scenario_resume.py"
}

@test "DAP RISC-V32: Breakpoint" {
    msim_dap_run "dap-simple-riscv32" "scenario_breakpoint.py"
}

@test "DAP RISC-V32: Register" {
    msim_dap_run "dap-simple-riscv32" "scenario_register.py"
}

@test "DAP RISC-V32: CSR" {
    msim_dap_run "dap-simple-riscv32" "scenario_csr.py"
}

@test "DAP RISC-V32: PC" {
    msim_dap_run "dap-simple-riscv32" "scenario_pc.py"
}

@test "DAP RISC-V32: CPU info" {
    msim_dap_run "dap-simple-riscv32" "scenario_cpu_info.py"
}

@test "DAP RISC-V32: Physical memory" {
    msim_dap_run "dap-simple-riscv32" "scenario_physmem.py"
}

@test "DAP RISC-V32: Bad request" {
    msim_dap_run "dap-simple-riscv32" "scenario_bad_req.py"
}

@test "DAP RISC-V64: Terminate" {
    msim_dap_run "dap-simple-riscv64" "scenario_terminate.py"
}

@test "DAP RISC-V64: Step" {
    msim_dap_run "dap-simple-riscv64" "scenario_step.py"
}

@test "DAP RISC-V64: Resume" {
    msim_dap_run "dap-simple-riscv64" "scenario_resume.py"
}

@test "DAP RISC-V64: Breakpoint" {
    msim_dap_run "dap-simple-riscv64" "scenario_breakpoint.py"
}

@test "DAP RISC-V64: Register" {
    msim_dap_run "dap-simple-riscv64" "scenario_register.py"
}

@test "DAP RISC-V64: CSR" {
    msim_dap_run "dap-simple-riscv64" "scenario_csr.py"
}

@test "DAP RISC-V64: PC" {
    msim_dap_run "dap-simple-riscv64" "scenario_pc.py"
}

@test "DAP RISC-V64: CPU info" {
    msim_dap_run "dap-simple-riscv64" "scenario_cpu_info.py"
}

@test "DAP RISC-V64: Physical memory" {
    msim_dap_run "dap-simple-riscv64" "scenario_physmem.py"
}

@test "DAP RISC-V64: Bad request" {
    msim_dap_run "dap-simple-riscv64" "scenario_bad_req.py"
}

@test "DAP MIPS32: Terminate" {
    msim_dap_run "dap-simple-mips32" "scenario_terminate.py"
}

@test "DAP MIPS32: Step" {
    msim_dap_run "dap-simple-mips32" "scenario_step.py"
}

@test "DAP MIPS32: Resume" {
    msim_dap_run "dap-simple-mips32" "scenario_resume.py"
}

@test "DAP MIPS32: Breakpoint" {
    msim_dap_run "dap-simple-mips32" "scenario_breakpoint.py"
}

@test "DAP MIPS32: Register" {
    msim_dap_run "dap-simple-mips32" "scenario_register.py"
}

@test "DAP MIPS32: CSR" {
    msim_dap_run "dap-simple-mips32" "scenario_csr.py"
}

@test "DAP MIPS32: PC" {
    msim_dap_run "dap-simple-mips32" "scenario_pc.py"
}

@test "DAP MIPS32: CPU info" {
    msim_dap_run "dap-simple-mips32" "scenario_cpu_info.py"
}

@test "DAP MIPS32: Physical memory" {
    msim_dap_run "dap-simple-mips32" "scenario_physmem.py"
}

@test "DAP MIPS32: Bad request" {
    msim_dap_run "dap-simple-mips32" "scenario_bad_req.py"
}
