# RISC-V CPU (CHIP.v)
## Introduction:
My CPU supports 32-bit multiplication and 12 single-cycle instructions in the RISC-V instruction set including:
* auipc
* jal
* jalr
* beq
* lw
* sw
* addi
* slti
* slli
* srli
* add
* sub
## How it works:
* I translated three different functions written in Python(.py) in "fact", "leaf" and "hw1" into RISC-V instructions(.s) and then translated RISC-V instructions into machine codes(.txt).
* Users could use 'Final_tb.v' to test the machine codes with my CPU:
<pre> ncverilog Final_tb.v +define+fact +access+r </pre>
P.S. The license for ncverilog is required.
