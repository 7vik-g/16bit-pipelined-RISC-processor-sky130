# Caravel User Project

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

# 16bit-pipelined-RISC-processor-using-Verilog-HDL
- Developed a new Reduced Instruction Set with least possible no. of instructions in it such that the designed Processor can be used to implement almost any Function by combination of instructions
- Implemented the Processor using Verilog HDL in Xilinx Vivado
- In the process of Designing the processor, I have implemented many smaller components like 16-bit adder, Decoders, Register file, Stack Pointer, Program Counter, ALU and finally integrating everything as processor by designing Control Unit, buses, instruction registers, etc
- Using the concept of pipelining divided execution of instruction in several stages such that approximately one instruction is executed for every clock cycle


Processor schematic
![processor_schematic1](https://user-images.githubusercontent.com/97520594/202527693-ed42324d-2df1-46be-a477-0ec00e922596.png)
![processor_schematic2](https://user-images.githubusercontent.com/97520594/202527696-e68d08b8-25bb-48c9-8bbf-ce217adde9e2.png)


ALU schematic
![ALU_schematic](https://user-images.githubusercontent.com/97520594/202527680-5959926b-3e10-4c98-9290-ae1fe6a81e47.png)


Control unit schematic
![Control_unit1_schematic](https://user-images.githubusercontent.com/97520594/202527687-0c7c778b-efc2-4fdf-9011-4a955223990d.png)
![Control_unit2_schematic](https://user-images.githubusercontent.com/97520594/202527689-11b1afc1-992a-4ee8-9a9c-ee918d242417.png)


ISA
![ISA 0](https://user-images.githubusercontent.com/97520594/209435151-2c232ca6-db8d-407c-b4bf-b649aa638251.jpeg)
![ISA 1](https://user-images.githubusercontent.com/97520594/209435154-c3989bb7-9cc3-49b3-b393-6b0960cd2947.jpeg)

