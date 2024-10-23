#!/bin/bash

python mem_constructor.py < memory.txt > the_memory.mem
mv the_memory.mem ./ca4pro/
python mem_constructor.py < memory.txt > the_memory.mem
mv the_memory.mem ./ca4pro/work/
python inst_mem_constructor.py < inst_mem.txt > the_inst_mem.mem
python binary_to_hex.py < inst_mem.txt > hex_inst.mem
mv the_inst_mem.mem ./ca4pro/
python inst_mem_constructor.py < inst_mem.txt > the_inst_mem.mem
mv the_inst_mem.mem ./ca4pro/work/