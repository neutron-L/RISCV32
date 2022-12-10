# TXT := $(wildcard *.txt)
# RESULT_TXT := $(filter-out imem.txt dmem.txt, $(TXT))
cpu: cpu.o
	g++ -O3 -g cpu.o -o cpu

cpu.o: RC32I.cpp
	g++ -c RC32I.cpp -o cpu.o


clean:
	rm -rf *.o cpu
# rm -rf $(RESULT_TXT)
