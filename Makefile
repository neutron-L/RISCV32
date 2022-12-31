cpu: cpu.o
	g++ -O3 -g cpu.o -o cpu

cpu.o: RV32I.cpp
	g++ -c RV32I.cpp -o cpu.o


clean:
	rm -rf *.o cpu
# rm -rf $(RESULT_TXT)
