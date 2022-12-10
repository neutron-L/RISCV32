#include <iostream>
#include <string>
#include <vector>
#include <bitset>
#include <fstream>
#include <cassert>
using namespace std;

#define MemSize 1000 // memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.

struct IFStruct
{
    bitset<32> PC;
    bool nop = false;
};

struct IDStruct
{
    bitset<32> PC;

    bitset<32> Instr;
    bool nop = true;
};

struct EXStruct
{
    bitset<32> PC;

    bitset<32> Read_data1;
    bitset<32> Read_data2;

    bitset<5> Rs;
    bitset<5> Rt;

    bitset<32> Imm;
    bitset<4> inst; // instruction[30,14-12]
    bitset<5> Write_register;

    bitset<2> branch_op;

    /* WB */
    bitset<1> MemtoReg;
    bool RegWrite;

    /* MEM */
    bool MemRead;
    bool MemWrite;

    /* EXE */
    bitset<2> alu_op;
    bitset<1> alu_src1;
    bitset<2> alu_src2;

    bool nop = true;
};

struct MEMStruct
{
    bitset<32> ALU_result;
    bitset<32> Store_data;
    bitset<5> Rs;
    bitset<5> Rt;
    bitset<5> Write_register;

    /* Mem */
    bool MemRead;
    bool MemWrite;

    /* WB */
    bitset<1> MemtoReg;
    bool RegWrite;

    bool nop = true;
};

struct WBStruct
{
    bitset<32> WriteData;
    bitset<32> ALU_result;
    bitset<5> Rs;
    bitset<5> Rt;
    bitset<5> Write_register;

    /* WB */
    bitset<1> MemtoReg;
    bool RegWrite;

    bool nop = true;
};

struct stateStruct
{
    IFStruct IF;
    IDStruct ID;
    EXStruct EX;
    MEMStruct MEM;
    WBStruct WB;
};

class InsMem
{
public:
    string id, ioDir;
    InsMem(string name, string ioDir)
    {
        id = name;
        IMem.resize(MemSize);
        ifstream imem;
        string line;
        int i = 0;
        imem.open(ioDir + "/imem.txt");
        if (imem.is_open())
        {
            while (getline(imem, line))
            {
                IMem[i] = bitset<8>(line);
                i++;
            }
            number = i;
        }
        else
            cout << "Unable to open IMEM input file.";
        imem.close();
    }

    bitset<32> readInstr(bitset<32> ReadAddress)
    {
        // read instruction memory
        uint32_t idx = ReadAddress.to_ulong();
        assert(idx + 3 < MemSize);
        uint32_t one = (IMem[idx].to_ulong() << 24) & 0xFF000000;
        uint32_t two = (IMem[idx + 1].to_ulong() << 16) & 0x00FF0000;
        uint32_t thr = (IMem[idx + 2].to_ulong() << 8) & 0x0000FF00;
        uint32_t fur = IMem[idx + 3].to_ulong() & 0x000000FF;
        bitset<32> instr(one | two | thr | fur);
        return instr;
    }

    int get_number()
    {
        return number / 4;
    }

private:
    vector<bitset<8>> IMem;
    int number;
};

class DataMem
{
public:
    string id, opFilePath, ioDir;
    DataMem(string name, string ioDir) : id{name}, ioDir{ioDir}
    {
        DMem.resize(MemSize);
        opFilePath = ioDir + "/" + name + "_DMEMResult.txt";
        ifstream dmem;
        string line;
        int i = 0;
        dmem.open(ioDir + "/dmem.txt");
        if (dmem.is_open())
        {
            while (getline(dmem, line))
            {
                DMem[i] = bitset<8>(line);
                ++i;
            }
        }
        else
            cout << "Unable to open DMEM input file.";
        dmem.close();
    }

    bitset<32> readDataMem(bitset<32> Address)
    {
        // read data memory
        uint32_t idx = Address.to_ulong();
        assert(idx + 3 < MemSize);
        uint32_t one = (DMem[idx].to_ulong() << 24) & 0xFF000000;
        uint32_t two = (DMem[idx + 1].to_ulong() << 16) & 0x00FF0000;
        uint32_t thr = (DMem[idx + 2].to_ulong() << 8) & 0x0000FF00;
        uint32_t fur = DMem[idx + 3].to_ulong() & 0x000000FF;
        bitset<32> data(one | two | thr | fur);
        return data;
    }

    void writeDataMem(bitset<32> Address, bitset<32> WriteData)
    {
        // write into memory
        uint32_t idx = Address.to_ulong();
        assert(idx + 3 < MemSize);
        uint32_t data = WriteData.to_ulong();
        uint32_t one = (data & 0xFF000000) >> 24;
        uint32_t two = (data & 0x00FF0000) >> 16;
        uint32_t thr = (data & 0x0000FF00) >> 8;
        uint32_t fur = (data & 0x000000FF);
        DMem[idx] = bitset<8>(one);
        DMem[idx + 1] = bitset<8>(two);
        DMem[idx + 2] = bitset<8>(thr);
        DMem[idx + 3] = bitset<8>(fur);
    }

    void outputDataMem()
    {
        ofstream dmemout;
        dmemout.open(opFilePath, std::ios_base::trunc);
        if (dmemout.is_open())
        {
            for (int j = 0; j < MemSize; j++)
            {
                dmemout << DMem[j] << endl;
            }
        }
        else
            cout << "Unable to open " << id << " DMEM result file." << endl;
        dmemout.close();
    }

private:
    vector<bitset<8>> DMem;
};

class RegisterFile
{
public:
    string outputFile;
    RegisterFile(string ioDir) : outputFile{ioDir + "RFResult.txt"}
    {
        Registers.resize(32);
        Registers[0] = bitset<32>(0);
    }

    bitset<32> readRF(bitset<5> Reg_addr)
    {
        // Fill in
        uint32_t idx = Reg_addr.to_ulong();
        return Registers[idx];
    }

    void writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data)
    {
        // Fill in
        uint32_t idx = Reg_addr.to_ulong();
        if (idx != 0)
            Registers[idx] = Wrt_reg_data;
    }

    void outputRF(int cycle)
    {
        ofstream rfout;
        if (cycle == 0)
            rfout.open(outputFile, std::ios_base::trunc);
        else
            rfout.open(outputFile, std::ios_base::app);
        if (rfout.is_open())
        {
            rfout << "State of RF after executing cycle:\t" << cycle << endl;
            for (int j = 0; j < 32; j++)
            {
                rfout << Registers[j] << endl;
            }
        }
        else
            cout << "Unable to open RF output file." << endl;
        rfout.close();
    }

private:
    vector<bitset<32>> Registers;
};

class Core
{
public:
    InsMem &ext_imem;
    DataMem &ext_dmem;
    RegisterFile myRF;
    struct stateStruct state, nextState;
    uint32_t cycle = 0;
    bool halted = false;
    string ioDir;

    Core(string ioDir, InsMem &imem, DataMem &dmem) : myRF(ioDir), ioDir{ioDir}, ext_imem{imem}, ext_dmem{dmem}
    {
    }

    int get_instruction_number()
    {
        return ext_imem.get_number();
    }

    virtual void step() {}

    virtual void printState() {}

protected:
    // ALU control
    /*
    input: 2-bit ALUOp;instruction[30,14-12]
    output: 3-bit ALU input control signal
    */
    static bitset<3> ALUControl(bitset<2> &alu_op, bitset<4> &inst)
    {
        switch (alu_op.to_ulong())
        {
        case 0b00:
            return {0b000};
        case 0b01:
            return {0b001};
        case 0b10:
        {
            if (!inst[3])
                return {inst.to_ulong()};
            else
                return {0b001};
        }
        default:
            cerr << "invalid alu_op " << alu_op << "!" << endl;
            assert(0);
        }
    }

    // branch_op control
    /*
        input: 2-bit Branch ALUOp and 3-bit funct3 field
        output: 2-bit Branch ALU control signal
    */
    static bitset<2> BranchControl(bitset<2> &branch_op, bitset<3> &funct3)
    {
        if (!(branch_op[0] ^ branch_op[1]))
            return branch_op;
        else
        {
            assert(branch_op[0] == 0); // branch_alu_op must be 10
            if (!funct3[0])
                return {0b01};
            else
                return {0b10};
        }
    }

    // Control Unit
    /*
        input: instruction[31-0]
        set the field in state.EX
    */
    void control(bitset<7> &opcode)
    {
        nextState.EX.alu_src1[0] = opcode[2];
        nextState.EX.alu_src2[1] = opcode[6] & opcode[2];
        nextState.EX.alu_src2[0] = ~opcode[6] & (~opcode[5] | ~opcode[4]);
        nextState.EX.MemtoReg[0] = ~opcode[4] & ~opcode[3];
        nextState.EX.RegWrite = ~opcode[5] | opcode[4] | opcode[3];
        nextState.EX.MemRead = ~opcode[5] & ~opcode[4];
        nextState.EX.MemWrite = ~opcode[6] & opcode[5] & ~opcode[4];
        nextState.EX.alu_op[1] = opcode[4];
        nextState.EX.alu_op[0] = opcode[6] & ~opcode[2];
        nextState.EX.branch_op[1] = opcode[6];
        nextState.EX.branch_op[0] = opcode[2];
    }

    // Immediate generation
    static bitset<32> immGen(bitset<32> &instruction)
    {
        uint8_t opcode = instruction.to_ulong() % (1 << 7);
        uint32_t val = instruction.to_ulong();
        int32_t imm;

        switch (opcode)
        {
        case 0b0110011: // R-type
        {
            // do nothing
        }
        break;
        case 0b0000011:
        case 0b0010011: // I-type 12-bit
        {
            uint32_t i11_0 = val >> 20;
            imm = i11_0;
            if (imm >= (1 << 11))
                imm -= 1 << 12;
        }
        break;
        case 0b0100011: // S-type 12-bit
        {
            uint32_t i11_5 = (val >> 25) << 5;
            uint32_t i4_0 = (val & 0xf80) >> 7;
            imm = i11_5 | i4_0;
            if (imm >= (1 << 11))
                imm -= 1 << 12;
        }
        break;
        case 0b1100011: // B-type 13-bit
        {
            uint32_t i12 = (val >> 31) << 12;
            uint32_t i11 = ((val & 0x80) >> 7) << 11;
            uint32_t i10_5 = ((val & 0x7e000000) >> 25) << 5;
            uint32_t i4_1 = ((val & 0xf00) >> 8) << 1;
            imm = i12 | i11 | i10_5 | i4_1;
            if (imm >= (1 << 12))
                imm -= 1 << 13;
        }
        break;
        case 0b1101111: // J-type 21-bit
        {
            uint32_t i20 = (val >> 31) << 20;
            uint32_t i19_12 = val & 0xff000;
            uint32_t i11 = ((val & 0x100000) >> 20) << 11;
            uint32_t i10_1 = ((val & 0x7fe00000) >> 21) << 1;
            imm = i20 | i19_12 | i11 | i10_1;
            if (imm >= (1 << 20))
                imm -= 1 << 21;
        }
        break;
        default:
            cerr << hex << "invalid opcode " << bitset<7>(opcode) << endl;
            assert(0);
            break;
        }
        return {static_cast<unsigned long long>(imm)};
    }

    static bitset<32> ALU(bitset<32> &r1, bitset<32> &r2, bitset<3> &alu_signal)
    {
        auto a = static_cast<int32_t>(r1.to_ulong());
        auto b = static_cast<int32_t>(r2.to_ulong());
        switch (alu_signal.to_ulong())
        {
        case 0b000:
            return {static_cast<uint32_t>(a + b)};
        case 0b001:
            return {static_cast<uint32_t>(a - b)};
        case 0b111:
            return {static_cast<uint32_t>(a & b)};
        case 0b110:
            return {static_cast<uint32_t>(a | b)};
        case 0b100:
            return {static_cast<uint32_t>(a ^ b)};
        default:
            cerr << "invalid ALU signal " << alu_signal << "!" << endl;
            assert(0);
        }
    }

    static bitset<1> BranchALU(bool zero, const bitset<2> &branch_signal)
    {
        switch (branch_signal.to_ulong())
        {
        case 0b00:
            return {static_cast<uint32_t>(0)};
        case 0b01:
            return {static_cast<uint32_t>(1 & zero)};
        case 0b10:
            return {static_cast<uint32_t>(1 ^ zero)};
        case 0b11:
            return {static_cast<uint32_t>(1)};
        default:
            cerr << "invalid Branch signal " << branch_signal << "!" << endl;
            assert(0);
        }
    }

    // pc + offset(4 or signed Imm)
    static bitset<32> Add(bitset<32> &pc_address, bitset<32> &offset)
    {
        return {static_cast<uint32_t>(pc_address.to_ulong() + offset.to_ulong())};
    }

    static bitset<32> MUX2(bitset<32> &r1, bitset<32> &r2, bitset<1> &index)
    {
        return index[0] == 0 ? r1 : r2;
    }

    static bitset<32> MUX3(bitset<32> &r1, bitset<32> &r2, bitset<32> &r3, bitset<2> &index)
    {
        switch (index.to_ulong())
        {
        case 0b00:
            return r1;
        case 0b01:
            return r2;
        case 0b10:
            return r3;
        default:
            cerr << "invalid index in MUX3" << endl;
            assert(0);
        }
    }

    // for debug, print the asm instruction
    void decode(bitset<7> &opcode, bitset<3> &funct3, bitset<7> &funct7)
    {
        cout << nextState.IF.PC.to_ulong() << ": ";
        uint8_t op_val = opcode.to_ulong();
        string op;

        if (op_val == 0b0110011 || op_val == 0b0010011) // R-type / I-type
        {
            uint8_t f3_val = funct3.to_ulong();
            switch (f3_val)
            {
            case 0b000:
            {
                uint8_t f7_val = funct7.to_ulong();
                if (op_val == 0b0110011 && f7_val == 0b0100000)
                {
                    op = "sub";
                }
                else
                {
                    if (op_val == 0b0110011)
                        assert(f7_val == 0);
                    op = "add";
                }
            }
            break;
            case 0b111:
                op = "and";
                break;
            case 0b110:
                op = "or";
                break;
            case 0b100:
                op = "xor";
                break;
            default:
                cerr << "invalid I-type/R-type instruction with funct3="
                     << funct3 << endl;
                assert(0);
            }
            if (op_val == 0b0010011)
            {
                op += 'i';
                cout << op << " "
                     << "x" << nextState.EX.Write_register.to_ulong()
                     << ", x" << nextState.EX.Rs.to_ulong() << ", " << (int)nextState.EX.Imm.to_ulong() << endl;
            }
            else
                cout << op << " "
                     << "x" << nextState.EX.Write_register.to_ulong()
                     << ", x" << nextState.EX.Rs.to_ulong() << ", x" << nextState.EX.Rt.to_ulong() << endl;
        }
        else if (op_val == 0b0000011) // lw
        {
            //             assert(funct3.to_ulong()==0b010);
            cout << "lw"
                 << " "
                 << "x" << nextState.EX.Write_register.to_ulong()
                 << ", " << (int)nextState.EX.Imm.to_ulong() << "(x" << nextState.EX.Rs.to_ulong() << ")" << endl;
        }
        else if (op_val == 0b0100011) // sw
        {
            assert(funct3.to_ulong() == 0b010);
            cout << "sw"
                 << " "
                 << "x" << nextState.EX.Rt.to_ulong()
                 << ", " << (int)nextState.EX.Imm.to_ulong() << "(x" << nextState.EX.Rs.to_ulong() << ")" << endl;
        }
        else if (op_val == 0b1100011) // B-type
        {
            uint8_t f3_val = funct3.to_ulong();
            switch (f3_val)
            {
            case 0b000:
                op = "beq";
                break;
            case 0b001:
                op = "bne";
                break;
            default:
                cerr << "invalid B-type instruction with funct3=" << funct3 << endl;
                assert(0);
            }
            cout << op << " x" << nextState.EX.Rs.to_ulong() << ", x" << nextState.EX.Rt.to_ulong() << ", "
                 << (int)(nextState.EX.Imm.to_ulong() + nextState.ID.PC.to_ulong()) << endl;
        }
        else if (op_val == 0b1101111) // J-type
        {
            cout << "jal x" << nextState.EX.Write_register.to_ulong() << ", "
                 << (int)(nextState.EX.Imm.to_ulong() + nextState.ID.PC.to_ulong()) << endl;
        }
        else
        {
            cerr << "invalid instruction with opcode=" << opcode << endl;
            assert(0);
        }
    }
};

class SingleStageCore : public Core
{
public:
    SingleStageCore(string ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir + "/SS_", imem, dmem), opFilePath(ioDir + "/StateResult_SS.txt")
    {
        state.IF.PC = bitset<32>(0);
    }

    void step()
    {
        /* Your implementation*/
        bitset<3> funct3;  // funct3 field
        bitset<32> delta;  // PC offset 4
        bitset<32> nextPC; // PC + 4

        nextState = state;

        /* IF */
        if (!nextState.IF.nop)
        {
            nextState.ID.Instr = ext_imem.readInstr(nextState.IF.PC);
            nextState.ID.PC = nextState.IF.PC;
            delta = bitset<32>(4);
            nextPC = Add(nextState.IF.PC, delta);

            // read the halt
            if ((nextState.ID.Instr.to_ulong() % (1 << 7)) == 0b1111111)
                nextState.IF.nop = true;
        }
        nextState.ID.nop = nextState.IF.nop;

        /* ID */
        if (!nextState.ID.nop)
        {
            uint32_t val = nextState.ID.Instr.to_ulong();

            // read register file
            nextState.EX.Rs = bitset<5>((val & 0xf8000) >> 15);          // instruction[19-15]
            nextState.EX.Rt = bitset<5>((val & 0x1f00000) >> 20);        // instruction[24-20]
            nextState.EX.Write_register = bitset<5>((val & 0xf80) >> 7); // instruction[11-7]
            nextState.EX.Read_data1 = myRF.readRF(nextState.EX.Rs);
            nextState.EX.Read_data2 = myRF.readRF(nextState.EX.Rt);

            // get the instruction[30, 14-12]
            uint32_t i30_14_12 = ((nextState.ID.Instr[30] & nextState.ID.Instr[5]) << 3) | (nextState.ID.Instr[14] << 2) | (nextState.ID.Instr[13] << 1) | nextState.ID.Instr[12];
            nextState.EX.inst = bitset<4>(i30_14_12);

            // imm gen: sign extend and maybe shift left 1
            nextState.EX.Imm = immGen(nextState.ID.Instr);

            // control
            bitset<7> opcode = bitset<7>(nextState.ID.Instr.to_ulong() % (1 << 7));
            control(opcode);

            funct3 = bitset<3>((val & 0x7000) >> 12);
#ifdef DEBUG
            bitset<7> funct7((val & 0xfe000000) >> 25);
            decode(opcode, funct3, funct7);
#endif
            nextState.EX.PC = nextState.ID.PC;
        }
        nextState.EX.nop = nextState.ID.nop;

        /* EX */
        if (!nextState.EX.nop)
        {
            // get data1
            auto data1 = MUX2(nextState.EX.Read_data1, nextState.EX.PC, nextState.EX.alu_src1);
            // get data2
            auto data2 = MUX3(nextState.EX.Read_data2,
                              nextState.EX.Imm, delta, nextState.EX.alu_src2);
            // get alu_signal
            auto alu_signal = ALUControl(nextState.EX.alu_op, nextState.EX.inst);

            nextState.MEM.ALU_result = ALU(data1, data2, alu_signal);
            // check the branch
            bool zero = nextState.MEM.ALU_result.to_ulong() == 0;
            auto branch_address = Add(nextState.ID.PC, nextState.EX.Imm);
            auto branch_signal = BranchControl(nextState.EX.branch_op, funct3);
            auto PCSrc = BranchALU(zero, branch_signal);
            nextState.IF.PC = MUX2(nextPC, branch_address, PCSrc);

            nextState.MEM.Write_register = nextState.EX.Write_register;
            nextState.MEM.Store_data = nextState.EX.Read_data2;

            nextState.MEM.MemRead = nextState.EX.MemRead;
            nextState.MEM.MemWrite = nextState.EX.MemWrite;

            nextState.MEM.MemtoReg = nextState.EX.MemtoReg;
            nextState.MEM.RegWrite = nextState.EX.RegWrite;
        }
        nextState.MEM.nop = nextState.EX.nop;

        /* MEM */
        if (!nextState.MEM.nop)
        {
            // read memory
            if (nextState.MEM.MemRead)
                nextState.WB.WriteData = ext_dmem.readDataMem(nextState.MEM.ALU_result);
            // write memory
            if (nextState.MEM.MemWrite)
                ext_dmem.writeDataMem(nextState.MEM.ALU_result, nextState.MEM.Store_data);

            nextState.WB.ALU_result = nextState.MEM.ALU_result;
            nextState.WB.Write_register = nextState.MEM.Write_register;
            nextState.WB.RegWrite = nextState.MEM.RegWrite;
            nextState.WB.MemtoReg = nextState.MEM.MemtoReg;
        }
        nextState.WB.nop = nextState.MEM.nop;

        /* WB */
        if (!nextState.WB.nop)
        {
            if (nextState.WB.RegWrite)
            {
                auto data = MUX2(nextState.WB.ALU_result,
                                 nextState.WB.WriteData, nextState.WB.MemtoReg);
                myRF.writeRF(nextState.WB.Write_register, data);
            }
        }

        if (state.IF.nop)
            halted = true;

        myRF.outputRF(cycle);         // dump RF
        printState(nextState, cycle); // print states after executing cycle 0, cycle 1, cycle 2 ...

        state = nextState; // The end of the cycle and updates the current state with the values calculated in this cycle
        cycle++;
    }

    void printState(stateStruct state, int cycle)
    {
        ofstream printstate;
        if (cycle == 0)
            printstate.open(opFilePath, std::ios_base::trunc);
        else
            printstate.open(opFilePath, std::ios_base::app);
        if (printstate.is_open())
        {
            printstate << "State after executing cycle:\t" << cycle << endl;

            printstate << "IF.PC:\t" << state.IF.PC.to_ulong() << endl;
            printstate << "IF.nop:\t" << state.IF.nop << endl;
        }
        else
            cout << "Unable to open SS StateResult output file." << endl;
        printstate.close();
    }

private:
    string opFilePath;
};

class FiveStageCore : public Core
{
public:
    FiveStageCore(string ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir + "/FS_", imem, dmem), opFilePath(ioDir + "/StateResult_FS.txt") {}

    void step()
    {
        /* Your implementation */
        nextState = state;
        bitset<32> branch_address;
        bitset<1> PCSrc(0b0);
        static auto delta = bitset<32>(4);
        bool nop = false;

        /* --------------------- WB stage --------------------- */
        if (!state.WB.nop)
        {
            if (state.WB.RegWrite)
            {
                auto data = MUX2(state.WB.ALU_result, state.WB.WriteData, state.WB.MemtoReg);
                myRF.writeRF(state.WB.Write_register, data);
            }
        }

        /* --------------------- MEM stage -------------------- */
        if (!state.MEM.nop)
        {
            // read memory
            if (state.MEM.MemRead)
                nextState.WB.WriteData = ext_dmem.readDataMem(state.MEM.ALU_result);

            // write memory
            if (state.MEM.MemWrite)
                ext_dmem.writeDataMem(state.MEM.ALU_result, state.MEM.Store_data);

            nextState.WB.ALU_result = state.MEM.ALU_result;
            nextState.WB.Write_register = state.MEM.Write_register;
        }
        nextState.WB.RegWrite = state.MEM.RegWrite;
        nextState.WB.MemtoReg = state.MEM.MemtoReg;
        nextState.WB.nop = state.MEM.nop;

        /* --------------------- EX stage --------------------- */
        if (!state.EX.nop)
        {
            /* Data hazard */
            auto forward = Forwarding_unit(state.EX.Rs, state.EX.Rt);
            // get data1
            auto data1 = MUX3(state.EX.Read_data1, state.WB.WriteData, state.MEM.ALU_result, forward.first);
            data1 = MUX2(data1, state.EX.PC, state.EX.alu_src1);

            // get data2
            auto data = MUX2(state.WB.ALU_result, state.WB.WriteData, state.WB.MemtoReg);
            auto data2 = MUX3(state.EX.Read_data2, data, state.MEM.ALU_result, forward.second);
            nextState.MEM.Store_data = data2;
            data2 = MUX3(data2, state.EX.Imm, delta, state.EX.alu_src2);

            // get alu_signal
            auto alu_signal = ALUControl(state.EX.alu_op, state.EX.inst);
            nextState.MEM.ALU_result = ALU(data1, data2, alu_signal);

            nextState.MEM.Write_register = state.EX.Write_register;
        }
        nextState.MEM.MemRead = state.EX.MemRead;
        nextState.MEM.MemWrite = state.EX.MemWrite;

        nextState.MEM.MemtoReg = state.EX.MemtoReg;
        nextState.MEM.RegWrite = state.EX.RegWrite;
        nextState.MEM.nop = state.EX.nop;

        /* --------------------- ID stage --------------------- */
        if (!state.ID.nop)
        {
            uint32_t val = state.ID.Instr.to_ulong();

            // control
            bitset<7> opcode = bitset<7>(state.ID.Instr.to_ulong() % (1 << 7));

            // read register file
            nextState.EX.Rs = bitset<5>((val & 0xf8000) >> 15);   // instruction[19-15]
            nextState.EX.Rt = bitset<5>((val & 0x1f00000) >> 20); // instruction[24-20]

            /* Hazard detection */
            HazardDetectionUnit(opcode, nextState.EX.Rs, nextState.EX.Rt);

            // insert a bubble(nop)
            if (stall_cycles || opcode.to_ulong() == 0b0000000)
            {
                insert_nop();
                nop = true;
            }
            else
            {
                control(opcode);
                nextState.EX.Write_register = bitset<5>((val & 0xf80) >> 7);
                auto data1 = myRF.readRF(nextState.EX.Rs);
                auto data2 = myRF.readRF(nextState.EX.Rt);
                if (opcode.to_ulong() == 0b1100011)
                {
                    /* Data hazard */
                    auto forward = Forwarding_unit(nextState.EX.Rs, nextState.EX.Rt);
                    auto data = MUX2(state.WB.ALU_result, state.WB.WriteData, state.WB.MemtoReg);
                    data1 = MUX3(data1, data, state.MEM.ALU_result, forward.first);
                    data2 = MUX3(data2, data, state.MEM.ALU_result, forward.second);
                }
                nextState.EX.Read_data1 = data1;
                nextState.EX.Read_data2 = data2;

                // get the instruction[30, 14-12]
                uint32_t i30_14_12 = ((state.ID.Instr[30] & state.ID.Instr[5]) << 3) | (state.ID.Instr[14] << 2) | (state.ID.Instr[13] << 1) | state.ID.Instr[12];
                nextState.EX.inst = bitset<4>(i30_14_12);

                // imm gen: sign extend and maybe shift left 1
                nextState.EX.Imm = immGen(nextState.ID.Instr);

                // check the branch
                bool zero = nextState.EX.Read_data1 == nextState.EX.Read_data2;
                auto funct3 = bitset<3>((val & 0x7000) >> 12);
                auto branch_signal = BranchControl(nextState.EX.branch_op, funct3);
                branch_address = Add(state.ID.PC, nextState.EX.Imm);
                PCSrc = BranchALU(zero, branch_signal);

                // PC
                nextState.EX.PC = state.ID.PC;
            }
        }
        if (!nop)
            nextState.EX.nop = state.ID.nop;

        /* --------------------- IF stage --------------------- */
        if (!state.IF.nop)
        {
            if (stall_cycles)
                --stall_cycles;
            else
            {
                auto nextPC = Add(state.IF.PC, delta);
                nextState.IF.PC = MUX2(nextPC, branch_address, PCSrc);
                // if branch occurs, IF_Flush is true
                if (PCSrc[0])
                    nextState.ID.Instr.reset();
                else
                {
                    nextState.ID.Instr = ext_imem.readInstr(state.IF.PC);
                    nextState.ID.PC = state.IF.PC;
                    // read the halt
                    if ((nextState.ID.Instr.to_ulong() % (1 << 7)) == 0b1111111)
                        nextState.IF.nop = nextState.ID.nop = true;
                    else
                        nextState.ID.nop = state.IF.nop;
                }
            }
        }

        if (state.IF.nop && state.ID.nop && state.EX.nop && state.MEM.nop && state.WB.nop)
            halted = true;

        myRF.outputRF(cycle);         // dump RF
        printState(nextState, cycle); // print states after executing cycle 0, cycle 1, cycle 2 ...

        state = nextState; // The end of the cycle and updates the current state with the values calculated in this cycle
        cycle++;
    }

    void printState(stateStruct state, int cycle)
    {
        ofstream printstate;
        if (cycle == 0)
            printstate.open(opFilePath, std::ios_base::trunc);
        else
            printstate.open(opFilePath, std::ios_base::app);
        if (printstate.is_open())
        {
            printstate << "State after executing cycle:\t" << cycle << endl;

            printstate << "IF.PC:\t" << state.IF.PC.to_ulong() << endl;
            printstate << "IF.nop:\t" << state.IF.nop << endl;

            printstate << "ID.PC:\t" << state.ID.PC << endl;
            printstate << "ID.Instr:\t" << state.ID.Instr << endl;
            printstate << "ID.nop:\t" << state.ID.nop << endl;

            printstate << "EX.PC:\t" << state.EX.PC << endl;
            printstate << "EX.Read_data1:\t" << state.EX.Read_data1 << endl;
            printstate << "EX.Read_data2:\t" << state.EX.Read_data2 << endl;
            printstate << "EX.Rs:\t" << state.EX.Rs << endl;
            printstate << "EX.Rt:\t" << state.EX.Rt << endl;
            printstate << "EX.Imm:\t" << state.EX.Imm << endl;
            printstate << "EX.inst:\t" << state.EX.inst << endl;
            printstate << "EX.Write_register:\t" << state.EX.Write_register << endl;
            printstate << "EX.branch_op:\t" << state.EX.branch_op << endl;
            printstate << "EX.MemtoReg:\t" << state.EX.MemtoReg << endl;
            printstate << "EX.RegWrite:\t" << state.EX.RegWrite << endl;

            printstate << "EX.MemRead:\t" << state.EX.MemRead << endl;
            printstate << "EX.MemWrite:\t" << state.EX.MemWrite << endl;

            printstate << "EX.alu_op:\t" << state.EX.alu_op << endl;
            printstate << "EX.alu_src1:\t" << state.EX.alu_src1 << endl;
            printstate << "EX.alu_src2:\t" << state.EX.alu_src2 << endl;

            printstate << "EX.nop:\t" << state.EX.nop << endl;

            printstate << "MEM.ALU_result:\t" << state.MEM.ALU_result << endl;
            printstate << "MEM.Store_data:\t" << state.MEM.Store_data << endl;
            printstate << "MEM.Rs:\t" << state.MEM.Rs << endl;
            printstate << "MEM.Rt:\t" << state.MEM.Rt << endl;
            printstate << "MEM.Write_register:\t" << state.MEM.Write_register << endl;
            printstate << "MEM.MemRead:\t" << state.MEM.MemRead << endl;
            printstate << "MEM.MemWrite:\t" << state.MEM.MemWrite << endl;
            printstate << "MEM.MemtoReg:\t" << state.MEM.MemtoReg << endl;
            printstate << "MEM.RegWrite:\t" << state.MEM.RegWrite << endl;
            printstate << "MEM.nop:\t" << state.MEM.nop << endl;

            printstate << "WB.WriteData:\t" << state.WB.WriteData << endl;
            printstate << "WB.ALU_result:\t" << state.WB.ALU_result << endl;
            printstate << "WB.Rs:\t" << state.WB.Rs << endl;
            printstate << "WB.Rt:\t" << state.WB.Rt << endl;
            printstate << "WB.Write_register:\t" << state.WB.Write_register << endl;
            printstate << "WB.MemtoReg:\t" << state.WB.MemtoReg << endl;
            printstate << "WB.RegWrite:\t" << state.WB.RegWrite << endl;
            printstate << "WB.nop:\t" << state.WB.nop << endl;
        }
        else
            cout << "Unable to open FS StateResult output file." << endl;
        printstate.close();
    }

private:
    string opFilePath;
    // stall cycles setted by hazard detection unit
    uint8_t stall_cycles = 0;

    pair<bitset<2>, bitset<2>> Forwarding_unit(bitset<5> &Rs, bitset<5> &Rt)
    {
        bitset<2> ForwardA, ForwardB;

        bool pre_condition_1 = state.MEM.RegWrite && state.MEM.Write_register.to_ulong() != 0;
        bool pre_condition_2 = state.WB.RegWrite && state.WB.Write_register.to_ulong() != 0;

        // EX hazard
        bool a1 = pre_condition_1 && state.MEM.Write_register == Rs;
        bool b1 = pre_condition_1 && state.MEM.Write_register == Rt;
        bitset<2> ex_hazard(0b10);
        // MEM hazard
        bool a2 = pre_condition_2 && !a1 && state.WB.Write_register == Rs;
        bool b2 = pre_condition_2 && !b1 && state.WB.Write_register == Rt;
        bitset<2> mem_hazard(0b01);

        if (a1)
            ForwardA = ex_hazard;
        else if (a2)
            ForwardA = mem_hazard;
        if (b1)
            ForwardB = ex_hazard;
        else if (b2)
            ForwardB = mem_hazard;

        return {ForwardA, ForwardB};
    }

    void HazardDetectionUnit(bitset<7> opcode, bitset<5> &Rs, bitset<5> &Rt)
    {
        // conditional branch
        if (opcode.to_ulong() == 0b1100011)
        {
            // a lw or instruction needs to write Rs or Rt
            if (state.EX.RegWrite && (state.EX.Write_register == Rs || state.EX.Write_register == Rt))
                stall_cycles = state.EX.MemRead ? 2 : 1;
        }
        else if (state.EX.MemRead && (state.EX.Write_register == Rs || state.EX.Write_register == Rt))
            stall_cycles = 1;
    }

    // a nop instruction
    void insert_nop()
    {
        nextState.EX.MemWrite = false;
        nextState.EX.MemRead = false;
        nextState.EX.MemtoReg = false;
        nextState.EX.RegWrite = false;
        nextState.EX.nop = true;
    }
};

int main(int argc, char *argv[])
{
    string ioDir = "";
    if (argc == 1)
    {
        cout << "Enter path containing the memory files: ";
        cin >> ioDir;
    }
    else if (argc > 2)
    {
        cout << "Invalid number of arguments. Machine stopped." << endl;
        return -1;
    }
    else
    {
        ioDir = argv[1];
        cout << "IO Directory: " << ioDir << endl;
    }

    InsMem imem = InsMem("Imem", ioDir);
    DataMem dmem_ss = DataMem("SS", ioDir);
    DataMem dmem_fs = DataMem("FS", ioDir);

    SingleStageCore SSCore(ioDir, imem, dmem_ss);
    FiveStageCore FSCore(ioDir, imem, dmem_fs);

    while (1)
    {
        if (!SSCore.halted)
            SSCore.step();

        if (!FSCore.halted)
            FSCore.step();

        if (SSCore.halted && FSCore.halted)
            break;
    }

    // dump SS and FS data mem.
    dmem_ss.outputDataMem();
    dmem_fs.outputDataMem();

    // print
    ofstream printstate;
    string performance_metrics = ioDir + "/PerformanceMetrics_Result.txt";
    printstate.open(performance_metrics, std::ios_base::trunc);
    if (printstate.is_open())
    {
        printstate << "IO Directory: " << performance_metrics << endl;
        printstate << "Single Stage Core Performance Metrics-----------------------------" << endl;
        printstate << "Number of cycles taken: " << SSCore.cycle << endl;
        printstate << "Cycles per instruction: " << 1.0 * SSCore.cycle / SSCore.get_instruction_number() << endl;
        printstate << "Instructions per cycle: " << 1.0 * SSCore.get_instruction_number() / SSCore.cycle << endl;
        printstate << endl;
        printstate << "Five Stage Core Performance Metrics-----------------------------" << endl;
        printstate << "Number of cycles taken: " << FSCore.cycle << endl;
        printstate << "Cycles per instruction: " << 1.0 * FSCore.cycle / FSCore.get_instruction_number() << endl;
        printstate << "Instructions per cycle: " << 1.0 * FSCore.get_instruction_number() / FSCore.cycle << endl;
        printstate << endl;
    }
    else
        cerr << "open file failed" << endl;
    printstate.close();

    return 0;
}