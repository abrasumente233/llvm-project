#include "RISCV.h"
#include "RISCVSubtarget.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Pass.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-tie-reg"

namespace {
class RISCVTieReg : public MachineFunctionPass {
public:
  static char ID;

  RISCVTieReg() : MachineFunctionPass(ID) {
    initializeRISCVTieRegPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override {
    if (skipFunction(MF.getFunction()))
      return false;

    const auto &STI = MF.getSubtarget<RISCVSubtarget>();
    const auto *TII = STI.getInstrInfo();

    for (auto &MBB : MF) {
      for (auto &MI : MBB) {
        // If MI is an XOR
        if (MI.getOpcode() == RISCV::XOR) {
          // Insert a copy instruction before XOR
          MachineInstrBuilder MIB =
              BuildMI(MBB, MI, MI.getDebugLoc(), TII->get(RISCV::COPY),
                      MI.getOperand(0).getReg());
          MIB.addReg(MI.getOperand(1).getReg());

          // TODO: Update LiveVariables

          // Change the second operand of XOR to the first operand,
          // updating use info
          assert(!MI.defs().empty());

          errs() << *MIB.getInstr();
          errs() << MI;
        }
      }
    }
    return false;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    // AU.addRequiredID(LiveVariablesID);
    // AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};
} // end anonymous namespace

char RISCVTieReg::ID = 0;

INITIALIZE_PASS_BEGIN(RISCVTieReg, "riscv-tie-reg", "RISCV Tie Reg Pass", false,
                      false)
//INITIALIZE_PASS_DEPENDENCY(LiveVariables)
INITIALIZE_PASS_END(RISCVTieReg, "riscv-tie-reg", "RISCV Tie Reg Pass", false,
                    false)

FunctionPass *llvm::createRISCVTieRegPass() { return new RISCVTieReg(); }