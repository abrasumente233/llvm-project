//=- RISCVRegCompressionPriorities.cpp -- Compression Priorities Analysis ---=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass analyses register compression priorities for each function.
//
//===----------------------------------------------------------------------===//

#include "RISCV.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Pass.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-reg-compression-priorities"

namespace {
class RISCVRegCompressionPriorities : public MachineFunctionPass {
private:
  // Virt2PriorityMap - This is a virtual to compression priority
  // mapping, a.k.a the number of times a virtual register appears in
  // compressible instructions. Each virtual register is required to have
  // an entry in it, even if it doesn't appear in compressible instructions,
  // in which case its priority is zero.
  IndexedMap<unsigned, VirtReg2IndexFunctor> Virt2PriorityMap;

public:
  static char ID;

  MachineFunction *MF = nullptr;

  RISCVRegCompressionPriorities() : MachineFunctionPass(ID) {
    initializeRISCVRegCompressionPrioritiesPass(
        *PassRegistry::getPassRegistry());
  }

protected:
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

public:
  bool runOnMachineFunction(MachineFunction &MF) override {
    if (skipFunction(MF.getFunction()))
      return false;

    Virt2PriorityMap.clear();
    unsigned NumRegs = MF.getRegInfo().getNumVirtRegs();
    Virt2PriorityMap.resize(NumRegs);

    for (auto &MBB : MF) {
      for (auto &MI : MBB) {
        for (unsigned OpIdx = 0, E = MI.getNumOperands(); OpIdx < E; ++OpIdx) {
          auto Op = MI.getOperand(OpIdx);
          if (!Op.isReg())
            continue;
          auto RegOp = Op.getReg();
          if (RegOp.isVirtual()) {
            // FIXME: Now we only count the number of times a virtual register
            // appears in ALL instructions.
            errs() << "This reg is virtual: " << printReg(RegOp) << "\n";
            Virt2PriorityMap[RegOp]++;
          } else {
            errs() << "This reg is not virtual: " << printReg(RegOp) << "\n";
          }
        }
      }
    }

    // Print the priorities
    for (unsigned Idx = 0, E = Virt2PriorityMap.size(); Idx < E; Idx++) {
      auto Virt = Register::index2VirtReg(Idx); // Probably non-existent?
      auto Priority = Virt2PriorityMap[Virt];

      errs() << printReg(Virt) << ": " << Priority << "\n";
    }

    // MF.print(errs());

    return false;
  }
};
} // end anonymous namespace

char RISCVRegCompressionPriorities::ID = 0;

INITIALIZE_PASS(RISCVRegCompressionPriorities,
                "riscv-reg-compression-priorities",
                "Analyze register compression priorities", false, true)

FunctionPass *llvm::createRISCVRegCompressionPrioritiesPass() {
  return new RISCVRegCompressionPriorities();
}