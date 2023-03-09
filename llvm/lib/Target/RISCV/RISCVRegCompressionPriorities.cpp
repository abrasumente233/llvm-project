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

#include "RISCVRegCompressionPriorities.h"
#include "RISCV.h"
#include "RISCVSubtarget.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Pass.h"

using namespace llvm;

#define DEBUG_TYPE "riscv-reg-compression-priorities"

bool llvm::RISCVRegCompressionPriorities::runOnMachineFunction(
    MachineFunction &MF) {

  if (skipFunction(MF.getFunction()))
    return false;

  const auto &STI = MF.getSubtarget<RISCVSubtarget>();

  Virt2PriorityMap.clear();
  unsigned NumRegs = MF.getRegInfo().getNumVirtRegs();
  Virt2PriorityMap.resize(NumRegs);

  for (auto &MBB : MF) {
    for (auto &MI : MBB) {

      auto CompressibleRegs = RISCVRVC::getCompressibleRegs(MI, STI);

      if (CompressibleRegs.empty())
        continue;

      LLVM_DEBUG(MI.print(dbgs()));
      LLVM_DEBUG(dbgs() << "this instruction can be compressed, and it has the "
                           "following compressible registers: ");
      for (const auto &Reg : CompressibleRegs) {
        LLVM_DEBUG(dbgs() << printReg(Reg) << ", ");
      }
      LLVM_DEBUG(dbgs() << "\n");

      for (const auto &Reg : CompressibleRegs) {
        assert(Reg.isVirtual() &&
               "I thought all registers you returned are virtual?");
        Virt2PriorityMap[Reg]++;
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

char RISCVRegCompressionPriorities::ID = 0;

INITIALIZE_PASS(RISCVRegCompressionPriorities,
                "riscv-reg-compression-priorities",
                "Analyze register compression priorities", false, true)

FunctionPass *llvm::createRISCVRegCompressionPrioritiesPass() {
  return new RISCVRegCompressionPriorities();
}