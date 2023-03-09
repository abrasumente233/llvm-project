//===-- RISCVFrameLowering.h - Define frame lowering for RISCV -*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class implements RISCV-specific bits of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RISCV_REG_COMPRESSION_PRIORITIES_H
#define LLVM_LIB_TARGET_RISCV_REG_COMPRESSION_PRIORITIES_H

#include "RISCV.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Pass.h"
#include "llvm/Support/TypeSize.h"

namespace llvm {
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

  const IndexedMap<unsigned, VirtReg2IndexFunctor> &getPriorityMap() const {
    return Virt2PriorityMap;
  }

  RISCVRegCompressionPriorities() : MachineFunctionPass(ID) {
    initializeRISCVRegCompressionPrioritiesPass(
        *PassRegistry::getPassRegistry());
  }

protected:
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  };

public:
  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace llvm
#endif
