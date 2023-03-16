//=- RISCVRegShuffler.cpp - Shuffle registers post-RA                    ----=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// TODO
//
//===----------------------------------------------------------------------===//

#include "RISCV.h"
#include "RISCVInstrInfo.h"
#include "RISCVRegCompressionPriorities.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveRegMatrix.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"

#include <queue>

using namespace llvm;

#define DEBUG_TYPE "riscv-reg-shuffler"

namespace {
class RISCVRegShuffler : public MachineFunctionPass {
  const MachineRegisterInfo *MRI;
  const TargetRegisterInfo *TRI;
  const TargetInstrInfo *TII;
  ArrayRef<uint8_t> RegCosts;
  LiveRegMatrix *Matrix;

public:
  static char ID;
  RISCVRegShuffler() : MachineFunctionPass(ID) {
    initializeRISCVRegShufflerPass(*PassRegistry::getPassRegistry());
  }

  bool isCompressibleReg(MCRegister PhysReg);
  void getAnalysisUsage(AnalysisUsage &AU) const override;
  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return "RISCV Register Shuffler"; }

private:
  bool optimizeBlock(MachineBasicBlock &MBB);
};

} // end anonymous namespace

char RISCVRegShuffler::ID = 0;

INITIALIZE_PASS_BEGIN(RISCVRegShuffler, "riscv-reg-shuffler",
                      "RISCV register shuffling pass", false, false)
INITIALIZE_PASS_DEPENDENCY(RISCVRegCompressionPriorities)
INITIALIZE_PASS_DEPENDENCY(VirtRegMap)
INITIALIZE_PASS_DEPENDENCY(LiveRegMatrix)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_END(RISCVRegShuffler, "riscv-reg-shuffler",
                    "RISCV register shuffling pass", false, false)

void RISCVRegShuffler::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<RISCVRegCompressionPriorities>();
  AU.addRequired<VirtRegMap>();
  AU.addRequired<LiveRegMatrix>();
  AU.addRequired<LiveIntervals>();
  AU.setPreservesAll();
  MachineFunctionPass::getAnalysisUsage(AU);
}

SmallVector<MCRegister, 8> getCompressibleRegs() {
  return {RISCV::X10, RISCV::X11, RISCV::X12, RISCV::X13,
          RISCV::X14, RISCV::X15, RISCV::X8,  RISCV::X9};
}

bool RISCVRegShuffler::isCompressibleReg(MCRegister PhysReg) {
  // FIXME: We don't want to use X8, X9 unconditionally, because
  // these two are callee save and might introduce extra save/restore.
  //  return PhysReg == RISCV::X8 || PhysReg == RISCV::X9 ||
  //         PhysReg == RISCV::X10 || PhysReg == RISCV::X11 ||
  //         PhysReg == RISCV::X12 || PhysReg == RISCV::X13 ||
  //         PhysReg == RISCV::X14 || PhysReg == RISCV::X15;
  return PhysReg == RISCV::X10 || PhysReg == RISCV::X11 ||
         PhysReg == RISCV::X12 || PhysReg == RISCV::X13 ||
         PhysReg == RISCV::X14 || PhysReg == RISCV::X15 ||
         PhysReg == RISCV::X8 || PhysReg == RISCV::X9;

  ;
  // return RegCosts[PhysReg] == 0;
}

bool isCalleeSave(MCRegister PhysReg) {
  return PhysReg == RISCV::X8 || PhysReg == RISCV::X9;
}

bool RISCVRegShuffler::runOnMachineFunction(MachineFunction &MF) {
  if (skipFunction(MF.getFunction()))
    return false;

  TRI = MF.getSubtarget().getRegisterInfo();
  MRI = &MF.getRegInfo();
  RegCosts = TRI->getRegisterCosts(MF);

  auto const &PriorityMap =
      getAnalysis<RISCVRegCompressionPriorities>().getPriorityMap();
  Matrix = &getAnalysis<LiveRegMatrix>();
  auto &RegMap = getAnalysis<VirtRegMap>();
  auto const &LIS = getAnalysis<LiveIntervals>();

  unsigned NumRegs = MF.getRegInfo().getNumVirtRegs();
  assert(NumRegs == PriorityMap.size());

  // FIXME: Be careful of RegClass!
  // Put all virtual registers into a priority queue, according to
  // their compression priority

  // `Queue` stores (priority, register_id) pair.
  std::priority_queue<std::pair<unsigned, unsigned>> Queue;

  for (unsigned RegIdx = 0; RegIdx < NumRegs; RegIdx++) {
    auto Reg = Register::index2VirtReg(RegIdx);

    // Make sure that `Reg` has corresponding physreg, rather
    // than stack slots.
    if (!RegMap.hasPhys(Reg))
      continue;

    Queue.push(std::make_pair(PriorityMap[Reg], RegIdx));
  }

  // Dump `Queue`
  //  while (!Queue.empty()) {
  //    auto [Priority, RegIdx] = Queue.top();
  //    Queue.pop();
  //    auto Reg = Register::index2VirtReg(RegIdx);
  //    errs() << "hello: " << printReg(Reg) << ", priority = " << Priority <<
  //    "\n";
  //  }

  const unsigned CalleeRegUseCost = 2;

  // Iterate over `Queue` to perform register shuffling
  // FIXME: Skip VRegs that have Priority 0
  while (!Queue.empty()) {
    auto [Priority, RegIdx] = Queue.top();
    Queue.pop();
    auto Reg = Register::index2VirtReg(RegIdx);
    auto &RegLI = LIS.getInterval(Reg);

    LLVM_DEBUG(dbgs() << "try to find better register for " << printReg(Reg)
                      << ", priority = " << Priority << "\n");

    auto PhysReg = RegMap.getPhys(Reg);
    assert(PhysReg != MCRegister::NoRegister);

    // If this VReg is already allocated to a popular physical
    // register, we're happy.
    if (isCompressibleReg(PhysReg)) {
      LLVM_DEBUG(dbgs() << "  " << printReg(Reg) << " already has popular reg "
                        << printReg(PhysReg, TRI) << "\n");
      continue;
    }

    // If not, see if there are free popular registers that we
    // can use directly.
    // TODO: WHAT, IT SEEMS LIKE LLVM DOESN'T RECYCLE REGISTERS?
    const auto *RegClass = MRI->getRegClass(Reg);
    bool Swapped = false;
    for (TargetRegisterClass::iterator RegClassPhys = RegClass->begin();
         RegClassPhys < RegClass->end(); RegClassPhys++) {
      if (PhysReg == *RegClassPhys)
        continue;

      if (!isCompressibleReg(*RegClassPhys))
        continue;

      // When using callee save registers such as x8 and x9,
      // in the worst case we will have to introduce two extra
      // save and restore, costing us another 4 bytes.
      // So we're aiming for the worst case, and make sure the
      // priority, which is approximately (bytes_save / 2),
      // is greater than 2.
      if (isCalleeSave(*RegClassPhys) && Priority <= CalleeRegUseCost) {
        continue;
      }

      LLVM_DEBUG(dbgs() << "  Found a popular register "
                        << printReg(*RegClassPhys, TRI)
                        << ", checking interference\n");

      if (Matrix->checkInterference(RegLI, *RegClassPhys) ==
          LiveRegMatrix::IK_Free) {
        Matrix->unassign(RegLI);
        Matrix->assign(RegLI, *RegClassPhys);
        LLVM_DEBUG(dbgs() << "  interference check passed, swapped ok.\n";
                   Swapped = true);
      }
    }

    if (Swapped)
      continue;

    LLVM_DEBUG(
        dbgs()
        << "  No free popular regs found, finding an suitable interfering "
           "vreg to swap\n");

    // If no free popular registers, find an interfering VReg that
    //   1) occupies a popular register,
    //   2) has the lowest compression priority,
    //   3) compression priority is lower than current VReg
    // and try to swap their physical registers.
    SmallVector<const LiveInterval *, 4> BestInterferingVRegs;
    unsigned MinPriority = std::numeric_limits<unsigned>::max();

    for (TargetRegisterClass::iterator RegClassPhys = RegClass->begin();
         RegClassPhys < RegClass->end(); RegClassPhys++) {

      if (!isCompressibleReg(*RegClassPhys))
        continue;

      // TODO: Make use of x8 and x9
      if (isCalleeSave(*RegClassPhys)) {
        continue;
      }

      const auto &PopularPhysReg = *RegClassPhys;

      SmallVector<const LiveInterval *, 4> InterferingVRegs;
      unsigned TotalPriority = 0;

      for (MCRegUnitIterator Units(PopularPhysReg, TRI); Units.isValid();
           ++Units) {
        LiveIntervalUnion::Query &Q = Matrix->query(RegLI, *Units);
        ArrayRef<const LiveInterval *> IVR = Q.interferingVRegs();
        for (const auto *I : IVR) {
          auto IReg = I->reg();
          auto IPriority = PriorityMap[IReg];

          TotalPriority += IPriority;
          if ((!isCalleeSave(PopularPhysReg) && TotalPriority >= Priority) ||
              (isCalleeSave(PopularPhysReg) &&
               TotalPriority + CalleeRegUseCost >= Priority)) {
            // Skip this PhysReg
            goto SkipThisPhys;
          }

          InterferingVRegs.push_back(I);
        }
      }

      if (TotalPriority < MinPriority) {
        MinPriority = TotalPriority;
        BestInterferingVRegs = InterferingVRegs;
      }

    SkipThisPhys:;
    }

    if (BestInterferingVRegs.size() == 0)
      continue;

    LLVM_DEBUG(dbgs() << "  BestInterferingVRegs = ");
    for (const auto *ILI : BestInterferingVRegs) {
      LLVM_DEBUG(dbgs() << printReg(ILI->reg()) << " ");
    }
    LLVM_DEBUG(dbgs() << "\n");

    // FIXME: Deal with more than one interfering vregs
    auto IPhysReg = RegMap.getPhys(BestInterferingVRegs[0]->reg());
    LLVM_DEBUG(dbgs() << "  IPhysReg = " << printReg(IPhysReg, TRI) << "\n");

    // Now we try to swap, if and only if the end result doesn't violate
    // interference rules.

    // Unassign all vregs involved
    LLVM_DEBUG(dbgs() << "  unassigning RegLI " << printReg(RegLI.reg())
                      << "\n");
    Matrix->unassign(RegLI);
    for (const auto *ILI : BestInterferingVRegs) {
      LLVM_DEBUG(dbgs() << "  unassigning " << printReg(ILI->reg()) << "\n");
      Matrix->unassign(*ILI);
    }
    LLVM_DEBUG(dbgs() << "  unasisign done\n");

    // Check if interference is violated
    bool NoViolation = true;
    NoViolation &=
        Matrix->checkInterference(RegLI, IPhysReg) == LiveRegMatrix::IK_Free;
    if (NoViolation) {
      for (const auto *ILI : BestInterferingVRegs) {
        if (Matrix->checkInterference(*ILI, PhysReg) !=
            LiveRegMatrix::IK_Free) {
          NoViolation = false;
          break;
        }
      }
    }

    if (!NoViolation) {
      LLVM_DEBUG(dbgs() << "  Swapping violates interference, bailing out\n");

      // Reassign all vregs involved
      Matrix->assign(RegLI, PhysReg);
      for (const auto *ILI : BestInterferingVRegs) {
        Matrix->assign(*ILI, IPhysReg);
      }

      continue;
    }

    // TODO: Continue swapping.
    LLVM_DEBUG(dbgs() << "  Swapping, before = { " << printReg(Reg) << " -> "
                      << printReg(PhysReg, TRI) << " ");
    for (const auto *ILI : BestInterferingVRegs) {
      LLVM_DEBUG(dbgs() << printReg(ILI->reg()) << " -> "
                        << printReg(IPhysReg, TRI) << " ");
    }

    Matrix->assign(RegLI, IPhysReg);
    for (const auto *ILI : BestInterferingVRegs) {
      Matrix->assign(*ILI, PhysReg);
    }
  }

  return false;
}

FunctionPass *llvm::createRISCVRegShufflerPass() {
  return new RISCVRegShuffler();
}
