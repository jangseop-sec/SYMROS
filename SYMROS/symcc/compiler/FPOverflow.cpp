#include "FPOverflow.h"

using namespace llvm;

void FPOverflowChecker::visitBinaryOperator(BinaryOperator &I) {
  // result check
  if (!I.getType()->isFloatingPointTy())
    return;

  // operand check
  if (!I.getOperand(0)->getType()->isFloatingPointTy() ||
      !I.getOperand(1)->getType()->isFloatingPointTy())
    return;

  // opcode filter
  if (I.getOpcode() != Instruction::FAdd &&
      I.getOpcode() != Instruction::FSub &&
      I.getOpcode() != Instruction::FMul &&
      I.getOpcode() != Instruction::FDiv &&
      I.getOpcode() != Instruction::FRem)
    return;

  // nnan, ninf -> compiler assumption 
  if (I.hasNoNaNs() && I.hasNoInfs())
    return;

  // TODO implement!!!
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();
  BasicBlock *OriginBB = I.getParent();

  Instruction *NextInst = I.getNextNode();
  if (!NextInst)
    return;

  BasicBlock *ContBB = OriginBB->splitBasicBlock(NextInst, "origin_cont");
  IRBuilder<> IRB(OriginBB);

  OriginBB->getTerminator()->eraseFromParent();

  if (mode == 0) {
    // new basic blocks for numeric execption check
    BasicBlock *OverflowCheckBB = BasicBlock::Create(Ctx, "overflow_check", F);
    BasicBlock *DividedByZeroCheckBB =
        BasicBlock::Create(Ctx, "devided_by_zero_check", F);
    // connect origin branch to overflow check branch
    IRB.CreateBr(OverflowCheckBB);

    // overflow check branch
    IRBuilder<> IR1(OverflowCheckBB);

    Value *OverflowCond = getOverflowCond(I, IR1);

    BranchInst *OverflowCheckBranch =
        IR1.CreateCondBr(OverflowCond, DividedByZeroCheckBB, DividedByZeroCheckBB);

    // set metadata
    OverflowCheckBranch->setMetadata(
        "symros.check", MDNode::get(Ctx, MDString::get(Ctx, "fp_overflow")));
    OverflowCheckBranch->setDebugLoc(I.getDebugLoc());

    // divided by zero check branch
    IRBuilder<> IR3(DividedByZeroCheckBB);
    bool is_div = true;

    if (I.getOpcode() != Instruction::FDiv) {
      is_div = false;
    }

    Value *DividedByZeroCond = getDividedByZeroCondition(I, IR3, is_div);

    BranchInst *DividedByZeroCheckBranch =
        IR3.CreateCondBr(DividedByZeroCond, ContBB, ContBB);

    // set metadata
    DividedByZeroCheckBranch->setMetadata(
        "symros.check",
        MDNode::get(Ctx, MDString::get(Ctx, "fp_divided_by_zero")));
    DividedByZeroCheckBranch->setDebugLoc(I.getDebugLoc());

  } else if (mode == 1) {
    BasicBlock *MidBB = BasicBlock::Create(I.getContext(), "guide.mid", I.getFunction());
    addBoundCheckChain(I, IRB, OriginBB, MidBB);
    addEpsilonBoundCheckChain(I, IRB, MidBB, ContBB);
  }
}

Value *FPOverflowChecker::getOverflowCond(llvm::BinaryOperator &I,
                                          llvm::IRBuilder<> &IRB) {

  Value *Result = &I;

  Value *PosInf = ConstantFP::getInfinity(Result->getType());
  Value *NegInf = ConstantFP::getInfinity(Result->getType(), true);

  Value *IsPosInf = IRB.CreateFCmpOEQ(Result, PosInf);
  Value *IsNegInf = IRB.CreateFCmpOEQ(Result, NegInf);

  return IRB.CreateOr(IsPosInf, IsNegInf);
}

Value *FPOverflowChecker::getDividedByZeroCondition(llvm::BinaryOperator &I,
                                                    llvm::IRBuilder<> &IRB, bool is_div) {
  if (!is_div) {
    return ConstantInt::getFalse(I.getContext());
  }

  Value *Divisor = I.getOperand(1);

  Value *Zero = ConstantFP::get(Divisor->getType(), 0.0);

  return IRB.CreateFCmpOEQ(Divisor, Zero);
}

void
FPOverflowChecker::addBoundCheckChain(llvm::BinaryOperator &I, llvm::IRBuilder<> &IRB, BasicBlock *EntryBB, BasicBlock *ExitBB) {
  Value *Result = &I;
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();

  double boundary_values[] = {2147483646.0, 65536.0, 256.0, 16.0, 8.0, 4.0, 2.0};

  BasicBlock *CurBB = EntryBB;
  bool isFirst = true;

  for (double BV : boundary_values) {

    IRB.SetInsertPoint(CurBB);

    Value *PlusBoundary = ConstantFP::get(Result->getType(), BV);
    Value *PlusCmp = IRB.CreateFCmpOGT(Result, PlusBoundary);

    Value *MinusBoundary = ConstantFP::get(Result->getType(), -BV);
    Value *MinusCmp = IRB.CreateFCmpOLT(Result, MinusBoundary);

    Value *Cmp = IRB.CreateOr(PlusCmp, MinusCmp);

    BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

    BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);
    if (isFirst) {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "float_bound_check_start")));
      isFirst = false;
    } else {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "float_bound_check")));
    }
    Br->setDebugLoc(I.getDebugLoc());

    CurBB = NextBB;
  }

  IRB.SetInsertPoint(CurBB);
  IRB.CreateBr(ExitBB);
}

void
FPOverflowChecker::addEpsilonBoundCheckChain(llvm::BinaryOperator &I, llvm::IRBuilder<> &IRB, BasicBlock *EntryBB, BasicBlock *ExitBB) {
  Value *Result = &I;
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();

  double boundary_values[] = {1e-5, 1e-10, 1e-20};

  BasicBlock *CurBB = EntryBB;
  bool isFirst = true;

  for (double BV : boundary_values) {

    IRB.SetInsertPoint(CurBB);

    Value *PlusBoundary = ConstantFP::get(Result->getType(), BV);
    Value *PlusCmp = IRB.CreateFCmpOLT(Result, PlusBoundary);

    Value *MinusBoundary = ConstantFP::get(Result->getType(), -BV);
    Value *MinusCmp = IRB.CreateFCmpOGT(Result, MinusBoundary);

    Value *Cmp = IRB.CreateAnd(PlusCmp, MinusCmp);

    BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

    BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);
    if (isFirst) {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "float_epsilon_bound_check_start")));
      isFirst = false;
    } else {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "float_epsilon_bound_check")));
    }
    Br->setDebugLoc(I.getDebugLoc());

    CurBB = NextBB;
  }

  IRB.SetInsertPoint(CurBB);
  IRB.CreateBr(ExitBB);
}

void FPOverflowChecker::setMode(int mode) {
  this->mode = mode;
}