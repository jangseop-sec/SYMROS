#include "Overflow.h"

using namespace llvm;

void OverflowChecker::visitBinaryOperator(BinaryOperator &I) {

  if (!I.getOperand(0)->getType()->isIntegerTy(32) ||
      !I.getOperand(1)->getType()->isIntegerTy(32))
    return;

  // if (I.hasNoSignedWrap() || I.hasNoUnsignedWrap())
  //   return;

  if (I.getOpcode() != Instruction::Add && I.getOpcode() != Instruction::Sub &&
      I.getOpcode() != Instruction::Mul && I.getOpcode() != Instruction::UDiv &&
      I.getOpcode() != Instruction::SDiv) {
    return;
  }

  // TODO implement!!!
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();
  BasicBlock *OriginBB = I.getParent();

  Instruction *NextInst = I.getNextNode();
  if (!NextInst)
    return;

  BasicBlock *ContBB = OriginBB->splitBasicBlock(NextInst, "orgin_cont");
  IRBuilder<> IRB(OriginBB);

  OriginBB->getTerminator()->eraseFromParent();

  if (mode == 0) {

    // basic block construction (overflow / signed bound / unsigned bound)
    BasicBlock *OverflowCheckBB = BasicBlock::Create(Ctx, "overflow_check", F);\
    IRB.CreateBr(OverflowCheckBB);

    IRBuilder<> IR1(OverflowCheckBB);

    // overflow check branch
    Value *OverflowCond = nullptr;
    std::string overflow_tag = "int_overflow";

    switch (I.getOpcode()) {
    case Instruction::Add:
      OverflowCond = getAddOverflowCondition(I, IR1);
      break;
    case Instruction::Sub:
      OverflowCond = getSubOverflowCondition(I, IR1);
      break;
    case Instruction::Mul:
      OverflowCond = getMulOverflowCondition(I, IR1);
      break;
    case Instruction::SDiv:
    case Instruction::UDiv:
      overflow_tag = "int_divided_by_zero";
      OverflowCond = getDividedByZeroCondition(I, IR1);
      break;
    default:
      break;
    }

    if (!OverflowCond)
      return;

    BranchInst *OverflowCheckBranch =
        IR1.CreateCondBr(OverflowCond, ContBB, ContBB);

    // set metadata
    OverflowCheckBranch->setMetadata(
        "symros.check",
        MDNode::get(Ctx, MDString::get(Ctx, overflow_tag.c_str())));
    OverflowCheckBranch->setDebugLoc(I.getDebugLoc());
  } else if (mode == 1) { 
    BasicBlock *MidBB = BasicBlock::Create(I.getContext(), "guide.mid", I.getFunction());
    getBoundCheckChain(I, IRB, OriginBB, MidBB);
    getSignedBoundCheckChain(I, IRB, MidBB, ContBB);
  }
}

void OverflowChecker::visitLoadInst(LoadInst &I) {
  if (!I.getPointerOperand()->getType()->isPointerTy())
    return;
  
  auto *GEP = dyn_cast<GetElementPtrInst>(I.getPointerOperand());
  if (!GEP) return;

  Value *Index = GEP->getOperand(GEP->getNumOperands() - 1);
  if (!Index->getType()->isIntegerTy()) return;


  if (mode == 1) {
    BasicBlock *OriginBB = I.getParent();
    Instruction *NextInst = I.getNextNode();
    if (!NextInst)
      return;

    BasicBlock *ContBB = OriginBB->splitBasicBlock(NextInst, "orgin_cont");
    IRBuilder<> IRB(OriginBB);
    OriginBB->getTerminator()->eraseFromParent();

    BasicBlock *MidBB = BasicBlock::Create(I.getContext(), "guide.mid", I.getFunction());
    getBoundCheckChain(I, Index, IRB, OriginBB, MidBB);
    getSignedBoundCheckChain(I, Index, IRB, MidBB, ContBB);
  }
}

void OverflowChecker::visitStoreInst(StoreInst &I) {
  if (!I.getPointerOperand()->getType()->isPointerTy())
    return;

  auto *GEP = dyn_cast<GetElementPtrInst>(I.getPointerOperand());
  if (!GEP) return;

  Value *Index = GEP->getOperand(GEP->getNumOperands() - 1);
  if (!Index->getType()->isIntegerTy()) return;

  if (mode == 1) {
    BasicBlock *OriginBB = I.getParent();
    Instruction *NextInst = I.getNextNode();
    if (!NextInst)
      return;

    BasicBlock *ContBB = OriginBB->splitBasicBlock(NextInst, "orgin_cont");
    IRBuilder<> IRB(OriginBB);
    OriginBB->getTerminator()->eraseFromParent();

    BasicBlock *MidBB = BasicBlock::Create(I.getContext(), "guide.mid", I.getFunction());
    getBoundCheckChain(I, Index, IRB, OriginBB, MidBB);
    getSignedBoundCheckChain(I, Index, IRB, MidBB, ContBB);
  }
}

Value *OverflowChecker::getAddOverflowCondition(BinaryOperator &I,
                                                IRBuilder<> &IRB) {

  Value *a = I.getOperand(0);
  Value *b = I.getOperand(1);

  LLVMContext &Ctx = I.getContext();
  Type *Ty = a->getType();

  assert(Ty->isIntegerTy() && "only integer add supported");

  /*
   * signed overflow
   */
  unsigned BitWidth = Ty->getIntegerBitWidth();

  APInt IntMax = APInt::getSignedMaxValue(BitWidth); // INT_MAX
  APInt IntMin = APInt::getSignedMinValue(BitWidth); // INT_MIN

  Value *INT_MAX_V = ConstantInt::get(Ctx, IntMax);
  Value *INT_MIN_V = ConstantInt::get(Ctx, IntMin);
  Value *ZERO = ConstantInt::get(Ty, 0);

  // cond: (b > 0 && a > INT_MAX - b)
  Value *b_gt_0 = IRB.CreateICmpSGT(b, ZERO);
  Value *intmax_minus_b = IRB.CreateSub(INT_MAX_V, b);
  Value *a_gt_intmax_minus_b = IRB.CreateICmpSGT(a, intmax_minus_b);
  Value *pos_overflow = IRB.CreateAnd(b_gt_0, a_gt_intmax_minus_b);

  // cond: (b < 0 && a < INT_MIN - b)
  Value *b_lt_0 = IRB.CreateICmpSLT(b, ZERO);
  Value *intmin_minus_b = IRB.CreateSub(INT_MIN_V, b);
  Value *a_lt_intmin_minus_b = IRB.CreateICmpSLT(a, intmin_minus_b);
  Value *neg_overflow = IRB.CreateAnd(b_lt_0, a_lt_intmin_minus_b);

  // cond: (pos || neg)
  Value *signed_overflow = IRB.CreateOr(pos_overflow, neg_overflow);

  /*
   * unsigned overflow
   */
  Value *unsigned_sum = IRB.CreateAdd(a, b);
  // cond: (a + b) < a
  Value *unsigned_overflow = IRB.CreateICmpULT(unsigned_sum, a);

  /*
   * final overflow condition
   */
  return IRB.CreateOr(signed_overflow, unsigned_overflow);
}

Value *OverflowChecker::getSubOverflowCondition(BinaryOperator &I,
                                                IRBuilder<> &IRB) {
  Value *a = I.getOperand(0);
  Value *b = I.getOperand(1);

  LLVMContext &Ctx = I.getContext();

  Type *Ty = a->getType();
  assert(Ty->isIntegerTy() && "only integer sub supported");
  unsigned BitWidth = Ty->getIntegerBitWidth();

  // ============================
  // signed overflow condition
  // ============================

  APInt IntMax = APInt::getSignedMaxValue(BitWidth);
  APInt IntMin = APInt::getSignedMinValue(BitWidth);

  Value *INT_MAX_V = ConstantInt::get(Ctx, IntMax);
  Value *INT_MIN_V = ConstantInt::get(Ctx, IntMin);
  Value *ZERO = ConstantInt::get(Ty, 0);

  // (b < 0)
  Value *b_lt_0 = IRB.CreateICmpSLT(b, ZERO);

  // (a > INT_MAX + b)
  Value *intmax_plus_b = IRB.CreateAdd(INT_MAX_V, b);
  Value *a_gt_intmax_plus_b = IRB.CreateICmpSGT(a, intmax_plus_b);
  Value *pos_overflow = IRB.CreateAnd(b_lt_0, a_gt_intmax_plus_b);

  // (b > 0)
  Value *b_gt_0 = IRB.CreateICmpSGT(b, ZERO);

  // (a < INT_MIN + b)
  Value *intmin_plus_b = IRB.CreateAdd(INT_MIN_V, b);
  Value *a_lt_intmin_plus_b = IRB.CreateICmpSLT(a, intmin_plus_b);
  Value *neg_overflow = IRB.CreateAnd(b_gt_0, a_lt_intmin_plus_b);

  Value *signed_overflow = IRB.CreateOr(pos_overflow, neg_overflow);

  // ============================
  // unsigned overflow condition
  // ============================

  // a < b  → unsigned underflow
  Value *unsigned_overflow = IRB.CreateICmpULT(a, b);

  // ============================
  // final condition
  // ============================
  return IRB.CreateOr(signed_overflow, unsigned_overflow);
}

Value *OverflowChecker::getMulOverflowCondition(BinaryOperator &I,
                                                IRBuilder<> &IRB) {
  Value *a = I.getOperand(0);
  Value *b = I.getOperand(1);

  LLVMContext &Ctx = I.getContext();

  Type *Ty = a->getType();
  assert(Ty->isIntegerTy() && "only integer mul supported");
  unsigned BitWidth = Ty->getIntegerBitWidth();

  // ============================
  // signed overflow
  // ============================

  Value *ZERO = ConstantInt::get(Ty, 0);

  // for safe check
  Value *a_ne_0 = IRB.CreateICmpNE(a, ZERO);

  Value *prod = IRB.CreateMul(a, b);

  Value *safe_a = IRB.CreateSelect(a_ne_0, a, ConstantInt::get(Ty, 1));
  Value *div = IRB.CreateSDiv(prod, safe_a);
  Value *div_ne_b = IRB.CreateICmpNE(div, b);
  Value *signed_overflow = IRB.CreateAnd(a_ne_0, div_ne_b);


  // ============================
  // Unsigned overflow
  // ============================

  APInt UIntMax = APInt::getMaxValue(BitWidth);
  Value *UINT_MAX_V = ConstantInt::get(Ctx, UIntMax);

  Value *b_ne_0 = IRB.CreateICmpNE(b, ZERO);
  Value *safe_b = IRB.CreateSelect(b_ne_0, b, ConstantInt::get(Ty, 1));
  Value *umax_div_b = IRB.CreateUDiv(UINT_MAX_V, safe_b);
  Value *unsigned_overflow = IRB.CreateAnd(b_ne_0, IRB.CreateICmpUGT(a, umax_div_b));


  // ============================
  // final condtion
  // ============================

  return IRB.CreateOr(signed_overflow, unsigned_overflow);
}

Value *OverflowChecker::getDividedByZeroCondition(llvm::BinaryOperator &I,
                                                  IRBuilder<> &IRB) {

  Value *Divisor = I.getOperand(1);

  Value *Zero = ConstantInt::get(Divisor->getType(), 0);

  return IRB.CreateICmpEQ(Divisor, Zero);
}

void
OverflowChecker::getBoundCheckChain(llvm::BinaryOperator &I, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB) {
  Value *Result = &I;
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();

  uint64_t boundary_values[] = {4294967294, 65536, 256, 16, 8, 4, 2};

  BasicBlock *CurBB = EntryBB;
  bool isFirst = true;

  for (uint64_t BV : boundary_values) {
    IRB.SetInsertPoint(CurBB);

    Value *Boundary = ConstantInt::get(Result->getType(), BV, true);
    Value *Cmp = IRB.CreateICmpUGT(Result, Boundary);

    BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

    BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);

    if (isFirst) {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "unsigned_bound_check_start")));
      isFirst = false;
    } else {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "unsigned_bound_check")));
    }
    Br->setDebugLoc(I.getDebugLoc());

    CurBB = NextBB;
  }

  IRB.SetInsertPoint(CurBB);
  IRB.CreateBr(ExitBB);
}

void
OverflowChecker::getSignedBoundCheckChain(llvm::BinaryOperator &I, llvm::IRBuilder<> &IRB, BasicBlock *EntryBB, BasicBlock *ExitBB) {
  Value *Result = &I;
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();

  int64_t boundary_values[] = {2147483646, 65536, 256, 16, 8, 4, 2};

  BasicBlock *CurBB = EntryBB;
  bool isFirst = true;

  for (int64_t BV : boundary_values) {
    
    IRB.SetInsertPoint(CurBB);

    Value *PlusBoundary = ConstantInt::get(Result->getType(), BV, true);
    Value *PlusCmp = IRB.CreateICmpSGT(Result, PlusBoundary);

    Value *MinusBoundary = ConstantInt::get(Result->getType(), -BV, true);
    Value *MinusCmp = IRB.CreateICmpSLT(Result, MinusBoundary);

    Value *Cmp = IRB.CreateOr(PlusCmp, MinusCmp);

    BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

    BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);
    if (isFirst) {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "signed_bound_check_start")));
      isFirst = false;
    } else {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "signed_bound_check")));
    }
    Br->setDebugLoc(I.getDebugLoc());

    CurBB = NextBB;
  }

  IRB.SetInsertPoint(CurBB);
  IRB.CreateBr(ExitBB);
}

void OverflowChecker::getBoundCheckChain(llvm::Instruction &I, llvm::Value *Index, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB) {
  
  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();
  
  uint64_t boundary_values[] = {4294967294, 65536, 256, 16, 8, 4, 2};

  BasicBlock *CurBB = EntryBB;
  bool isFirst = true;

  for (uint64_t BV : boundary_values) {
    IRB.SetInsertPoint(CurBB);

    Value *Boundary = ConstantInt::get(Index->getType(), BV, true);
    Value *Cmp = IRB.CreateICmpUGT(Index, Boundary);

    BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

    BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);

    if (isFirst) {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "unsigned_bound_check_start")));
      isFirst = false;
    } else {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "unsigned_bound_check")));
    }
    Br->setDebugLoc(I.getDebugLoc());

    CurBB = NextBB;
  }

  IRB.SetInsertPoint(CurBB);
  IRB.CreateBr(ExitBB);
}

void OverflowChecker::getSignedBoundCheckChain(llvm::Instruction &I, llvm::Value *Index, llvm::IRBuilder<> &IRB, BasicBlock *EntryBB, BasicBlock *ExitBB) {

  LLVMContext &Ctx = I.getContext();
  Function *F = I.getFunction();

  int64_t boundary_values[] = {2147483646, 65536, 256, 16, 8, 4, 2};

  BasicBlock *CurBB = EntryBB;
  bool isFirst = true;

  for (int64_t BV : boundary_values) {
    
    IRB.SetInsertPoint(CurBB);

    Value *PlusBoundary = ConstantInt::get(Index->getType(), BV, true);
    Value *PlusCmp = IRB.CreateICmpSGT(Index, PlusBoundary);

    Value *MinusBoundary = ConstantInt::get(Index->getType(), -BV, true);
    Value *MinusCmp = IRB.CreateICmpSLT(Index, MinusBoundary);

    Value *Cmp = IRB.CreateOr(PlusCmp, MinusCmp);

    BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

    BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);
    if (isFirst) {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "signed_bound_check_start")));
      isFirst = false;
    } else {
      Br->setMetadata(
        "symros.guide",
        MDNode::get(Ctx, MDString::get(Ctx, "signed_bound_check")));
    }
    Br->setDebugLoc(I.getDebugLoc());

    CurBB = NextBB;
  }

  IRB.SetInsertPoint(CurBB);
  IRB.CreateBr(ExitBB);
}

// void OverflowChecker::getBoundCheckChain(llvm::StoreInst &I, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB) {
//   Value *Address = I.getPointerOperand();
//   LLVMContext &Ctx = I.getContext();
//   Function *F = I.getFunction();

//   IRB.SetInsertPoint(EntryBB);
//   Value *AddressInt = IRB.CreatePtrToInt(Address, IRB.getInt64Ty());

//   uint64_t boundary_values[] = {4294967294, 65536, 256, 16, 8, 4, 2};

//   BasicBlock *CurBB = EntryBB;
//   bool isFirst = true;

//   for (uint64_t BV : boundary_values) {
//     IRB.SetInsertPoint(CurBB);

//     Value *Boundary = ConstantInt::get(AddressInt->getType(), BV, true);
//     Value *Cmp = IRB.CreateICmpUGT(AddressInt, Boundary);

//     BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

//     BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);

//     if (isFirst) {
//       Br->setMetadata(
//         "symros.guide",
//         MDNode::get(Ctx, MDString::get(Ctx, "unsigned_bound_check_start")));
//       isFirst = false;
//     } else {
//       Br->setMetadata(
//         "symros.guide",
//         MDNode::get(Ctx, MDString::get(Ctx, "unsigned_bound_check")));
//     }
//     Br->setDebugLoc(I.getDebugLoc());

//     CurBB = NextBB;
//   }

//   IRB.SetInsertPoint(CurBB);
//   IRB.CreateBr(ExitBB);
// }

// void OverflowChecker::getSignedBoundCheckChain(llvm::StoreInst &I, llvm::IRBuilder<> &IRB, BasicBlock *EntryBB, BasicBlock *ExitBB) {
//   Value *Address = I.getPointerOperand();
//   LLVMContext &Ctx = I.getContext();
//   Function *F = I.getFunction();

//   IRB.SetInsertPoint(EntryBB);
//   Value *AddressInt = IRB.CreatePtrToInt(Address, IRB.getInt64Ty());

//   int64_t boundary_values[] = {2147483646, 65536, 256, 16, 8, 4, 2};

//   BasicBlock *CurBB = EntryBB;
//   bool isFirst = true;

//   for (int64_t BV : boundary_values) {
    
//     IRB.SetInsertPoint(CurBB);

//     Value *PlusBoundary = ConstantInt::get(AddressInt->getType(), BV, true);
//     Value *PlusCmp = IRB.CreateICmpSGT(AddressInt, PlusBoundary);

//     Value *MinusBoundary = ConstantInt::get(AddressInt->getType(), -BV, true);
//     Value *MinusCmp = IRB.CreateICmpSLT(AddressInt, MinusBoundary);

//     Value *Cmp = IRB.CreateOr(PlusCmp, MinusCmp);

//     BasicBlock *NextBB = BasicBlock::Create(Ctx, "guide.check", F);

//     BranchInst *Br = IRB.CreateCondBr(Cmp, ExitBB, NextBB);
//     if (isFirst) {
//       Br->setMetadata(
//         "symros.guide",
//         MDNode::get(Ctx, MDString::get(Ctx, "signed_bound_check_start")));
//       isFirst = false;
//     } else {
//       Br->setMetadata(
//         "symros.guide",
//         MDNode::get(Ctx, MDString::get(Ctx, "signed_bound_check")));
//     }
//     Br->setDebugLoc(I.getDebugLoc());

//     CurBB = NextBB;
//   }

//   IRB.SetInsertPoint(CurBB);
//   IRB.CreateBr(ExitBB);
// }

void
OverflowChecker::setMode(int mode) {
  this->mode = mode;
}