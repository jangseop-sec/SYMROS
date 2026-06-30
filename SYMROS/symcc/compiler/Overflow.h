#ifndef OVERFLOW_H
#define OVERFLOW_H

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/InstVisitor.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/ValueMap.h>
#include <llvm/Support/raw_ostream.h>

#include "Runtime.h"

#include <utility>

class OverflowChecker : public llvm::InstVisitor<OverflowChecker> {
public:
  explicit OverflowChecker(llvm::Module &M) : runtime(M) {}
  void visitBinaryOperator(llvm::BinaryOperator &I);
  void visitLoadInst(llvm::LoadInst &I);
  void visitStoreInst(llvm::StoreInst &I);
  void setMode(int mode);
  const Runtime runtime;

private:
  llvm::Value *getAddOverflowCondition(llvm::BinaryOperator &I,
                                       llvm::IRBuilder<> &IRB);
  llvm::Value *getSubOverflowCondition(llvm::BinaryOperator &I,
                                       llvm::IRBuilder<> &IRB);
  llvm::Value *getMulOverflowCondition(llvm::BinaryOperator &I,
                                       llvm::IRBuilder<> &IRB);
  llvm::Value *getDividedByZeroCondition(llvm::BinaryOperator &I,
                                         llvm::IRBuilder<> &IRB);

  void getBoundCheckChain(llvm::BinaryOperator &I, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB);
  void getSignedBoundCheckChain(llvm::BinaryOperator &I, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB);
  
  void getBoundCheckChain(llvm::Instruction &I, llvm::Value *Index, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB);
  void getSignedBoundCheckChain(llvm::Instruction &I, llvm::Value *Index, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB);

  // void getBoundCheckChain(llvm::StoreInst &I, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB);
  // void getSignedBoundCheckChain(llvm::StoreInst &I, llvm::IRBuilder<> &IRB, llvm::BasicBlock *EntryBB, llvm::BasicBlock *ExitBB);

  int mode; // 0:numberic_exception_check, 1:unsigned_bound_check, 2:signed_bound_check
};
#endif