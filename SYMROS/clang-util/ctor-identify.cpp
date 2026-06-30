#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/AST/PrettyPrinter.h"
#include "clang/AST/Expr.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <optional>

using namespace clang;

std::string filename;

class CtorIdentifyVisitor
  : public RecursiveASTVisitor<CtorIdentifyVisitor> {
public:
  explicit CtorIdentifyVisitor(ASTContext *Context)
    : Context(Context), SM(Context->getSourceManager()) {}

  bool VisitCXXConstructorDecl(CXXConstructorDecl *CtorDecl) {

    SourceLocation Loc = CtorDecl->getLocation();
    if (!SM.isInMainFile(Loc)) return true;
    
    std::string nodeName = "";
    std::string params = "";

    if (auto record = CtorDecl->getParent()) {
      nodeName = record->getQualifiedNameAsString();
    } else {
        return true;
    }

    for (const auto *param : CtorDecl->parameters()) {
      QualType T = param->getType().getCanonicalType().getUnqualifiedType().getNonReferenceType();
      std::string paramType = T.getAsString();
      std::string paramName = param->getNameAsString();
      std::string typeType = "";

      if (T->isPointerType()) {
        QualType TT = T->getPointeeType();
        paramType = TT.getAsString();
        // TODO how handle multi-pointed types?

        if (TT->isBuiltinType()) {
          typeType = "pointer::builtin";
        } else if (TT->isStructureType()) {
          typeType = "pointer::struct";
        } else if (TT->isClassType()) {
          typeType = "pointer::class";
        } else {
          typeType = "pointer::other";
        }
      } else {
        if (T->isBuiltinType()) {
          typeType = "builtin";
        } else if (T->isStructureType()) {
          typeType = "struct";
        } else if (T->isClassType()) {
          typeType = "class";
        } else {
          typeType = "other";
        }
      }
      
      if (!param->hasDefaultArg()) {   
        params += "[" + paramType + "](" + typeType + ") ";
      }
    }

    std::string is_lifecycle = "false";
    if (hasLifecycleBase(CtorDecl->getParent())) {
      is_lifecycle = "true";
    }

    std::cout << "symros_result$$" << filename << "$$" << nodeName << "$$" << params << "$$" << is_lifecycle << "\n";

    return true;
  }

  bool hasLifecycleBase(const CXXRecordDecl *C) {
    if (!C || !C->hasDefinition()) return false;

    for (const auto base : C->bases()) {
      const CXXRecordDecl *baseDecl = base.getType()->getAsCXXRecordDecl();
      if (!baseDecl) continue;
      std::cout << "!" << std::endl;
      std::string name = baseDecl->getQualifiedNameAsString();
      std::cout << name << std::endl;
      if (name == "rclcpp::LifeCycleNode") {
        return true;
      }

      if (baseDecl->hasDefinition() && hasLifecycleBase(baseDecl)) {
        return true;
      }
    }
    return false;
  }

private:
  ASTContext *Context;
  SourceManager &SM;
};

class CtorIdentifyConsumer : public clang::ASTConsumer {
public:
  explicit CtorIdentifyConsumer(ASTContext *Context)
    : Visitor(Context) {}

  virtual void HandleTranslationUnit(clang::ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
  }
private:
  CtorIdentifyVisitor Visitor;
};

class CtorIdentifyAction : public clang::ASTFrontendAction {
public:
  virtual std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
    clang::CompilerInstance &Compiler, llvm::StringRef InFile) {
      return std::make_unique<CtorIdentifyConsumer>(&Compiler.getASTContext());
  }
};

int main(int argc, char **argv) {

  if (argc <= 1) {
    std::cout << "input file (.cpp, .hpp, .h)" << std::endl;
    return 0;
  }

  filename = argv[1];
  std::ifstream file(argv[1]);
  if (!file) {
    std::cout << "cannot open file: " << argv[1] << std::endl;
  }

  std::ostringstream ss;
  ss << file.rdbuf();

  clang::tooling::runToolOnCodeWithArgs(
    std::make_unique<CtorIdentifyAction>(), 
    ss.str(),
    {
      "-std=c++17", 
      "-I/usr/local/lib/clang/14.0.6/include", 
      "-I/home/ubuntu/symros/SYMROS/ros-nodds/include",
      "-I/opt/ros/symros/include",
      "-I/home/ubuntu/symros/symros_ws/src/navigation2/nav2_amcl/include",
      "-DHAVE_DRAND48=1"
    },
    argv[1]
  );
}