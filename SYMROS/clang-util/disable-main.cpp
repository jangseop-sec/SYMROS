// inject executor code to project main function
// TODO 1. annotate existed main (deactivate)
// TODO 2. rewrite main function to executor main function
// TODO 3. inject executor's header 

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/AST/PrettyPrinter.h"
#include "clang/AST/Expr.h"
#include "clang/Rewrite/Core/Rewriter.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Lex/Lexer.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <optional>
#include <filesystem>

using namespace clang;
std::string filename;

class DisableMainVisitor
  : public RecursiveASTVisitor<DisableMainVisitor> {
public:
  explicit DisableMainVisitor(ASTContext *Context, Rewriter &R)
    : Context(Context), SM(Context->getSourceManager()), LO(Context->getLangOpts()), TheRewriter(R) {}
  
  bool visitFunctionDecl(FunctionDecl * FD) {
    
    if (!FD) return true;
    
    SourceLocation Loc = FD->getLocation();
    if (!SM.isInMainFile(Loc)) return true;

    if (!FD->isThisDeclarationADefinition()) return true;
    if (!FD->isMain()) return true;
    
    SourceLocation Begin = SM.getExpansionLoc(FD->getBeginLoc());

    const Stmt * Body = FD->getBody();
    if (!Body) return true;

    SourceLocation BodyEnd = SM.getExpansionLoc(Body->getEndLoc());
    SourceLocation EndAfter = Lexer::getLocForEndOfToken(BodyEnd, 0, SM, LO);
    
    llvm::outs() << "[clang-util] disable exsited main function: " << filename << "\n";
    TheRewriter.InsertText(Begin, "#if !SYMROS_TEST // disable by symros\n", true, true);
    TheRewriter.InsertText(EndAfter, "\n#endif // disables by symros\n");

    FileID FID = SM.getMainFileID();
    SourceLocation EoFLoc = SM.getLocForEndOfFile(FID);

    TheRewriter.InsertText(EoFLoc, "#if SYMROS_TEST\n/* SYMROS_MAIN */\n#endif", true, true);
    
    return true;
  }

private:
  ASTContext *Context;
  SourceManager &SM;
  const LangOptions &LO;
  Rewriter &TheRewriter;
};

class DisableMainConsumer : public clang::ASTConsumer {
public:
  explicit DisableMainConsumer(ASTContext *Context, Rewriter &R)
    : Visitor(Context, R) {}

  virtual void HandleTranslationUnit(clang::ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
  }
private:
  DisableMainVisitor Visitor;
};

class DisableMainAction : public clang::ASTFrontendAction {
public:
  virtual std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
    clang::CompilerInstance &Compiler, llvm::StringRef InFile) {
      return std::make_unique<DisableMainConsumer>(&Compiler.getASTContext(), TheRewriter);
  }
private:
  Rewriter TheRewriter;
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

  // Get include floder
  std::vector<std::string> args = {
    "-std=c++17", 
    "-DHAVE_DRAND48=1",
    "-I/usr/local/lib/clang/14.0.6/include", 
    "-I/home/ubuntu/symros/SYMROS/ros-nodds/include",
    "-I/opt/ros/symros/include/rosidl_runtime_cpp"
  };

  std::ifstream include_paths("./include_paths.txt");
  std::string line;
  while (std::getline(include_paths, line)) {
    args.push_back("-I" + line);
    std::cout << line << std::endl;
  }

  for (auto path : args) {
    std::cout << path << std::endl;
  }

  clang::tooling::runToolOnCodeWithArgs(
    std::make_unique<DisableMainAction>(), 
    ss.str(),
    args,
    argv[1]
  );
}