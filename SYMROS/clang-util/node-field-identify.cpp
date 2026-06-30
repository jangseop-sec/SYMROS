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
#include <vector>
#include <utility>

using namespace clang;

std::string filename;

class NodeFieldIdentifyVisitor
  : public RecursiveASTVisitor<NodeFieldIdentifyVisitor> {
public:
  explicit NodeFieldIdentifyVisitor(ASTContext *Context)
    : Context(Context), SM(Context->getSourceManager()) {}

	bool VisitCXXRecordDecl(CXXRecordDecl *Decl) {
		if (!Decl->hasDefinition() || Decl->isDependentType()) return true;

    SourceLocation Loc = Decl->getLocation();
    if (!SM.isInMainFile(Loc)) return true;


    std::string className = getRecordPrettyName(Decl);
    if (className == "(anonymous)") return true;

    std::vector<std::pair<std::string, std::string>> fields;
		
		for (auto *Field : Decl->fields()) {
      const auto& Ctx = Field->getASTContext();
      const auto& SM  = Ctx.getSourceManager();

      auto QT = Field->getType();
      fields.push_back(std::make_pair(QT.getDesugaredType(Ctx).getAsString(), Field->getNameAsString()));

      // builtin-type filtering (disable)
      // if (isa<clang::BuiltinType>(QT) || QT.getAsString() == "std::string") {
      //   llvm::outs() << "[Owner] " << Decl->getQualifiedNameAsString() << "\n";
      //   llvm::outs() << "  Field name      : " << Field->getNameAsString() << "\n";
      //   llvm::outs() << "  Field type      : " << QT.getAsString() << "\n\n";
      //   fields.push_back(std::make_pair(Field->getNameAsString(), QT.getDesugaredType(Ctx).getAsString()));
      // } else if (auto *PT = dyn_cast<PointerType>(QT.getTypePtr())) {
      //   if (isa<clang::BuiltinType>(PT->getPointeeType())) {
      //     llvm::outs() << "[Owner] " << Decl->getQualifiedNameAsString() << "\n";
      //     llvm::outs() << "  Field name      : " << Field->getNameAsString() << "\n";
      //     llvm::outs() << "  Field type      : " << QT.getAsString() << "\n\n";
      //     fields.push_back(std::make_pair(Field->getNameAsString(), QT.getDesugaredType(Ctx).getAsString()));
      //   }
      // } else if (auto *CAT = llvm::dyn_cast<ConstantArrayType>(QT.getTypePtr())) {
      //   if (isa<clang::BuiltinType>(CAT->getElementType().getTypePtr())) {
      //     llvm::outs() << "[Owner] " << Decl->getQualifiedNameAsString() << "\n";
      //     llvm::outs() << "  Field name      : " << Field->getNameAsString() << "\n";
      //     llvm::outs() << "  Field type      : " << QT.getAsString() << "\n\n";
      //     fields.push_back(std::make_pair(Field->getNameAsString(), QT.getDesugaredType(Ctx).getAsString()));
      //   }
      // } else if (auto *AT = llvm::dyn_cast<ArrayType>(QT.getTypePtr())) {
      //   if (isa<clang::BuiltinType>(AT->getElementType().getTypePtr())) {
      //     llvm::outs() << "[Owner] " << Decl->getQualifiedNameAsString() << "\n";
      //     llvm::outs() << "  Field name      : " << Field->getNameAsString() << "\n";
      //     llvm::outs() << "  Field type      : " << QT.getAsString() << "\n\n";
      //   }
      // }
		}

    // create .node file
    std::string classFileName = "";
    for (char c : className) {
      if (c == ':') classFileName += "_";
      else classFileName += c; 
    }
    std::ofstream outFile("/home/ubuntu/symros/SYMROS/clang-util/result/" + classFileName + ".class");
    if (!outFile) std::cerr << "file open error\n";

    outFile << className << "\n";
    outFile << filename << "\n";

    for (auto field: fields) {
      outFile << field.first << " " << field.second << "\n";
    }

    outFile.close();
		llvm::outs() << "----------------------------------------------------------------\n";
		return true;
	}

  std::string getRecordPrettyName(const CXXRecordDecl* RD) {
    if (const IdentifierInfo* II = RD->getIdentifier())
      return RD->getQualifiedNameAsString();

    if (RD->isAnonymousStructOrUnion()) {
      if (const TypedefNameDecl* TD = RD->getTypedefNameForAnonDecl())
        return TD->getQualifiedNameAsString();
    }

    if (const clang::DeclContext* DC = RD->getDeclContext()) {
      for (const Decl* D : DC->decls()) {
        if (const auto* TD = dyn_cast<TypedefNameDecl>(D)) {
          QualType U = TD->getUnderlyingType();
          if (const auto* RT = U->getAs<RecordType>())
            if (RT->getDecl() == RD)
              return TD->getQualifiedNameAsString();
        }
      }
    }

    return "(anonymous)";
  }



private:
  ASTContext *Context;
  SourceManager &SM;
};

class NodeFieldIdentifyConsumer : public clang::ASTConsumer {
public:
  explicit NodeFieldIdentifyConsumer(ASTContext *Context)
    : Visitor(Context) {}

  virtual void HandleTranslationUnit(clang::ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
  }
private:
  NodeFieldIdentifyVisitor Visitor;
};

class NodeFieldIdentifyAction : public clang::ASTFrontendAction {
public:
  virtual std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
    clang::CompilerInstance &Compiler, llvm::StringRef InFile) {
      return std::make_unique<NodeFieldIdentifyConsumer>(&Compiler.getASTContext());
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
    std::make_unique<NodeFieldIdentifyAction>(), 
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