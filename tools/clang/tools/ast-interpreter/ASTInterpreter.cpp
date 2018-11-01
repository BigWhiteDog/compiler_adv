//==--- tools/clang-check/ClangInterpreter.cpp - Clang Interpreter tool --------------===//
//===----------------------------------------------------------------------===//

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/EvaluatedExprVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"

using namespace clang;

#include "Environment.h"

class InterpreterVisitor : 
    public EvaluatedExprVisitor<InterpreterVisitor> {
public:
    explicit InterpreterVisitor(const ASTContext &context, Environment * env)
    : EvaluatedExprVisitor(context), mEnv(env) {}
    virtual ~InterpreterVisitor() {}

    virtual void VisitBinaryOperator (BinaryOperator * bop) {
        VisitStmt(bop);
        mEnv->binop(bop);
    }
    virtual void VisitUnaryOperator (UnaryOperator * uop) {
        VisitStmt(uop);
        mEnv->unop(uop);
    }
    virtual void VisitDeclRefExpr(DeclRefExpr * expr) {
        VisitStmt(expr);
        mEnv->declref(expr);
    }
    virtual void VisitCastExpr(CastExpr * expr) {
        VisitStmt(expr);
        mEnv->cast(expr);
    }
    virtual void VisitCallExpr(CallExpr * call) {
        VisitStmt(call);
        mEnv->call(call);
        if (call->getDirectCallee()->hasBody())
        {
            VisitStmt(call->getDirectCallee()->getBody());
            mEnv->mStack.pop_back();
        }
    }
    virtual void VisitDeclStmt(DeclStmt * declstmt) {
        VisitStmt(declstmt);
        mEnv->decl(declstmt);
    }
    virtual void VisitReturnStmt(ReturnStmt * returnstmt){
        VisitStmt(returnstmt);
        mEnv->returnStmt(returnstmt);
    }
    virtual void VisitIfStmt(IfStmt * ifstmt){
        Visit(ifstmt->getCond());
        if(mEnv->getCondVal(ifstmt->getCond())){
            Visit(ifstmt->getThen());
        }
        else if(ifstmt->getElse()){
            Visit(ifstmt->getElse());
        }
    }
    virtual void VisitWhileStmt(WhileStmt * whilestmt){
        Stmt* condstmt = whilestmt->getCond();
        Stmt* bodystmt = whilestmt->getBody();
        
        Visit(condstmt);//get first cond val
        while(mEnv->getCondVal(condstmt))
        {
            if(bodystmt) Visit(bodystmt);
            Visit(condstmt);//update cond
        }
    }
    virtual void VisitForStmt(ForStmt* forstmt){

        Stmt* condstmt = forstmt->getCond();
        Stmt* incstmt = forstmt->getInc();
        Stmt* bodystmt = forstmt->getBody();
        
        if(forstmt->getInit())
            Visit(forstmt->getInit());

        Visit(condstmt);//get first cond val
        while(mEnv->getCondVal(condstmt))          
        {
            if(bodystmt)
                Visit(bodystmt);
            if(incstmt)
                Visit(incstmt);//inc stmt
            Visit(condstmt);//update cond after inc
        }
    }
    virtual void VisitParenExpr(ParenExpr * pe){
        VisitStmt(pe);
        mEnv->parenExpr(pe);
    }
    virtual void VisitIntegerLiteral(IntegerLiteral * intl){
        mEnv->integerLiteral(intl);
    }
    virtual void VisitCharacterLiteral(CharacterLiteral * charl){
        mEnv->characterLiteral(charl);
    }
    virtual void VisitUnaryExprOrTypeTraitExpr(UnaryExprOrTypeTraitExpr* uexpr){
        mEnv->unaryExprOrTypeTraitExpr(uexpr);
    }
private:
    Environment * mEnv;
};

class InterpreterConsumer : public ASTConsumer {
public:
    explicit InterpreterConsumer(const ASTContext& context) : mEnv(),
            mVisitor(context, &mEnv) {
    }
    virtual ~InterpreterConsumer() {}

    virtual void HandleTranslationUnit(clang::ASTContext &Context) {
        TranslationUnitDecl * decl = Context.getTranslationUnitDecl();
        mEnv.init(decl);

        Heap& mHeap = mEnv.mHeap;//visit init stmt
        for(auto x :mHeap.mVarsInit){
            mVisitor.Visit( x.second );
        }
        mEnv.initGlobal();//set initial value

        FunctionDecl * entry = mEnv.getEntry();
        mVisitor.VisitStmt(entry->getBody());
    }
private:
    Environment mEnv;
    InterpreterVisitor mVisitor;
};

class InterpreterClassAction : public ASTFrontendAction {
public: 
    virtual std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
        clang::CompilerInstance &Compiler, llvm::StringRef InFile) {
        return std::unique_ptr<clang::ASTConsumer>(
            new InterpreterConsumer(Compiler.getASTContext()));
    }
};

int main (int argc, char ** argv) {
    if (argc > 1) {
        clang::tooling::runToolOnCode(new InterpreterClassAction, argv[1]);
    }
}
