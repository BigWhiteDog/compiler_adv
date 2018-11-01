//==--- tools/clang-check/ClangInterpreter.cpp - Clang Interpreter tool --------------===//
//===----------------------------------------------------------------------===//
#include <stdio.h>
#include <stdlib.h>

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/Decl.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"

using namespace clang;

class StackFrame {
    /// StackFrame maps Variable Declaration to Value
    /// Which are either integer or addresses (also represented using an Integer value)
    std::map<Decl*, int64_t> mVars;
    std::map<Stmt*, int64_t> mExprs;
    /// The current stmt
    Stmt * mPC;
public:
    StackFrame() : mVars(), mExprs(), mPC() {
    }
    ~StackFrame(){

    }

    bool hasDecl(Decl *decl){
        return mVars.find(decl) != mVars.end();
    }
    void bindDecl(Decl* decl, int64_t val) {
      mVars[decl] = val;
    }
    int64_t getDeclVal(Decl * decl) {
        if(mVars.find(decl) == mVars.end())
        {
            decl->dump();
        }
        assert (mVars.find(decl) != mVars.end());
        return mVars.find(decl)->second;
    }
    void bindStmt(Stmt * stmt, int64_t val) {
        mExprs[stmt] = val;
    }
    int64_t getStmtVal(Stmt * stmt) {
        if(mExprs.find(stmt) == mExprs.end())
        {
            stmt->dump();
        }
        assert (mExprs.find(stmt) != mExprs.end());
        return mExprs[stmt];
    }
    void setPC(Stmt * stmt) {
        mPC = stmt;
    }
    Stmt * getPC() {
        return mPC;
    }
};

/// Heap maps address to a value
// Use system heap handle 
class Heap {

public:
    std::map<Decl*, int64_t> mVars;
    std::map<Decl*, Stmt*> mVarsInit;
    
    std::vector<void*> mPtrs;

    Heap() : mVars() ,mVarsInit(),mPtrs(){
    }
    ~Heap(){
        for(auto x : mPtrs)
            free(x);
    }
    bool hasDecl(Decl *decl){
        return mVars.find(decl) != mVars.end();
    }
    void bindDecl(Decl* decl, int64_t val) {
        mVars[decl] = val;
    }
    int64_t getDeclVal(Decl * decl) {
        assert (mVars.find(decl) != mVars.end());
        return mVars.find(decl)->second;
    }
    void bindInitStmt(Decl * decl, Stmt* val) {
        mVarsInit[decl] = val;
    }
    Stmt* getInitStmt(Decl * decl) {
        assert (mVarsInit.find(decl) != mVarsInit.end());
        return mVarsInit[decl];
    }

};

class Environment {
    
    //std::map<Decl*, int> mHeap; put it into the firststack

    FunctionDecl * mFree;               /// Declartions to the built-in functions
    FunctionDecl * mMalloc;
    FunctionDecl * mInput;
    FunctionDecl * mOutput;

    FunctionDecl * mEntry;
public:
    std::vector<StackFrame> mStack;
    Heap mHeap;
    
    /// Get the declartions to the built-in functions
    Environment() : mFree(NULL), mMalloc(NULL), mInput(NULL), mOutput(NULL), mEntry(NULL), mStack(), mHeap() {
    }

    /// Initialize the Environment
    void init(TranslationUnitDecl * unit) {
        for (TranslationUnitDecl::decl_iterator i =unit->decls_begin(), e = unit->decls_end(); i != e; ++ i) {
            if (FunctionDecl * fdecl = dyn_cast<FunctionDecl>(*i) ) {
                if (fdecl->getName().equals("FREE")) mFree = fdecl;
                else if (fdecl->getName().equals("MALLOC")) mMalloc = fdecl;
                else if (fdecl->getName().equals("GET")) mInput = fdecl;
                else if (fdecl->getName().equals("PRINT")) mOutput = fdecl;
                else if (fdecl->getName().equals("main")) mEntry = fdecl;
            }else if (VarDecl * vardecl = dyn_cast<VarDecl>(*i) ){//WIP: support array
                if (vardecl->hasInit())
                    mHeap.bindInitStmt(vardecl,vardecl->getInit());
                else
                    mHeap.bindDecl(vardecl,0);
            }
        }
        mStack.push_back(StackFrame());
    }

    FunctionDecl * getEntry() {
        return mEntry;
    }

    //set initial value
    void initGlobal(){
        for(auto x :mHeap.mVarsInit){
            int64_t val=mStack.back().getStmtVal(x.second) ;
            mHeap.bindDecl(x.first,val);
        }
    }

    /// !TODO Support comparison operation
    void binop(BinaryOperator *bop) {
        mStack.back().setPC(bop);

        Expr * left = bop->getLHS();
        Expr * right = bop->getRHS();
        int64_t val=0;
        int64_t lval,rval;
        if (bop->isAssignmentOp()) {
            rval = mStack.back().getStmtVal(right);
            mStack.back().bindStmt(left, rval);
            if (DeclRefExpr * declexpr = dyn_cast<DeclRefExpr>(left)) {
                Decl * decl = declexpr->getFoundDecl();
                if(mStack.back().hasDecl(decl))
                    mStack.back().bindDecl(decl, rval);
                else if(mHeap.hasDecl(decl))
                    mHeap.bindDecl(decl,rval);
            }
            else if(UnaryOperator *unop = dyn_cast<UnaryOperator>(left)){//is * operator
                int64_t ptr_val = mStack.back().getStmtVal(unop->getSubExpr());
                QualType pointee_type = unop->getSubExpr()->getType()->getPointeeType();
                if(pointee_type->isCharType()){
                    char * temp_ptr = (char*)ptr_val;
                    *temp_ptr = (char)rval;
                }
                else if(pointee_type->isIntegerType()){//note that char type is also an integertype
                    int64_t * temp_ptr = (int64_t*)ptr_val;
                    *temp_ptr = rval;
                }
                else if (pointee_type->isPointerType()){
                    void ** temp_ptr = (void**)ptr_val;
                    *temp_ptr = (void*)rval;
                }
            }
        }
        else
        {
            lval = mStack.back().getStmtVal(left);
            rval = mStack.back().getStmtVal(right);
            switch(bop-> getOpcode()){
                case BO_EQ:{
                    if (lval==rval)
                        val=1;
                    break;
                }
                case BO_LT:{
                    if (lval<rval)
                        val=1;
                    break;
                }
                case BO_GT:{
                    if (lval>rval)
                        val=1;
                    break;
                }
                case BO_Add:{//WIP:support pointer add
                    Expr* lrexpr[2];
                    lrexpr[0]=left;
                    lrexpr[1]=right;
                    uint c[2]={1,1};//factor of values
                    for (uint i = 0; i < 2; ++i)
                    {
                        if (lrexpr[i]->getType()->isPointerType()){
                            QualType pointee_type = lrexpr[i]->getType()->getPointeeType();
                            if(pointee_type->isCharType()){//change the other factor
                                c[i^1]=1;
                            }else if(pointee_type->isIntegerType()){
                                c[i^1]=8;
                            }
                            else if(pointee_type->isPointerType()){
                                c[i^1]=8;
                            }
                        }
                    }   
                    val=lval*c[0]+rval*c[1];
                    break;
                }
                case BO_Sub:{
                    val=lval-rval;
                    break;
                }
                case BO_Mul:{
                    val=lval*rval;
                    break;
                }
                case BO_Div:{
                    val=lval/rval;
                    break;
                }
                default:
                    ;
            }
        }
        mStack.back().bindStmt(bop,val);
    }

    void unop(UnaryOperator *uop) {
        mStack.back().setPC(uop);
        Expr *subexpr = uop->getSubExpr();
        assert(uop->getOpcode() == UO_Deref);
        int64_t ptr_val = mStack.back().getStmtVal(subexpr);
        int64_t val;
        assert(subexpr->getType()->isPointerType());
        QualType pointee_type=subexpr->getType()->getPointeeType();
        if(pointee_type->isCharType()){
            char * temp_ptr = (char*)ptr_val;
            val = *temp_ptr;
        }
        else if(pointee_type->isIntegerType()){//note that char type is also an integertype
            int64_t * temp_ptr = (int64_t *)ptr_val;
            val = *temp_ptr;
        }
        else if (pointee_type->isPointerType()){
            void ** temp_ptr = (void**)ptr_val;
            val = (int64_t) *temp_ptr;
        }
        mStack.back().bindStmt(uop,val);
    }

    void decl(DeclStmt * declstmt) {
        for (DeclStmt::decl_iterator it = declstmt->decl_begin(), ie = declstmt->decl_end();
                it != ie; ++ it) {
            Decl * decl = *it;
            if (VarDecl * vardecl = dyn_cast<VarDecl>(decl)) {
                if(vardecl->hasInit()){
                    int64_t ival = mStack.back().getStmtVal(vardecl->getInit());
                    mStack.back().bindDecl(vardecl,ival);
                }
                else
                    mStack.back().bindDecl(vardecl, 0);
            }
        }
    }
    void declref(DeclRefExpr * declref) {
        mStack.back().setPC(declref);
        if (declref->getType()->isIntegerType() || declref->getType()->isPointerType()) {
            Decl* decl = declref->getFoundDecl();
            int64_t val;
            if (mHeap.hasDecl(decl))//check if in the heap
                val = mHeap.getDeclVal(decl);
            else
                val = mStack.back().getDeclVal(decl);
            mStack.back().bindStmt(declref, val);
        }
    }

    void cast(CastExpr * castexpr) {
        mStack.back().setPC(castexpr);
        CastKind castkind = castexpr->getCastKind();
        if (castkind == CK_LValueToRValue || castkind == CK_BitCast || castkind == CK_IntegralCast){
            Expr * expr = castexpr->getSubExpr();
            int64_t val = mStack.back().getStmtVal(expr);
            mStack.back().bindStmt(castexpr, val );
        }
        /*else if (castexpr->getType()->isIntegerType()) {
            Expr * expr = castexpr->getSubExpr();
            int64_t val = mStack.back().getStmtVal(expr);
            mStack.back().bindStmt(castexpr, val );
        }
        else if(castexpr->getType()->isCharType()){
            Expr * expr = castexpr->getSubExpr();
            int64_t val = mStack.back().getStmtVal(expr);
            mStack.back().bindStmt(castexpr, val );
        }*/
    }


    /// !TODO Support Function Call
    void call(CallExpr * callexpr) {
        mStack.back().setPC(callexpr);
        int64_t val = 0;
        FunctionDecl * callee = callexpr->getDirectCallee();
        if (callee == mInput) {
            llvm::errs() << "Please Input an Integer Value : ";
            scanf("%d", &val);

            mStack.back().bindStmt(callexpr, val);
        } else if (callee == mOutput) {
            Expr * decl = callexpr->getArg(0);
            val = mStack.back().getStmtVal(decl);
            llvm::errs() << val << '\n';
        } else if (callee == mMalloc){
            Expr * para = callexpr->getArg(0);
            val = mStack.back().getStmtVal(para);
            void * ptr = malloc(val);
            mHeap.mPtrs.push_back(ptr);//record in heap ptrs
            int64_t ptr_val = (int64_t)ptr;
            mStack.back().bindStmt(callexpr,ptr_val);
        } else if (callee == mFree){
            Expr * para = callexpr->getArg(0);
            val = mStack.back().getStmtVal(para);
            void * ptr = (void*)val;
            std::vector<void*>::iterator pit;
            pit = find(mHeap.mPtrs.begin(), mHeap.mPtrs.end(), ptr);
            assert(pit != mHeap.mPtrs.end());
            mHeap.mPtrs.erase(pit);
            free(ptr);
        }
        else {
            StackFrame calleeStack = StackFrame();
            int nargs=callexpr->getNumArgs();
            for (int i = 0; i < nargs; ++i)
            {
                val = mStack.back().getStmtVal(callexpr->getArg(i));
                calleeStack.bindDecl(callee->getParamDecl(i),val);
            }
            calleeStack.setPC(callee->getBody());
            mStack.push_back(calleeStack);
            /// You could add your code here for Function call Return
        }
    }

    void returnStmt(ReturnStmt * returnstmt){
        mStack.back().setPC(returnstmt);
        if(mStack.size()==1){//main return

        }
        StackFrame& callerStack = mStack.rbegin()[1];
        if(Expr* retvalExpr = returnstmt->getRetValue()){
            int64_t val = mStack.back().getStmtVal(retvalExpr);
            callerStack.bindStmt(callerStack.getPC(),val);
        }
    }

    void integerLiteral(IntegerLiteral * intl){
        int64_t val = intl->getValue().getLimitedValue();
        mStack.back().bindStmt(intl,val);
    }
    void characterLiteral(CharacterLiteral * charl){
        int64_t val = charl->getValue();
        mStack.back().bindStmt(charl,val);
    }
    void unaryExprOrTypeTraitExpr(UnaryExprOrTypeTraitExpr* uexpr){
        UnaryExprOrTypeTrait kind = uexpr->getKind();
        if(kind == UETT_SizeOf){
            if(uexpr->getArgumentType()->isCharType()){
                mStack.back().bindStmt(uexpr,1);
            }
            else if(uexpr->getArgumentType()->isIntegerType()){//note that char type is also an integertype
                mStack.back().bindStmt(uexpr,8);
            }
            else if (uexpr->getArgumentType()->isPointerType()){
                mStack.back().bindStmt(uexpr,8);
            }
        }
    }
    void parenExpr(ParenExpr* pe){//() just pass value
        int64_t val = mStack.back().getStmtVal(pe->getSubExpr());
        mStack.back().bindStmt(pe,val);
    }

    int64_t getCondVal(Stmt* condstmt){
        return mStack.back().getStmtVal(condstmt);
    }

};


