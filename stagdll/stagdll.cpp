// stagdll.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "stagdll.h"


// This is an example of an exported variable
STAGDLL_API int nstagdll=0;

// This is an example of an exported function.
STAGDLL_API int fnstagdll(void)
{
    return 0;
}

// This is the constructor of a class that has been exported.
Cstagdll::Cstagdll()
{
    return;
}
