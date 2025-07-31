// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the STAGDLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// STAGDLL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef STAGDLL_EXPORTS
#define STAGDLL_API __declspec(dllexport)
#else
#define STAGDLL_API __declspec(dllimport)
#endif

// This class is exported from the dll
class STAGDLL_API Cstagdll {
public:
	Cstagdll(void);
	// TODO: add your methods here.
};

extern STAGDLL_API int nstagdll;

STAGDLL_API int fnstagdll(void);
