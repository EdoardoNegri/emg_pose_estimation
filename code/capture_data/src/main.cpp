#include "stdafx.h"
#include "BodyBasics.h"
int APIENTRY wWinMain(
    _In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE,
    _In_ LPWSTR,
    _In_ int nShowCmd)
{
    CBodyBasics application;
    if (!application.Initialize(hInstance, nShowCmd)) return 0;
    

    MSG msg = {0};
    while (WM_QUIT != msg.message)
    {
        application.UpdateData();

#ifdef ENABLE_RECORDING
        application.RecordData();
#endif

#ifdef ENABLE_RENDERING
        application.Render();
#endif

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

#ifdef ENABLE_RECORDING
    application.SaveRecordedData();
#endif

    return static_cast<int>(msg.wParam);
}
