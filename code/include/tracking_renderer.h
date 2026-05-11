#pragma once

#include "resource.h"

struct TrackedBodyFrame
{
    bool tracked;
    Joint joints[JointType_Count];
    HandState leftHandState;
    HandState rightHandState;
};

class TrackingRenderer
{
    static const int cDepthWidth = 512;
    static const int cDepthHeight = 424;

public:
    TrackingRenderer();
    ~TrackingRenderer();

    bool Initialize(HINSTANCE hInstance, int nCmdShow, void* userData, WNDPROC windowProc);
    void Shutdown();

    HWND MainWindow() const;
    void LayoutControls();
    bool SetStatusMessage(_In_z_ WCHAR* message, DWORD showTimeMsec, bool force);
    HRESULT Update(
        INT64 frameTime,
        ICoordinateMapper* coordinateMapper,
        const TrackedBodyFrame& body);

private:
    INT64 m_startTime;
    INT64 m_lastCounter;
    double m_frequency;
    INT64 m_nextStatusTime;
    DWORD m_framesSinceUpdate;

    HWND m_hWnd;
    HWND m_hWndVideoView;
    HWND m_hWndStatus;
    ID2D1Factory* m_pD2DFactory;
    ID2D1HwndRenderTarget* m_pRenderTarget;
    ID2D1SolidColorBrush* m_pBrushJointTracked;
    ID2D1SolidColorBrush* m_pBrushJointInferred;
    ID2D1SolidColorBrush* m_pBrushBoneTracked;
    ID2D1SolidColorBrush* m_pBrushBoneInferred;
    ID2D1SolidColorBrush* m_pBrushHandClosed;
    ID2D1SolidColorBrush* m_pBrushHandOpen;
    ID2D1SolidColorBrush* m_pBrushHandLasso;

    HRESULT EnsureDirect2DResources();
    void DiscardDirect2DResources();
    D2D1_POINT_2F BodyToScreen(ICoordinateMapper* coordinateMapper, const CameraSpacePoint& bodyPoint, int width, int height) const;
    void DrawBody(const Joint* joints, const D2D1_POINT_2F* jointPoints);
    void DrawBone(const Joint* joints, const D2D1_POINT_2F* jointPoints, JointType joint0, JointType joint1);
    void DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);
};
