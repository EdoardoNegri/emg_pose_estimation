#include "stdafx.h"
#include <algorithm>
#include <strsafe.h>
#include "BodyBasics.h"
#include "tracking_renderer.h"

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;
static const wchar_t c_WindowClassName[] = L"tracking_window";
static const wchar_t c_WindowTitle[] = L"tracking";
static const int c_StatusHeight = 24;

TrackingRenderer::TrackingRenderer() :
    m_startTime(0),
    m_lastCounter(0),
    m_frequency(0),
    m_nextStatusTime(0LL),
    m_framesSinceUpdate(0),
    m_hWnd(NULL),
    m_hWndVideoView(NULL),
    m_hWndStatus(NULL),
    m_pD2DFactory(NULL),
    m_pRenderTarget(NULL),
    m_pBrushJointTracked(NULL),
    m_pBrushJointInferred(NULL),
    m_pBrushBoneTracked(NULL),
    m_pBrushBoneInferred(NULL),
    m_pBrushHandClosed(NULL),
    m_pBrushHandOpen(NULL),
    m_pBrushHandLasso(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_frequency = double(qpf.QuadPart);
    }
}

TrackingRenderer::~TrackingRenderer()
{
    Shutdown();
}

bool TrackingRenderer::Initialize(HINSTANCE hInstance, int nCmdShow, void* userData, WNDPROC windowProc)
{
    WNDCLASSW wc = {};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpfnWndProc = windowProc;
    wc.lpszClassName = c_WindowClassName;

    if (!RegisterClassW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS)
    {
        return false;
    }

    m_hWnd = CreateWindowExW(
        0,
        c_WindowClassName,
        c_WindowTitle,
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_VISIBLE,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        540,
        500,
        NULL,
        NULL,
        hInstance,
        userData);

    if (!m_hWnd)
    {
        return false;
    }

    m_hWndVideoView = CreateWindowExW(
        0,
        L"STATIC",
        L"",
        WS_CHILD | WS_VISIBLE | SS_BLACKFRAME,
        0, 0, 0, 0,
        m_hWnd,
        reinterpret_cast<HMENU>(IDC_VIDEOVIEW),
        hInstance,
        NULL);
    m_hWndStatus = CreateWindowExW(
        0,
        L"STATIC",
        L"",
        WS_CHILD | WS_VISIBLE | SS_SUNKEN,
        0, 0, 0, 0,
        m_hWnd,
        reinterpret_cast<HMENU>(IDC_STATUS),
        hInstance,
        NULL);

    LayoutControls();
    D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);
    ShowWindow(m_hWnd, nCmdShow);
    UpdateWindow(m_hWnd);
    return true;
}

void TrackingRenderer::Shutdown()
{
    DiscardDirect2DResources();
    SafeRelease(m_pD2DFactory);
    m_hWndVideoView = NULL;
    m_hWndStatus = NULL;
    m_hWnd = NULL;
}

HWND TrackingRenderer::MainWindow() const
{
    return m_hWnd;
}

void TrackingRenderer::LayoutControls()
{
    if (!m_hWnd)
    {
        return;
    }

    RECT clientRect = {};
    GetClientRect(m_hWnd, &clientRect);

    const int width = clientRect.right - clientRect.left;
    const int height = clientRect.bottom - clientRect.top;
    const int videoHeight = std::max(0, height - c_StatusHeight);

    if (m_hWndVideoView)
    {
        MoveWindow(m_hWndVideoView, 0, 0, width, videoHeight, TRUE);
    }

    if (m_hWndStatus)
    {
        MoveWindow(m_hWndStatus, 0, videoHeight, width, std::min(c_StatusHeight, height), TRUE);
    }
}

bool TrackingRenderer::SetStatusMessage(_In_z_ const WCHAR* message, DWORD showTimeMsec, bool force)
{
    static INT64 nextStatusTime = 0;
    INT64 now = GetTickCount64();

    if (m_hWnd && (force || nextStatusTime <= now))
    {
        SetWindowTextW(m_hWndStatus, message);
        nextStatusTime = now + showTimeMsec;
        return true;
    }

    return false;
}

HRESULT TrackingRenderer::Update(
    INT64 frameTime,
    ICoordinateMapper* coordinateMapper,
    const TrackedBodyFrame& body)
{
    if (!m_hWnd)
    {
        return S_OK;
    }

    HRESULT hr = EnsureDirect2DResources();
    if (FAILED(hr) || !m_pRenderTarget || !coordinateMapper)
    {
        return hr;
    }

    m_pRenderTarget->BeginDraw();
    m_pRenderTarget->Clear();

    RECT rct = {};
    GetClientRect(m_hWndVideoView, &rct);
    const int width = rct.right;
    const int height = rct.bottom;

    if (body.tracked)
    {
        D2D1_POINT_2F jointPoints[JointType_Count];
        for (int j = 0; j < JointType_Count; ++j)
        {
            jointPoints[j] = BodyToScreen(coordinateMapper, body.joints[j].Position, width, height);
        }

        DrawBody(body.joints, jointPoints);
        DrawHand(body.leftHandState, jointPoints[JointType_HandLeft]);
        DrawHand(body.rightHandState, jointPoints[JointType_HandRight]);
    }

    hr = m_pRenderTarget->EndDraw();
    if (D2DERR_RECREATE_TARGET == hr)
    {
        DiscardDirect2DResources();
        hr = S_OK;
    }

    if (!m_startTime)
    {
        m_startTime = frameTime;
    }

    double fps = 0.0;
    LARGE_INTEGER qpcNow = {0};
    if (m_frequency && QueryPerformanceCounter(&qpcNow))
    {
        if (m_lastCounter)
        {
            m_framesSinceUpdate++;
            fps = m_frequency * m_framesSinceUpdate / double(qpcNow.QuadPart - m_lastCounter);
        }
    }

    WCHAR statusMessage[64];
    StringCchPrintf(statusMessage, _countof(statusMessage), L" FPS = %0.2f    Time = %I64d", fps, (frameTime - m_startTime));

    INT64 now = GetTickCount64();
    if (m_hWnd && (m_nextStatusTime <= now))
    {
        SetWindowTextW(m_hWndStatus, statusMessage);
        m_nextStatusTime = now + 1000;
        m_lastCounter = qpcNow.QuadPart;
        m_framesSinceUpdate = 0;
    }

    return hr;
}

HRESULT TrackingRenderer::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc = {};
        GetClientRect(m_hWndVideoView, &rc);

        const int width = rc.right - rc.left;
        const int height = rc.bottom - rc.top;
        const D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(m_hWndVideoView, size),
            &m_pRenderTarget);

        if (FAILED(hr))
        {
            return hr;
        }

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);
    }

    return hr;
}

void TrackingRenderer::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);
    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);
    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

D2D1_POINT_2F TrackingRenderer::BodyToScreen(ICoordinateMapper* coordinateMapper, const CameraSpacePoint& bodyPoint, int width, int height) const
{
    DepthSpacePoint depthPoint = {0};
    coordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    const float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    const float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;
    return D2D1::Point2F(screenPointX, screenPointY);
}

void TrackingRenderer::DrawBody(const Joint* joints, const D2D1_POINT_2F* jointPoints)
{
    DrawBone(joints, jointPoints, JointType_Head, JointType_Neck);
    DrawBone(joints, jointPoints, JointType_Neck, JointType_SpineShoulder);
    DrawBone(joints, jointPoints, JointType_SpineShoulder, JointType_SpineMid);
    DrawBone(joints, jointPoints, JointType_SpineMid, JointType_SpineBase);
    DrawBone(joints, jointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
    DrawBone(joints, jointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
    DrawBone(joints, jointPoints, JointType_SpineBase, JointType_HipRight);
    DrawBone(joints, jointPoints, JointType_SpineBase, JointType_HipLeft);
    DrawBone(joints, jointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(joints, jointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(joints, jointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(joints, jointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(joints, jointPoints, JointType_WristRight, JointType_ThumbRight);
    DrawBone(joints, jointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(joints, jointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(joints, jointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(joints, jointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(joints, jointPoints, JointType_WristLeft, JointType_ThumbLeft);
    DrawBone(joints, jointPoints, JointType_HipRight, JointType_KneeRight);
    DrawBone(joints, jointPoints, JointType_KneeRight, JointType_AnkleRight);
    DrawBone(joints, jointPoints, JointType_AnkleRight, JointType_FootRight);
    DrawBone(joints, jointPoints, JointType_HipLeft, JointType_KneeLeft);
    DrawBone(joints, jointPoints, JointType_KneeLeft, JointType_AnkleLeft);
    DrawBone(joints, jointPoints, JointType_AnkleLeft, JointType_FootLeft);

    for (int i = 0; i < JointType_Count; ++i)
    {
        const D2D1_ELLIPSE ellipse = D2D1::Ellipse(jointPoints[i], c_JointThickness, c_JointThickness);
        if (joints[i].TrackingState == TrackingState_Inferred)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (joints[i].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

void TrackingRenderer::DrawBone(const Joint* joints, const D2D1_POINT_2F* jointPoints, JointType joint0, JointType joint1)
{
    const TrackingState joint0State = joints[joint0].TrackingState;
    const TrackingState joint1State = joints[joint1].TrackingState;

    if (joint0State == TrackingState_NotTracked || joint1State == TrackingState_NotTracked)
    {
        return;
    }

    if (joint0State == TrackingState_Inferred && joint1State == TrackingState_Inferred)
    {
        return;
    }

    if (joint0State == TrackingState_Tracked && joint1State == TrackingState_Tracked)
    {
        m_pRenderTarget->DrawLine(jointPoints[joint0], jointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
    }
    else
    {
        m_pRenderTarget->DrawLine(jointPoints[joint0], jointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
    }
}

void TrackingRenderer::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    const D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
        case HandState_Closed:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
            break;
        case HandState_Open:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
            break;
        case HandState_Lasso:
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
            break;
    }
}

void CBodyBasics::Render()
{
    if (!m_hasTrackedBodyFrame)
    {
        return;
    }

    m_renderer.Update(
        m_lastFrameTime,
        m_pCoordinateMapper,
        m_lastTrackedBody);
}
