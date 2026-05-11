#include "stdafx.h"
#include "BodyBasics.h"

CBodyBasics::CBodyBasics() :
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_lastFrameTime(0),
    m_lastTrackedBody(),
    m_hasTrackedBodyFrame(false)
{
    ZeroMemory(&m_lastTrackedBody, sizeof(m_lastTrackedBody));
}

CBodyBasics::~CBodyBasics()
{
    Shutdown();
}

bool CBodyBasics::Initialize(HINSTANCE hInstance, int nCmdShow)
{
    if (!m_renderer.Initialize(hInstance, nCmdShow, this, CBodyBasics::MessageRouter)) return false;
    

    return SUCCEEDED(InitializeDefaultSensor());
}

void CBodyBasics::Shutdown()
{
    SafeRelease(m_pBodyFrameReader);
    SafeRelease(m_pCoordinateMapper);

    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

void CBodyBasics::UpdateData()
{
    TrackedBodyFrame trackedBody = {};
    if (!TryGetTrackedBodyFrame(m_lastFrameTime, trackedBody))
    {
        m_hasTrackedBodyFrame = false;
        return;
    }

    m_lastTrackedBody = trackedBody;
    m_hasTrackedBodyFrame = true;
}

void CBodyBasics::RecordData()
{
    if (!m_hasTrackedBodyFrame)
    {
        return;
    }

    m_storage.Record(m_lastFrameTime, m_lastTrackedBody);
}

bool CBodyBasics::SaveRecordedData() const
{
    return m_storage.SaveNextRecording();
}

LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;

    if (WM_NCCREATE == uMsg)
    {
        CREATESTRUCTW* createStruct = reinterpret_cast<CREATESTRUCTW*>(lParam);
        pThis = reinterpret_cast<CBodyBasics*>(createStruct->lpCreateParams);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return DefWindowProcW(hWnd, uMsg, wParam, lParam);
}

LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_SIZE:
            m_renderer.LayoutControls();
            return 0;

        case WM_CLOSE:
            DestroyWindow(hWnd);
            return 0;

        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }

    return DefWindowProcW(hWnd, message, wParam, lParam);
}

HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        m_renderer.SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

bool CBodyBasics::TryGetTrackedBodyFrame(INT64& frameTime, TrackedBodyFrame& trackedBody)
{
    if (!m_pBodyFrameReader)
    {
        return false;
    }

    IBodyFrame* pBodyFrame = NULL;
    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
    if (FAILED(hr))
    {
        SafeRelease(pBodyFrame);
        return false;
    }

    hr = pBodyFrame->get_RelativeTime(&frameTime);

    IBody* bodyInterfaces[BODY_COUNT] = {0};
    if (SUCCEEDED(hr))
    {
        hr = pBodyFrame->GetAndRefreshBodyData(_countof(bodyInterfaces), bodyInterfaces);
    }

    if (SUCCEEDED(hr))
    {
        for (int i = 0; i < BODY_COUNT; ++i)
        {
            IBody* pBody = bodyInterfaces[i];
            if (!pBody)
            {
                continue;
            }

            BOOLEAN bTracked = false;
            hr = pBody->get_IsTracked(&bTracked);
            if (FAILED(hr) || !bTracked)
            {
                continue;
            }

            trackedBody.tracked = true;
            trackedBody.leftHandState = HandState_Unknown;
            trackedBody.rightHandState = HandState_Unknown;
            pBody->get_HandLeftState(&trackedBody.leftHandState);
            pBody->get_HandRightState(&trackedBody.rightHandState);
            pBody->GetJoints(_countof(trackedBody.joints), trackedBody.joints);
            break;
        }
    }

    for (int i = 0; i < _countof(bodyInterfaces); ++i)
    {
        SafeRelease(bodyInterfaces[i]);
    }

    SafeRelease(pBodyFrame);
    return SUCCEEDED(hr);
}
