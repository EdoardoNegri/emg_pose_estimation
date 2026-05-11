//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "tracking_renderer.h"
#include "tracking_storage.h"

class CBodyBasics
{ 
public:
    /// <summary>
    /// Constructor
    /// </summary>
    CBodyBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CBodyBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    bool                    Initialize(HINSTANCE hInstance, int nCmdShow);
    void                    Shutdown();
    void                    UpdateData();
    void                    RecordData();
    void                    Render();
    bool                    SaveRecordedData() const;

private:
    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;

    // Body reader
    IBodyFrameReader*       m_pBodyFrameReader;

    INT64                   m_lastFrameTime;
    TrackedBodyFrame        m_lastTrackedBody;
    bool                    m_hasTrackedBodyFrame;
    TrackingStorage         m_storage;

    HRESULT                 InitializeDefaultSensor();
    bool                    TryGetTrackedBodyFrame(INT64& frameTime, TrackedBodyFrame& trackedBody);

public:
    TrackingRenderer        m_renderer;
};

