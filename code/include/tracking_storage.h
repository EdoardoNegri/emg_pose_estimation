#pragma once

#include "tracking_renderer.h"
#include <cstdint>
#include <vector>

struct StoredBodyFrame
{
    INT64 frameTime;
    TrackedBodyFrame body;
};

class TrackingStorage
{
public:
    void Record(INT64 frameTime, const TrackedBodyFrame& body);
    bool SaveNextRecording() const;

private:
    bool SaveBinary(const wchar_t* filePath) const;
    std::vector<StoredBodyFrame> m_frames;
};
