#include "stdafx.h"
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include "tracking_storage.h"

namespace
{
    std::int16_t QuantizeMetersToMillimeters(float value)
    {
        const float scaledValue = value * 1000.0f;
        const float minValue = static_cast<float>((std::numeric_limits<std::int16_t>::min)());
        const float maxValue = static_cast<float>((std::numeric_limits<std::int16_t>::max)());
        const float clampedValue = (std::max)(minValue, (std::min)(scaledValue, maxValue));
        return static_cast<std::int16_t>(clampedValue);
    }
}

void TrackingStorage::Record(INT64 frameTime, const TrackedBodyFrame& body)
{
    if (!body.tracked)
    {
        return;
    }

    StoredBodyFrame frame = {};
    frame.frameTime = frameTime;
    frame.body = body;
    m_frames.push_back(frame);
}

bool TrackingStorage::SaveBinary(const wchar_t* filePath) const
{
    std::ofstream output(filePath, std::ios::binary);
    if (!output.is_open())
    {
        return false;
    }

    const char magic[4] = {'E', 'P', '0', '1'};
    output.write(magic, sizeof(magic));

    std::uint32_t frameCount = static_cast<std::uint32_t>(m_frames.size());
    output.write(reinterpret_cast<const char*>(&frameCount), sizeof(frameCount));

    INT64 previousFrameTime = 0;
    for (std::size_t frameIndex = 0; frameIndex < m_frames.size(); ++frameIndex)
    {
        const StoredBodyFrame& frame = m_frames[frameIndex];
        const INT64 rawDeltaTime = (frameIndex == 0) ? 0 : (frame.frameTime - previousFrameTime);
        const INT64 maxDeltaTime = static_cast<INT64>((std::numeric_limits<std::uint32_t>::max)());
        INT64 clampedDeltaTime = rawDeltaTime;
        if (clampedDeltaTime < 0)
        {
            clampedDeltaTime = 0;
        }
        else if (clampedDeltaTime > maxDeltaTime)
        {
            clampedDeltaTime = maxDeltaTime;
        }
        const std::uint32_t deltaTime = static_cast<std::uint32_t>(clampedDeltaTime);

        std::uint8_t trackedJointCount = 0;
        for (int jointIndex = 0; jointIndex < JointType_Count; ++jointIndex)
        {
            if (frame.body.joints[jointIndex].TrackingState == TrackingState_Tracked)
            {
                ++trackedJointCount;
            }
        }

        output.write(reinterpret_cast<const char*>(&deltaTime), sizeof(deltaTime));
        output.write(reinterpret_cast<const char*>(&trackedJointCount), sizeof(trackedJointCount));

        for (int jointIndex = 0; jointIndex < JointType_Count; ++jointIndex)
        {
            const Joint& joint = frame.body.joints[jointIndex];
            if (joint.TrackingState != TrackingState_Tracked)
            {
                continue;
            }

            const std::uint8_t jointId = static_cast<std::uint8_t>(jointIndex);
            const std::int16_t x = QuantizeMetersToMillimeters(joint.Position.X);
            const std::int16_t y = QuantizeMetersToMillimeters(joint.Position.Y);
            const std::int16_t z = QuantizeMetersToMillimeters(joint.Position.Z);

            output.write(reinterpret_cast<const char*>(&jointId), sizeof(jointId));
            output.write(reinterpret_cast<const char*>(&x), sizeof(x));
            output.write(reinterpret_cast<const char*>(&y), sizeof(y));
            output.write(reinterpret_cast<const char*>(&z), sizeof(z));
        }

        previousFrameTime = frame.frameTime;
    }

    return output.good();
}

bool TrackingStorage::SaveNextRecording() const
{
    const std::filesystem::path recordingDirectory = L"data/recordings/raw";
    std::filesystem::create_directories(recordingDirectory);

    int recordingIndex = 0;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(recordingDirectory))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }

        ++recordingIndex;
    }

    const std::filesystem::path recordingPath =
        recordingDirectory / (L"recording_" + std::to_wstring(recordingIndex) + L".bin");
    return SaveBinary(recordingPath.c_str());
}
