#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>

struct OwnshipFrame
{
    uint64_t frameNumber = 0;
    double Latitude = 0.0;
    double Longitude = 0.0;
    float Altitude = 0.0f;
    float Yaw = 0.0f;
    float Pitch = 0.0f;
    float Roll = 0.0f;

    bool operator==(const OwnshipFrame& other) const
    {
        return Latitude == other.Latitude &&
               Longitude == other.Longitude &&
               Altitude == other.Altitude &&
               Yaw == other.Yaw &&
               Pitch == other.Pitch &&
               Roll == other.Roll;
    }
    bool operator!=(const OwnshipFrame& other) const { return !(*this == other); }
};

class OwnshipSmoother
{
public:
    // One AddFrame per 60 Hz Host simulation frame.
    void AddFrame(const OwnshipFrame& frame)
    {
        OwnshipFrame copy = frame;
        copy.frameNumber = ++frameNumber_;
        history_.push_back(copy);
        if (history_.size() > 5)
            history_.pop_front();
    }

    // Returns one delayed output for each input after 5 frames of startup.
    // 2-frame (33.3 ms at 60 Hz) latency.
    bool GetOutput(OwnshipFrame& output) const
    {
        if (history_.size() < 5)
            return false;

        OwnshipFrame corrected[5];
        for (size_t i = 0; i < 5; ++i)
            corrected[i] = CorrectDuplicate(i);

        // Symmetric binomial FIR: preserves constant velocity and has no
        // additional group delay beyond the 2-frame history lookahead.
        constexpr double weights[5] = {1.0, 4.0, 6.0, 4.0, 1.0};
        constexpr double denominator = 16.0;

        output = history_[2];
        double lat = 0.0, lon = 0.0, alt = 0.0;
        for (size_t i = 0; i < 5; ++i)
        {
            lat += weights[i] * corrected[i].Latitude;
            lon += weights[i] * corrected[i].Longitude;
            alt += weights[i] * corrected[i].Altitude;
        }
        output.Latitude = lat / denominator;
        output.Longitude = lon / denominator;
        output.Altitude = static_cast<float>(alt / denominator);

        // Smooth wrapped Euler angles around the center sample.
        output.Yaw = SmoothAngle(corrected, &OwnshipFrame::Yaw);
        output.Pitch = SmoothAngle(corrected, &OwnshipFrame::Pitch);
        output.Roll = SmoothAngle(corrected, &OwnshipFrame::Roll);
        return true;
    }

    void Reset()
    {
        history_.clear();
        frameNumber_ = 0;
    }

private:
    static double Lerp(double a, double b, double t)
    {
        return a + (b - a) * t;
    }

    static double LerpAngle(double a, double b, double t)
    {
        return a + std::remainder(b - a, 360.0) * t;
    }

    OwnshipFrame CorrectDuplicate(size_t index) const
    {
        OwnshipFrame result = history_[index];
        if (index == 0 || history_[index] != history_[index - 1])
            return result;

        // Find distinct samples bracketing the repeated run.
        size_t before = index - 1;
        while (before > 0 && history_[before - 1] == history_[index])
            --before;

        size_t after = index + 1;
        while (after < history_.size() && history_[after] == history_[index])
            ++after;

        if (after == history_.size())
            return result; // No future distinct sample yet.

        const auto& a = history_[before];
        const auto& b = history_[after];
        const double span = double(b.frameNumber - a.frameNumber);
        if (span <= 0.0)
            return result;
        const double t = double(result.frameNumber - a.frameNumber) / span;

        result.Latitude = LerpAngle(a.Latitude, b.Latitude, t);
        result.Longitude = LerpAngle(a.Longitude, b.Longitude, t);
        result.Altitude = static_cast<float>(Lerp(double(a.Altitude), double(b.Altitude), t));
        result.Yaw = static_cast<float>(LerpAngle(a.Yaw, b.Yaw, t));
        result.Pitch = static_cast<float>(LerpAngle(a.Pitch, b.Pitch, t));
        result.Roll = static_cast<float>(LerpAngle(a.Roll, b.Roll, t));
        return result;
    }

    static float SmoothAngle(const OwnshipFrame (&frames)[5],
                             float OwnshipFrame::*field)
    {
        constexpr double weights[5] = {1.0, 4.0, 6.0, 4.0, 1.0};
        const double center = frames[2].*field;
        double sum = 0.0;
        for (size_t i = 0; i < 5; ++i)
            sum += weights[i] * (center + std::remainder(double(frames[i].*field) - center, 360.0));
        return static_cast<float>(std::remainder(sum / 16.0, 360.0));
    }

    std::deque<OwnshipFrame> history_;
    uint64_t frameNumber_ = 0;
};
