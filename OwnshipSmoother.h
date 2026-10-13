#include <deque>
#include <cmath>
#include <algorithm>

static uint64_t gFrameNumber = 0;

template <typename T>
T Lerp(T a, T b, T alpha)
{
    return a + (b - a) * alpha;
}

template <typename T>
T LerpAngle(T a, T b, T alpha)
{
    T delta = std::remainder(b - a, T(360));
    return Lerp(a, a + delta, alpha);
}

struct OwnshipFrame
{
    uint64_t frameNumber;
    double Latitude;  // degrees
    double Longitude; // degrees
    float Altitude;   // feet
    float Yaw;        // degrees
    float Pitch;      // degrees
    float Roll;       // degrees

    bool operator==(const OwnshipFrame& other) const
    {
        return Latitude == other.Latitude &&
               Longitude == other.Longitude &&
               Altitude == other.Altitude &&
               Yaw == other.Yaw &&
               Pitch == other.Pitch &&
               Roll == other.Roll;
    }

    bool operator!=(const OwnshipFrame& other) const
    {
        return !(*this == other);
    }
};

class OwnshipSmoother
{
public:
    void AddFrame(OwnshipFrame& frame)
    {
        frame.frameNumber = ++gFrameNumber;
        history.push_back(frame);

        while (history.size() > 5)
            history.pop_front();
    }

    bool GetOutput(OwnshipFrame& output)
    {
        constexpr size_t delay = 2;

        if (history.size() <= delay)
            return false;

        size_t target = history.size() - 1 - delay;

        output = history[target];

        // Only correct if target equals previous frame.
        if (target == 0 ||
            (history[target - 1] != history[target]))
            return true;

        // Find the previous distinct sample.
        size_t before = target - 1;

        while (before > 0 &&
               (history[before - 1] == history[target]))
        {
            --before;
        }

        // Find the next distinct sample.
        size_t after = target + 1;

        while (after < history.size() &&
               (history[after] == history[target]))
        {
            ++after;
        }

        if (after >= history.size())
            return true; // No future sample yet.

        // Interpolate using frame spacing.
        const auto& a = history[before];
        const auto& b = history[after];

        double span = static_cast<double>(
            b.frameNumber - a.frameNumber);

        if (span <= 0.0)
            return true;

        double alpha =
            static_cast<double>(
                output.frameNumber - a.frameNumber) / span;

        alpha = std::clamp(alpha, 0.0, 1.0);

        output.Latitude  = Lerp(a.Latitude, b.Latitude, alpha);
        output.Longitude = Lerp(a.Longitude, b.Longitude, alpha);
        output.Altitude  = Lerp(a.Altitude, b.Altitude, (float)alpha);

        output.Yaw   = LerpAngle(a.Yaw, b.Yaw, (float)alpha);
        output.Pitch = LerpAngle(a.Pitch, b.Pitch, (float)alpha);
        output.Roll  = LerpAngle(a.Roll, b.Roll, (float)alpha);

        return true;
    }

private:

    std::deque<OwnshipFrame> history;
};
