#include <ctre/Phoenix.h>
#include <lib/ctreJsonSerde.hpp>
#include <adi/ADIS16470_IMU.h>
#include <wpi/json.h>

#include "lib/TalonSRXMap.hpp"
#include "lib/VictorSPXMap.hpp"

namespace rj
{
    class SystemMap
    {
    public:
        TalonSRXMap driveLeft1;
        VictorSPXMap driveLeft2;
        TalonSRXMap driveRight1;
        VictorSPXMap driveRight2;
    };

    inline void
    to_json(wpi::json &nlohmann_json_j,
            const SystemMap &nlohmann_json_t)
    {
        nlohmann_json_j["driveLeft1"] = nlohmann_json_t.driveLeft1;
        nlohmann_json_j["driveLeft2"] = nlohmann_json_t.driveLeft2;
        nlohmann_json_j["driveRight1"] = nlohmann_json_t.driveRight1;
        nlohmann_json_j["driveRight2"] = nlohmann_json_t.driveRight2;
    }
    inline void
    from_json(const wpi::json &nlohmann_json_j,
              SystemMap &nlohmann_json_t)
    {
        wpi::adl_serializer<decltype(nlohmann_json_t.driveLeft1), void>::from_json(nlohmann_json_j.at("driveLeft1"), nlohmann_json_t.driveLeft1);
        wpi::adl_serializer<decltype(nlohmann_json_t.driveLeft2), void>::from_json(nlohmann_json_j.at("driveLeft2"), nlohmann_json_t.driveLeft2);
        wpi::adl_serializer<decltype(nlohmann_json_t.driveRight1), void>::from_json(nlohmann_json_j.at("driveRight1"), nlohmann_json_t.driveRight1);
        wpi::adl_serializer<decltype(nlohmann_json_t.driveRight2), void>::from_json(nlohmann_json_j.at("driveRight2"), nlohmann_json_t.driveRight2);
    }

} // namespace rj