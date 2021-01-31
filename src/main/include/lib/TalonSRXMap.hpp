#include <ctre/Phoenix.h>
#include <lib/ctreJsonSerde.hpp>
#include <adi/ADIS16470_IMU.h>
#include <wpi/json.h>

namespace rj
{
    class TalonSRXMap
    {
    public:
        int id;
        ctre::phoenix::motorcontrol::InvertType invertType;
        ctre::phoenix::motorcontrol::NeutralMode neutralMode;
    };

    inline void
    to_json(wpi::json &nlohmann_json_j,
            const TalonSRXMap &nlohmann_json_t)
    {
        nlohmann_json_j["id"] =
            nlohmann_json_t.id;
        nlohmann_json_j["invertType"] =
            nlohmann_json_t.invertType;
        nlohmann_json_j["neutralMode"] =
            nlohmann_json_t.neutralMode;
    }
    inline void
    from_json(const wpi::json &nlohmann_json_j,
              TalonSRXMap &nlohmann_json_t)
    {
        wpi::adl_serializer<
            decltype(nlohmann_json_t.id),
            void>::from_json(nlohmann_json_j.at("id"),
                             nlohmann_json_t.id);
        wpi::adl_serializer<
            decltype(nlohmann_json_t.invertType),
            void>::from_json(nlohmann_json_j.at("invertType"),
                             nlohmann_json_t.invertType);
        wpi::adl_serializer<
            decltype(nlohmann_json_t.neutralMode),
            void>::from_json(nlohmann_json_j.at("neutralMode"),
                             nlohmann_json_t.neutralMode);
    }

} // namespace rj