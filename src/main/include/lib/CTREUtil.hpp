
#include "lib/TalonSRXMap.hpp"
#include "lib/VictorSPXMap.hpp"

class CTREUtil {
public:
    WPI_TalonSRX & CreateTalonSRX(rj::TalonSRXMap &map, ctre::phoenix::motorcontrol::can::TalonSRXConfiguration &config) {
        WPI_TalonSRX srx(map.id);
        srx.ConfigAllSettings(config);

        srx.SetInverted(map.invertType);
        srx.SetNeutralMode(map.neutralMode);

        return srx;
    }

    WPI_VictorSPX & CreateVictorSPX(rj::TalonSRXMap &map, ctre::phoenix::motorcontrol::can::VictorSPXConfiguration &config) {
        WPI_VictorSPX spx(map.id);
        spx.ConfigAllSettings(config);

        spx.SetInverted(map.invertType);
        spx.SetNeutralMode(map.neutralMode);

        return spx;
    }
};
