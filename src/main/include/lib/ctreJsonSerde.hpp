#pragma once
#include "ctre/Phoenix.h"
#include "lib/json.hpp"

// enums
namespace ctre {
namespace phoenix {
NLOHMANN_JSON_SERIALIZE_ENUM(
    CANifierControlFrame, {{CANifierControlFrame::CANifier_Control_1_General,
                            "CANifier_Control_1_General"},
                           {CANifierControlFrame::CANifier_Control_2_PwmOutput,
                            "CANifier_Control_2_PwmOutput"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    CANifierStatusFrame,
    {{CANifierStatusFrame::CANifierStatusFrame_Status_1_General,
      "CANifierStatusFrame_Status_1_General"},
     {CANifierStatusFrame::CANifierStatusFrame_Status_2_General,
      "CANifierStatusFrame_Status_2_General"},
     {CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0,
      "CANifierStatusFrame_Status_3_PwmInputs0"},
     {CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1,
      "CANifierStatusFrame_Status_4_PwmInputs1"},
     {CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2,
      "CANifierStatusFrame_Status_5_PwmInputs2"},
     {CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3,
      "CANifierStatusFrame_Status_6_PwmInputs3"},
     {CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc,
      "CANifierStatusFrame_Status_8_Misc"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    CANifierVelocityMeasPeriod,
    {{CANifierVelocityMeasPeriod::Period_1Ms, "Period_1Ms"},
     {CANifierVelocityMeasPeriod::Period_2Ms, "Period_2Ms"},
     {CANifierVelocityMeasPeriod::Period_5Ms, "Period_5Ms"},
     {CANifierVelocityMeasPeriod::Period_10Ms, "Period_10Ms"},
     {CANifierVelocityMeasPeriod::Period_20Ms, "Period_20Ms"},
     {CANifierVelocityMeasPeriod::Period_25Ms, "Period_25Ms"},
     {CANifierVelocityMeasPeriod::Period_50Ms, "Period_50Ms"},
     {CANifierVelocityMeasPeriod::Period_100Ms, "Period_100Ms"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    ParamEnum,
    {{ParamEnum::eOnBoot_BrakeMode, "eOnBoot_BrakeMode"},
     {ParamEnum::eQuadFilterEn, "eQuadFilterEn"},
     {ParamEnum::eQuadIdxPolarity, "eQuadIdxPolarity"},
     {ParamEnum::eMotionProfileHasUnderrunErr, "eMotionProfileHasUnderrunErr"},
     {ParamEnum::eMotionProfileTrajectoryPointDurationMs,
      "eMotionProfileTrajectoryPointDurationMs"},
     {ParamEnum::eMotionProfileTrajectoryInterpolDis,
      "eMotionProfileTrajectoryInterpolDis"},
     {ParamEnum::eStatusFramePeriod, "eStatusFramePeriod"},
     {ParamEnum::eOpenloopRamp, "eOpenloopRamp"},
     {ParamEnum::eClosedloopRamp, "eClosedloopRamp"},
     {ParamEnum::eNeutralDeadband, "eNeutralDeadband"},
     {ParamEnum::ePeakPosOutput, "ePeakPosOutput"},
     {ParamEnum::eNominalPosOutput, "eNominalPosOutput"},
     {ParamEnum::ePeakNegOutput, "ePeakNegOutput"},
     {ParamEnum::eNominalNegOutput, "eNominalNegOutput"},
     {ParamEnum::eProfileParamSlot_P, "eProfileParamSlot_P"},
     {ParamEnum::eProfileParamSlot_I, "eProfileParamSlot_I"},
     {ParamEnum::eProfileParamSlot_D, "eProfileParamSlot_D"},
     {ParamEnum::eProfileParamSlot_F, "eProfileParamSlot_F"},
     {ParamEnum::eProfileParamSlot_IZone, "eProfileParamSlot_IZone"},
     {ParamEnum::eProfileParamSlot_AllowableErr,
      "eProfileParamSlot_AllowableErr"},
     {ParamEnum::eProfileParamSlot_MaxIAccum, "eProfileParamSlot_MaxIAccum"},
     {ParamEnum::eProfileParamSlot_PeakOutput, "eProfileParamSlot_PeakOutput"},
     {ParamEnum::eClearPositionOnLimitF, "eClearPositionOnLimitF"},
     {ParamEnum::eClearPositionOnLimitR, "eClearPositionOnLimitR"},
     {ParamEnum::eClearPositionOnQuadIdx, "eClearPositionOnQuadIdx"},
     {ParamEnum::eClearPosOnLimitF, "eClearPosOnLimitF"},
     {ParamEnum::eClearPosOnLimitR, "eClearPosOnLimitR"},
     {ParamEnum::eClearPositionOnIdx, "eClearPositionOnIdx"},
     {ParamEnum::eSampleVelocityPeriod, "eSampleVelocityPeriod"},
     {ParamEnum::eSampleVelocityWindow, "eSampleVelocityWindow"},
     {ParamEnum::eFeedbackSensorType, "eFeedbackSensorType"},
     {ParamEnum::eSelectedSensorPosition, "eSelectedSensorPosition"},
     {ParamEnum::eFeedbackNotContinuous, "eFeedbackNotContinuous"},
     {ParamEnum::eRemoteSensorSource, "eRemoteSensorSource"},
     {ParamEnum::eRemoteSensorDeviceID, "eRemoteSensorDeviceID"},
     {ParamEnum::eSensorTerm, "eSensorTerm"},
     {ParamEnum::eRemoteSensorClosedLoopDisableNeutralOnLOS,
      "eRemoteSensorClosedLoopDisableNeutralOnLOS"},
     {ParamEnum::ePIDLoopPolarity, "ePIDLoopPolarity"},
     {ParamEnum::ePIDLoopPeriod, "ePIDLoopPeriod"},
     {ParamEnum::eSelectedSensorCoefficient, "eSelectedSensorCoefficient"},
     {ParamEnum::eForwardSoftLimitThreshold, "eForwardSoftLimitThreshold"},
     {ParamEnum::eReverseSoftLimitThreshold, "eReverseSoftLimitThreshold"},
     {ParamEnum::eForwardSoftLimitEnable, "eForwardSoftLimitEnable"},
     {ParamEnum::eReverseSoftLimitEnable, "eReverseSoftLimitEnable"},
     {ParamEnum::eNominalBatteryVoltage, "eNominalBatteryVoltage"},
     {ParamEnum::eBatteryVoltageFilterSize, "eBatteryVoltageFilterSize"},
     {ParamEnum::eContinuousCurrentLimitAmps, "eContinuousCurrentLimitAmps"},
     {ParamEnum::ePeakCurrentLimitMs, "ePeakCurrentLimitMs"},
     {ParamEnum::ePeakCurrentLimitAmps, "ePeakCurrentLimitAmps"},
     {ParamEnum::eCurrLimit_Amps, "eCurrLimit_Amps"},
     {ParamEnum::eCurrThres_Amps, "eCurrThres_Amps"},
     {ParamEnum::eCurrEnable, "eCurrEnable"},
     {ParamEnum::eCurrThres_Ms, "eCurrThres_Ms"},
     {ParamEnum::eClosedLoopIAccum, "eClosedLoopIAccum"},
     {ParamEnum::eCustomParam, "eCustomParam"},
     {ParamEnum::eStickyFaults, "eStickyFaults"},
     {ParamEnum::eAnalogPosition, "eAnalogPosition"},
     {ParamEnum::eQuadraturePosition, "eQuadraturePosition"},
     {ParamEnum::ePulseWidthPosition, "ePulseWidthPosition"},
     {ParamEnum::eIntegratedSensor, "eIntegratedSensor"},
     {ParamEnum::eMotMag_Accel, "eMotMag_Accel"},
     {ParamEnum::eMotMag_VelCruise, "eMotMag_VelCruise"},
     {ParamEnum::eMotMag_SCurveLevel, "eMotMag_SCurveLevel"},
     {ParamEnum::eLimitSwitchSource, "eLimitSwitchSource"},
     {ParamEnum::eLimitSwitchNormClosedAndDis, "eLimitSwitchNormClosedAndDis"},
     {ParamEnum::eLimitSwitchDisableNeutralOnLOS,
      "eLimitSwitchDisableNeutralOnLOS"},
     {ParamEnum::eLimitSwitchRemoteDevID, "eLimitSwitchRemoteDevID"},
     {ParamEnum::eSoftLimitDisableNeutralOnLOS,
      "eSoftLimitDisableNeutralOnLOS"},
     {ParamEnum::ePulseWidthPeriod_EdgesPerRot,
      "ePulseWidthPeriod_EdgesPerRot"},
     {ParamEnum::ePulseWidthPeriod_FilterWindowSz,
      "ePulseWidthPeriod_FilterWindowSz"},
     {ParamEnum::eYawOffset, "eYawOffset"},
     {ParamEnum::eCompassOffset, "eCompassOffset"},
     {ParamEnum::eBetaGain, "eBetaGain"},
     {ParamEnum::eEnableCompassFusion, "eEnableCompassFusion"},
     {ParamEnum::eGyroNoMotionCal, "eGyroNoMotionCal"},
     {ParamEnum::eEnterCalibration, "eEnterCalibration"},
     {ParamEnum::eFusedHeadingOffset, "eFusedHeadingOffset"},
     {ParamEnum::eStatusFrameRate, "eStatusFrameRate"},
     {ParamEnum::eAccumZ, "eAccumZ"},
     {ParamEnum::eTempCompDisable, "eTempCompDisable"},
     {ParamEnum::eMotionMeas_tap_threshX, "eMotionMeas_tap_threshX"},
     {ParamEnum::eMotionMeas_tap_threshY, "eMotionMeas_tap_threshY"},
     {ParamEnum::eMotionMeas_tap_threshZ, "eMotionMeas_tap_threshZ"},
     {ParamEnum::eMotionMeas_tap_count, "eMotionMeas_tap_count"},
     {ParamEnum::eMotionMeas_tap_time, "eMotionMeas_tap_time"},
     {ParamEnum::eMotionMeas_tap_time_multi, "eMotionMeas_tap_time_multi"},
     {ParamEnum::eMotionMeas_shake_reject_thresh,
      "eMotionMeas_shake_reject_thresh"},
     {ParamEnum::eMotionMeas_shake_reject_time,
      "eMotionMeas_shake_reject_time"},
     {ParamEnum::eMotionMeas_shake_reject_timeout,
      "eMotionMeas_shake_reject_timeout"},
     {ParamEnum::eUnitString, "eUnitString"},
     {ParamEnum::eFeedbackTimeBase, "eFeedbackTimeBase"},
     {ParamEnum::eDefaultConfig, "eDefaultConfig"},
     {ParamEnum::eFastWriteCount, "eFastWriteCount"},
     {ParamEnum::eWriteCount, "eWriteCount"},
     {ParamEnum::eReserved1, "eReserved1"},
     {ParamEnum::eMotorCommutation, "eMotorCommutation"},
     {ParamEnum::eSensorInitStrategy, "eSensorInitStrategy"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    ErrorCode,
    {{ErrorCode::OK, "OK"},
     {ErrorCode::OKAY, "OKAY"},
     {ErrorCode::CAN_MSG_STALE, "CAN_MSG_STALE"},
     {ErrorCode::CAN_TX_FULL, "CAN_TX_FULL"},
     {ErrorCode::TxFailed, "TxFailed"},
     {ErrorCode::InvalidParamValue, "InvalidParamValue"},
     {ErrorCode::CAN_INVALID_PARAM, "CAN_INVALID_PARAM"},
     {ErrorCode::RxTimeout, "RxTimeout"},
     {ErrorCode::CAN_MSG_NOT_FOUND, "CAN_MSG_NOT_FOUND"},
     {ErrorCode::TxTimeout, "TxTimeout"},
     {ErrorCode::CAN_NO_MORE_TX_JOBS, "CAN_NO_MORE_TX_JOBS"},
     {ErrorCode::UnexpectedArbId, "UnexpectedArbId"},
     {ErrorCode::CAN_NO_SESSIONS_AVAIL, "CAN_NO_SESSIONS_AVAIL"},
     {ErrorCode::BufferFull, "BufferFull"},
     {ErrorCode::CAN_OVERFLOW, "CAN_OVERFLOW"},
     {ErrorCode::SensorNotPresent, "SensorNotPresent"},
     {ErrorCode::FirmwareTooOld, "FirmwareTooOld"},
     {ErrorCode::CouldNotChangePeriod, "CouldNotChangePeriod"},
     {ErrorCode::BufferFailure, "BufferFailure"},
     {ErrorCode::FirwmwareNonFRC, "FirwmwareNonFRC"},
     {ErrorCode::GeneralError, "GeneralError"},
     {ErrorCode::GENERAL_ERROR, "GENERAL_ERROR"},
     {ErrorCode::SIG_NOT_UPDATED, "SIG_NOT_UPDATED"},
     {ErrorCode::SigNotUpdated, "SigNotUpdated"},
     {ErrorCode::NotAllPIDValuesUpdated, "NotAllPIDValuesUpdated"},
     {ErrorCode::GEN_PORT_ERROR, "GEN_PORT_ERROR"},
     {ErrorCode::PORT_MODULE_TYPE_MISMATCH, "PORT_MODULE_TYPE_MISMATCH"},
     {ErrorCode::GEN_MODULE_ERROR, "GEN_MODULE_ERROR"},
     {ErrorCode::MODULE_NOT_INIT_SET_ERROR, "MODULE_NOT_INIT_SET_ERROR"},
     {ErrorCode::MODULE_NOT_INIT_GET_ERROR, "MODULE_NOT_INIT_GET_ERROR"},
     {ErrorCode::WheelRadiusTooSmall, "WheelRadiusTooSmall"},
     {ErrorCode::TicksPerRevZero, "TicksPerRevZero"},
     {ErrorCode::DistanceBetweenWheelsTooSmall,
      "DistanceBetweenWheelsTooSmall"},
     {ErrorCode::GainsAreNotSet, "GainsAreNotSet"},
     {ErrorCode::WrongRemoteLimitSwitchSource, "WrongRemoteLimitSwitchSource"},
     {ErrorCode::DoubleVoltageCompensatingWPI, "DoubleVoltageCompensatingWPI"},
     {ErrorCode::IncompatibleMode, "IncompatibleMode"},
     {ErrorCode::InvalidHandle, "InvalidHandle"},
     {ErrorCode::FeatureRequiresHigherFirm, "FeatureRequiresHigherFirm"},
     {ErrorCode::MotorControllerFeatureRequiresHigherFirm,
      "MotorControllerFeatureRequiresHigherFirm"},
     {ErrorCode::TalonFeatureRequiresHigherFirm,
      "TalonFeatureRequiresHigherFirm"},
     {ErrorCode::ConfigFactoryDefaultRequiresHigherFirm,
      "ConfigFactoryDefaultRequiresHigherFirm"},
     {ErrorCode::ConfigMotionSCurveRequiresHigherFirm,
      "ConfigMotionSCurveRequiresHigherFirm"},
     {ErrorCode::TalonFXFirmwarePreVBatDetect, "TalonFXFirmwarePreVBatDetect"},
     {ErrorCode::LibraryCouldNotBeLoaded, "LibraryCouldNotBeLoaded"},
     {ErrorCode::MissingRoutineInLibrary, "MissingRoutineInLibrary"},
     {ErrorCode::ResourceNotAvailable, "ResourceNotAvailable"},
     {ErrorCode::MusicFileNotFound, "MusicFileNotFound"},
     {ErrorCode::MusicFileWrongSize, "MusicFileWrongSize"},
     {ErrorCode::MusicFileTooNew, "MusicFileTooNew"},
     {ErrorCode::MusicFileInvalid, "MusicFileInvalid"},
     {ErrorCode::InvalidOrchestraAction, "InvalidOrchestraAction"},
     {ErrorCode::MusicFileTooOld, "MusicFileTooOld"},
     {ErrorCode::MusicInterrupted, "MusicInterrupted"},
     {ErrorCode::MusicNotSupported, "MusicNotSupported"},
     {ErrorCode::PulseWidthSensorNotPresent, "PulseWidthSensorNotPresent"},
     {ErrorCode::GeneralWarning, "GeneralWarning"},
     {ErrorCode::FeatureNotSupported, "FeatureNotSupported"},
     {ErrorCode::NotImplemented, "NotImplemented"},
     {ErrorCode::FirmVersionCouldNotBeRetrieved,
      "FirmVersionCouldNotBeRetrieved"},
     {ErrorCode::FeaturesNotAvailableYet, "FeaturesNotAvailableYet"},
     {ErrorCode::ControlModeNotValid, "ControlModeNotValid"},
     {ErrorCode::ControlModeNotSupportedYet, "ControlModeNotSupportedYet"},
     {ErrorCode::CascadedPIDNotSupporteYet, "CascadedPIDNotSupporteYet"},
     {ErrorCode::AuxiliaryPIDNotSupportedYet, "AuxiliaryPIDNotSupportedYet"},
     {ErrorCode::RemoteSensorsNotSupportedYet, "RemoteSensorsNotSupportedYet"},
     {ErrorCode::MotProfFirmThreshold, "MotProfFirmThreshold"},
     {ErrorCode::MotProfFirmThreshold2, "MotProfFirmThreshold2"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    CANifier::LEDChannel, {{CANifier::LEDChannel::LEDChannelA, "LEDChannelA"},
                           {CANifier::LEDChannel::LEDChannelB, "LEDChannelB"},
                           {CANifier::LEDChannel::LEDChannelC, "LEDChannelC"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    CANifier::PWMChannel, {{CANifier::PWMChannel::PWMChannel0, "PWMChannel0"},
                           {CANifier::PWMChannel::PWMChannel1, "PWMChannel1"},
                           {CANifier::PWMChannel::PWMChannel2, "PWMChannel2"},
                           {CANifier::PWMChannel::PWMChannel3, "PWMChannel3"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    CANifier::GeneralPin,
    {{CANifier::GeneralPin::QUAD_IDX, "QUAD_IDX"},
     {CANifier::GeneralPin::QUAD_B, "QUAD_B"},
     {CANifier::GeneralPin::QUAD_A, "QUAD_A"},
     {CANifier::GeneralPin::LIMR, "LIMR"},
     {CANifier::GeneralPin::LIMF, "LIMF"},
     {CANifier::GeneralPin::SDA, "SDA"},
     {CANifier::GeneralPin::SCL, "SCL"},
     {CANifier::GeneralPin::SPI_CS, "SPI_CS"},
     {CANifier::GeneralPin::SPI_MISO_PWM2P, "SPI_MISO_PWM2P"},
     {CANifier::GeneralPin::SPI_MOSI_PWM1P, "SPI_MOSI_PWM1P"},
     {CANifier::GeneralPin::SPI_CLK_PWM0P, "SPI_CLK_PWM0P"}})

namespace motorcontrol {
NLOHMANN_JSON_SERIALIZE_ENUM(NeutralMode,
                             {{NeutralMode::EEPROMSetting, "EEPROMSetting"},
                              {NeutralMode::Coast, "Coast"},
                              {NeutralMode::Brake, "Brake"}})

NLOHMANN_JSON_SERIALIZE_ENUM(VelocityMeasPeriod,
                             {{VelocityMeasPeriod::Period_1Ms, "Period_1Ms"},
                              {VelocityMeasPeriod::Period_2Ms, "Period_2Ms"},
                              {VelocityMeasPeriod::Period_5Ms, "Period_5Ms"},
                              {VelocityMeasPeriod::Period_10Ms, "Period_10Ms"},
                              {VelocityMeasPeriod::Period_20Ms, "Period_20Ms"},
                              {VelocityMeasPeriod::Period_25Ms, "Period_25Ms"},
                              {VelocityMeasPeriod::Period_50Ms, "Period_50Ms"},
                              {VelocityMeasPeriod::Period_100Ms,
                               "Period_100Ms"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    RemoteSensorSource,
    {{RemoteSensorSource::RemoteSensorSource_Off, "RemoteSensorSource_Off"},
     {RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,
      "RemoteSensorSource_TalonSRX_SelectedSensor"},
     {RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw,
      "RemoteSensorSource_Pigeon_Yaw"},
     {RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch,
      "RemoteSensorSource_Pigeon_Pitch"},
     {RemoteSensorSource::RemoteSensorSource_Pigeon_Roll,
      "RemoteSensorSource_Pigeon_Roll"},
     {RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature,
      "RemoteSensorSource_CANifier_Quadrature"},
     {RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0,
      "RemoteSensorSource_CANifier_PWMInput0"},
     {RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1,
      "RemoteSensorSource_CANifier_PWMInput1"},
     {RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2,
      "RemoteSensorSource_CANifier_PWMInput2"},
     {RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3,
      "RemoteSensorSource_CANifier_PWMInput3"},
     {RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw,
      "RemoteSensorSource_GadgeteerPigeon_Yaw"},
     {RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch,
      "RemoteSensorSource_GadgeteerPigeon_Pitch"},
     {RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll,
      "RemoteSensorSource_GadgeteerPigeon_Roll"},
     {RemoteSensorSource::RemoteSensorSource_CANCoder,
      "RemoteSensorSource_CANCoder"},
     {RemoteSensorSource::RemoteSensorSource_TalonFX_SelectedSensor,
      "RemoteSensorSource_TalonFX_SelectedSensor"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    StatusFrameEnhanced,
    {{StatusFrameEnhanced::Status_1_General, "Status_1_General"},
     {StatusFrameEnhanced::Status_2_Feedback0, "Status_2_Feedback0"},
     {StatusFrameEnhanced::Status_4_AinTempVbat, "Status_4_AinTempVbat"},
     {StatusFrameEnhanced::Status_6_Misc, "Status_6_Misc"},
     {StatusFrameEnhanced::Status_7_CommStatus, "Status_7_CommStatus"},
     {StatusFrameEnhanced::Status_9_MotProfBuffer, "Status_9_MotProfBuffer"},
     {StatusFrameEnhanced::Status_10_MotionMagic, "Status_10_MotionMagic"},
     {StatusFrameEnhanced::Status_10_Targets, "Status_10_Targets"},
     {StatusFrameEnhanced::Status_12_Feedback1, "Status_12_Feedback1"},
     {StatusFrameEnhanced::Status_13_Base_PIDF0, "Status_13_Base_PIDF0"},
     {StatusFrameEnhanced::Status_14_Turn_PIDF1, "Status_14_Turn_PIDF1"},
     {StatusFrameEnhanced::Status_15_FirmareApiStatus,
      "Status_15_FirmareApiStatus"},
     {StatusFrameEnhanced::Status_17_Targets1, "Status_17_Targets1"},
     {StatusFrameEnhanced::Status_3_Quadrature, "Status_3_Quadrature"},
     {StatusFrameEnhanced::Status_8_PulseWidth, "Status_8_PulseWidth"},
     {StatusFrameEnhanced::Status_11_UartGadgeteer, "Status_11_UartGadgeteer"},
     {StatusFrameEnhanced::Status_Brushless_Current,
      "Status_Brushless_Current"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    StatusFrame,
    {{StatusFrame::Status_1_General_, "Status_1_General_"},
     {StatusFrame::Status_2_Feedback0_, "Status_2_Feedback0_"},
     {StatusFrame::Status_4_AinTempVbat_, "Status_4_AinTempVbat_"},
     {StatusFrame::Status_6_Misc_, "Status_6_Misc_"},
     {StatusFrame::Status_7_CommStatus_, "Status_7_CommStatus_"},
     {StatusFrame::Status_9_MotProfBuffer_, "Status_9_MotProfBuffer_"},
     {StatusFrame::Status_10_MotionMagic_, "Status_10_MotionMagic_"},
     {StatusFrame::Status_10_Targets_, "Status_10_Targets_"},
     {StatusFrame::Status_12_Feedback1_, "Status_12_Feedback1_"},
     {StatusFrame::Status_13_Base_PIDF0_, "Status_13_Base_PIDF0_"},
     {StatusFrame::Status_14_Turn_PIDF1_, "Status_14_Turn_PIDF1_"},
     {StatusFrame::Status_15_FirmareApiStatus_, "Status_15_FirmareApiStatus_"},
     {StatusFrame::Status_17_Targets1_, "Status_17_Targets1_"}})

NLOHMANN_JSON_SERIALIZE_ENUM(ControlMode,
                             {{ControlMode::PercentOutput, "PercentOutput"},
                              {ControlMode::Position, "Position"},
                              {ControlMode::Velocity, "Velocity"},
                              {ControlMode::Current, "Current"},
                              {ControlMode::Follower, "Follower"},
                              {ControlMode::MotionProfile, "MotionProfile"},
                              {ControlMode::MotionMagic, "MotionMagic"},
                              {ControlMode::MotionProfileArc,
                               "MotionProfileArc"},
                              {ControlMode::MusicTone, "MusicTone"},
                              {ControlMode::Disabled, "Disabled"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    TalonFXControlMode,
    {{TalonFXControlMode::PercentOutput, "PercentOutput"},
     {TalonFXControlMode::Position, "Position"},
     {TalonFXControlMode::Velocity, "Velocity"},
     {TalonFXControlMode::Current, "Current"},
     {TalonFXControlMode::Follower, "Follower"},
     {TalonFXControlMode::MotionProfile, "MotionProfile"},
     {TalonFXControlMode::MotionMagic, "MotionMagic"},
     {TalonFXControlMode::MotionProfileArc, "MotionProfileArc"},
     {TalonFXControlMode::MusicTone, "MusicTone"},
     {TalonFXControlMode::Disabled, "Disabled"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    TalonSRXControlMode,
    {{TalonSRXControlMode::PercentOutput, "PercentOutput"},
     {TalonSRXControlMode::Position, "Position"},
     {TalonSRXControlMode::Velocity, "Velocity"},
     {TalonSRXControlMode::Current, "Current"},
     {TalonSRXControlMode::Follower, "Follower"},
     {TalonSRXControlMode::MotionProfile, "MotionProfile"},
     {TalonSRXControlMode::MotionMagic, "MotionMagic"},
     {TalonSRXControlMode::MotionProfileArc, "MotionProfileArc"},
     {TalonSRXControlMode::Disabled, "Disabled"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    VictorSPXControlMode,
    {{VictorSPXControlMode::PercentOutput, "PercentOutput"},
     {VictorSPXControlMode::Position, "Position"},
     {VictorSPXControlMode::Velocity, "Velocity"},
     {VictorSPXControlMode::Follower, "Follower"},
     {VictorSPXControlMode::MotionProfile, "MotionProfile"},
     {VictorSPXControlMode::MotionMagic, "MotionMagic"},
     {VictorSPXControlMode::MotionProfileArc, "MotionProfileArc"},
     {VictorSPXControlMode::Disabled, "Disabled"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    FollowerType,
    {{FollowerType::FollowerType_PercentOutput, "FollowerType_PercentOutput"},
     {FollowerType::FollowerType_AuxOutput1, "FollowerType_AuxOutput1"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    FeedbackDevice,
    {{FeedbackDevice::QuadEncoder, "QuadEncoder"},
     {FeedbackDevice::IntegratedSensor, "IntegratedSensor"},
     {FeedbackDevice::Analog, "Analog"},
     {FeedbackDevice::Tachometer, "Tachometer"},
     {FeedbackDevice::PulseWidthEncodedPosition, "PulseWidthEncodedPosition"},
     {FeedbackDevice::SensorSum, "SensorSum"},
     {FeedbackDevice::SensorDifference, "SensorDifference"},
     {FeedbackDevice::RemoteSensor0, "RemoteSensor0"},
     {FeedbackDevice::RemoteSensor1, "RemoteSensor1"},
     {FeedbackDevice::None, "None"},
     {FeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor"},
     {FeedbackDevice::CTRE_MagEncoder_Absolute, "CTRE_MagEncoder_Absolute"},
     {FeedbackDevice::CTRE_MagEncoder_Relative, "CTRE_MagEncoder_Relative"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    TalonSRXFeedbackDevice,
    {{TalonSRXFeedbackDevice::QuadEncoder, "QuadEncoder"},
     {TalonSRXFeedbackDevice::Analog, "Analog"},
     {TalonSRXFeedbackDevice::Tachometer, "Tachometer"},
     {TalonSRXFeedbackDevice::PulseWidthEncodedPosition,
      "PulseWidthEncodedPosition"},
     {TalonSRXFeedbackDevice::SensorSum, "SensorSum"},
     {TalonSRXFeedbackDevice::SensorDifference, "SensorDifference"},
     {TalonSRXFeedbackDevice::RemoteSensor0, "RemoteSensor0"},
     {TalonSRXFeedbackDevice::RemoteSensor1, "RemoteSensor1"},
     {TalonSRXFeedbackDevice::None, "None"},
     {TalonSRXFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor"},
     {TalonSRXFeedbackDevice::CTRE_MagEncoder_Absolute,
      "CTRE_MagEncoder_Absolute"},
     {TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative,
      "CTRE_MagEncoder_Relative"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    TalonFXFeedbackDevice,
    {{TalonFXFeedbackDevice::IntegratedSensor, "IntegratedSensor"},
     {TalonFXFeedbackDevice::SensorSum, "SensorSum"},
     {TalonFXFeedbackDevice::SensorDifference, "SensorDifference"},
     {TalonFXFeedbackDevice::RemoteSensor0, "RemoteSensor0"},
     {TalonFXFeedbackDevice::RemoteSensor1, "RemoteSensor1"},
     {TalonFXFeedbackDevice::None, "None"},
     {TalonFXFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    RemoteFeedbackDevice,
    {{RemoteFeedbackDevice::RemoteFeedbackDevice_FactoryDefaultOff,
      "RemoteFeedbackDevice_FactoryDefaultOff"},
     {RemoteFeedbackDevice::FactoryDefaultOff, "FactoryDefaultOff"},
     {RemoteFeedbackDevice::RemoteFeedbackDevice_SensorSum,
      "RemoteFeedbackDevice_SensorSum"},
     {RemoteFeedbackDevice::SensorSum, "SensorSum"},
     {RemoteFeedbackDevice::RemoteFeedbackDevice_SensorDifference,
      "RemoteFeedbackDevice_SensorDifference"},
     {RemoteFeedbackDevice::SensorDifference, "SensorDifference"},
     {RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0,
      "RemoteFeedbackDevice_RemoteSensor0"},
     {RemoteFeedbackDevice::RemoteSensor0, "RemoteSensor0"},
     {RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor1,
      "RemoteFeedbackDevice_RemoteSensor1"},
     {RemoteFeedbackDevice::RemoteSensor1, "RemoteSensor1"},
     {RemoteFeedbackDevice::RemoteFeedbackDevice_None,
      "RemoteFeedbackDevice_None"},
     {RemoteFeedbackDevice::None, "None"},
     {RemoteFeedbackDevice::RemoteFeedbackDevice_SoftwareEmulatedSensor,
      "RemoteFeedbackDevice_SoftwareEmulatedSensor"},
     {RemoteFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    SensorTerm, {{SensorTerm::SensorTerm_Sum0, "SensorTerm_Sum0"},
                 {SensorTerm::SensorTerm_Sum1, "SensorTerm_Sum1"},
                 {SensorTerm::SensorTerm_Diff0, "SensorTerm_Diff0"},
                 {SensorTerm::SensorTerm_Diff1, "SensorTerm_Diff1"}})

NLOHMANN_JSON_SERIALIZE_ENUM(InvertType,
                             {{InvertType::None, "None"},
                              {InvertType::InvertMotorOutput,
                               "InvertMotorOutput"},
                              {InvertType::FollowMaster, "FollowMaster"},
                              {InvertType::OpposeMaster, "OpposeMaster"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    TalonFXInvertType,
    {{TalonFXInvertType::CounterClockwise, "CounterClockwise"},
     {TalonFXInvertType::Clockwise, "Clockwise"},
     {TalonFXInvertType::FollowMaster, "FollowMaster"},
     {TalonFXInvertType::OpposeMaster, "OpposeMaster"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    DemandType, {{DemandType::DemandType_Neutral, "DemandType_Neutral"},
                 {DemandType::DemandType_AuxPID, "DemandType_AuxPID"},
                 {DemandType::DemandType_ArbitraryFeedForward,
                  "DemandType_ArbitraryFeedForward"}})

NLOHMANN_JSON_SERIALIZE_ENUM(MotorCommutation,
                             {{MotorCommutation::Trapezoidal, "Trapezoidal"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    LimitSwitchSource, {{LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                         "LimitSwitchSource_FeedbackConnector"},
                        {LimitSwitchSource::LimitSwitchSource_RemoteTalonSRX,
                         "LimitSwitchSource_RemoteTalonSRX"},
                        {LimitSwitchSource::LimitSwitchSource_RemoteCANifier,
                         "LimitSwitchSource_RemoteCANifier"},
                        {LimitSwitchSource::LimitSwitchSource_Deactivated,
                         "LimitSwitchSource_Deactivated"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    RemoteLimitSwitchSource,
    {{RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
      "RemoteLimitSwitchSource_RemoteTalonSRX"},
     {RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteCANifier,
      "RemoteLimitSwitchSource_RemoteCANifier"},
     {RemoteLimitSwitchSource::RemoteLimitSwitchSource_Deactivated,
      "RemoteLimitSwitchSource_Deactivated"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    LimitSwitchNormal, {{LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
                         "LimitSwitchNormal_NormallyOpen"},
                        {LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
                         "LimitSwitchNormal_NormallyClosed"},
                        {LimitSwitchNormal::LimitSwitchNormal_Disabled,
                         "LimitSwitchNormal_Disabled"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    ControlFrame, {{ControlFrame::Control_3_General, "Control_3_General"},
                   {ControlFrame::Control_4_Advanced, "Control_4_Advanced"},
                   {ControlFrame::Control_6_MotProfAddTrajPoint,
                    "Control_6_MotProfAddTrajPoint"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    ControlFrameEnhanced,
    {{ControlFrameEnhanced::Control_3_General_, "Control_3_General"},
     {ControlFrameEnhanced::Control_4_Advanced_, "Control_4_Advanced"},
     {ControlFrameEnhanced::Control_5_FeedbackOutputOverride_,
      "Control_5_FeedbackOutputOverride_"},
     {ControlFrameEnhanced::Control_6_MotProfAddTrajPoint_,
      "Control_6_MotProfAddTrajPoint"}})

} // namespace motorcontrol
namespace motion {
NLOHMANN_JSON_SERIALIZE_ENUM(SetValueMotionProfile,
                             {{SetValueMotionProfile::Disable, "Disable"},
                              {SetValueMotionProfile::Enable, "Enable"},
                              {SetValueMotionProfile::Hold, "Hold"}})

}
namespace sensors {
NLOHMANN_JSON_SERIALIZE_ENUM(
    CANCoderStatusFrame,
    {{CANCoderStatusFrame::CANCoderStatusFrame_SensorData,
      "CANCoderStatusFrame_SensorData"},
     {CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults,
      "CANCoderStatusFrame_VbatAndFaults"}})
NLOHMANN_JSON_SERIALIZE_ENUM(SensorTimeBase,
                             {{SensorTimeBase::Per100Ms_Legacy,
                               "Per100Ms_Legacy"},
                              {SensorTimeBase::PerSecond, "PerSecond"},
                              {SensorTimeBase::PerMinute, "PerMinute"}})
NLOHMANN_JSON_SERIALIZE_ENUM(
    PigeonIMU_ControlFrame,
    {{PigeonIMU_ControlFrame::PigeonIMU_CondStatus_Control_1,
      "PigeonIMU_CondStatus_Control_1"}})
NLOHMANN_JSON_SERIALIZE_ENUM(
    MagnetFieldStrength,
    {{MagnetFieldStrength::Invalid_Unknown, "Invalid_Unknown"},
     {MagnetFieldStrength::BadRange_RedLED, "BadRange_RedLED"},
     {MagnetFieldStrength::Adequate_OrangeLED, "Adequate_OrangeLED"},
     {MagnetFieldStrength::Good_GreenLED, "Good_GreenLED"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    AbsoluteSensorRange,
    {{AbsoluteSensorRange::Unsigned_0_to_360, "Unsigned_0_to_360"},
     {AbsoluteSensorRange::Signed_PlusMinus180, "Signed_PlusMinus180"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    PigeonIMU_StatusFrame,
    {{PigeonIMU_StatusFrame::PigeonIMU_CondStatus_1_General,
      "PigeonIMU_CondStatus_1_General"},
     {PigeonIMU_StatusFrame::PigeonIMU_CondStatus_9_SixDeg_YPR,
      "PigeonIMU_CondStatus_9_SixDeg_YPR"},
     {PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion,
      "PigeonIMU_CondStatus_6_SensorFusion"},
     {PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum,
      "PigeonIMU_CondStatus_11_GyroAccum"},
     {PigeonIMU_StatusFrame::PigeonIMU_CondStatus_2_GeneralCompass,
      "PigeonIMU_CondStatus_2_GeneralCompass"},
     {PigeonIMU_StatusFrame::PigeonIMU_CondStatus_3_GeneralAccel,
      "PigeonIMU_CondStatus_3_GeneralAccel"},
     {PigeonIMU_StatusFrame::PigeonIMU_CondStatus_10_SixDeg_Quat,
      "PigeonIMU_CondStatus_10_SixDeg_Quat"},
     {PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag,
      "PigeonIMU_RawStatus_4_Mag"},
     {PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_2_Gyro,
      "PigeonIMU_BiasedStatus_2_Gyro"},
     {PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_4_Mag,
      "PigeonIMU_BiasedStatus_4_Mag"},
     {PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_6_Accel,
      "PigeonIMU_BiasedStatus_6_Accel"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    SensorVelocityMeasPeriod,
    {{SensorVelocityMeasPeriod::Period_1Ms, "Period_1Ms"},
     {SensorVelocityMeasPeriod::Period_2Ms, "Period_2Ms"},
     {SensorVelocityMeasPeriod::Period_5Ms, "Period_5Ms"},
     {SensorVelocityMeasPeriod::Period_10Ms, "Period_10Ms"},
     {SensorVelocityMeasPeriod::Period_20Ms, "Period_20Ms"},
     {SensorVelocityMeasPeriod::Period_25Ms, "Period_25Ms"},
     {SensorVelocityMeasPeriod::Period_50Ms, "Period_50Ms"},
     {SensorVelocityMeasPeriod::Period_100Ms, "Period_100Ms"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    SensorInitializationStrategy,
    {{SensorInitializationStrategy::BootToZero, "BootToZero"},
     {SensorInitializationStrategy::BootToAbsolutePosition,
      "BootToAbsolutePosition"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    PigeonIMU::CalibrationMode,
    {{PigeonIMU::CalibrationMode::BootTareGyroAccel, "BootTareGyroAccel"},
     {PigeonIMU::CalibrationMode::Temperature, "Temperature"},
     {PigeonIMU::CalibrationMode::Magnetometer12Pt, "Magnetometer12Pt"},
     {PigeonIMU::CalibrationMode::Magnetometer360, "Magnetometer360"},
     {PigeonIMU::CalibrationMode::Accelerometer, "Accelerometer"}})

NLOHMANN_JSON_SERIALIZE_ENUM(
    PigeonIMU::PigeonState,
    {{PigeonIMU::PigeonState::NoComm, "NoComm"},
     {PigeonIMU::PigeonState::Initializing, "Initializing"},
     {PigeonIMU::PigeonState::Ready, "Ready"},
     {PigeonIMU::PigeonState::UserCalibration, "UserCalibration"}})
} // namespace sensors
} // namespace phoenix
} // namespace ctre

// structs
namespace ctre {
namespace phoenix {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CustomParamConfiguration, customParam0,
                                   customParam1, enableOptimizations)

// NO FIELDS YET NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANifierFaults)
// NO FIELDS YET NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANifierStickyFaults)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANifierConfiguration, customParam0,
                                   customParam1, enableOptimizations,
                                   velocityMeasurementPeriod,
                                   velocityMeasurementWindow,
                                   clearPositionOnLimitF, clearPositionOnLimitR,
                                   clearPositionOnQuadIdx)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANifier::PinValues, QUAD_IDX, QUAD_B,
                                   QUAD_A, LIMR, LIMF, SDA, SCL, SPI_CS_PWM3,
                                   SPI_MISO_PWM2, SPI_MOSI_PWM1, SPI_CLK_PWM0)

namespace motion {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPoint, position, velocity,
                                   arbFeedFwd, auxiliaryPos, auxiliaryVel,
                                   auxiliaryArbFeedFwd, profileSlotSelect0,
                                   profileSlotSelect1, isLastPoint, zeroPos,
                                   timeDur, useAuxPID)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MotionProfileStatus, topBufferRem,
                                   topBufferCnt, btmBufferCnt, hasUnderrun,
                                   isUnderrun, activePointValid, isLast,
                                   profileSlotSelect0, profileSlotSelect1,
                                   outputEnable, timeDurMs)
} // namespace motion
namespace motorcontrol {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Faults, UnderVoltage, ForwardLimitSwitch,
                                   ReverseLimitSwitch, ForwardSoftLimit,
                                   ReverseSoftLimit, HardwareFailure,
                                   ResetDuringEn, SensorOverflow,
                                   SensorOutOfPhase, HardwareESDReset,
                                   RemoteLossOfSignal, APIError, SupplyOverV,
                                   SupplyUnstable)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StickyFaults, UnderVoltage,
                                   ForwardLimitSwitch, ReverseLimitSwitch,
                                   ForwardSoftLimit, ReverseSoftLimit,
                                   ResetDuringEn, SensorOverflow,
                                   SensorOutOfPhase, HardwareESDReset,
                                   RemoteLossOfSignal, APIError, SupplyOverV,
                                   SupplyUnstable)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StatorCurrentLimitConfiguration, enable,
                                   currentLimit, triggerThresholdCurrent,
                                   triggerThresholdTime)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SupplyCurrentLimitConfiguration, enable,
                                   currentLimit, triggerThresholdCurrent,
                                   triggerThresholdTime)

namespace can {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BasePIDSetConfiguration,
                                   selectedFeedbackCoefficient)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FilterConfiguration, remoteSensorDeviceID,
                                   remoteSensorSource)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SlotConfiguration, kP, kI, kD, kF,
                                   integralZone, allowableClosedloopError,
                                   maxIntegralAccumulator, closedLoopPeakOutput,
                                   closedLoopPeriod)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    BaseMotorControllerConfiguration, customParam0, customParam1,
    enableOptimizations, openloopRamp, closedloopRamp, peakOutputForward,
    peakOutputReverse, nominalOutputForward, nominalOutputReverse,
    neutralDeadband, voltageCompSaturation, voltageMeasurementFilter,
    velocityMeasurementPeriod, velocityMeasurementWindow,
    forwardSoftLimitThreshold, reverseSoftLimitThreshold,
    forwardSoftLimitEnable, reverseSoftLimitEnable, slot0, slot1, slot2, slot3,
    auxPIDPolarity, remoteFilter0, remoteFilter1, motionCruiseVelocity,
    motionAcceleration, motionCurveStrength, motionProfileTrajectoryPeriod,
    feedbackNotContinuous, remoteSensorClosedLoopDisableNeutralOnLOS,
    clearPositionOnLimitF, clearPositionOnLimitR, clearPositionOnQuadIdx,
    limitSwitchDisableNeutralOnLOS, softLimitDisableNeutralOnLOS,
    pulseWidthPeriod_EdgesPerRot, pulseWidthPeriod_FilterWindowSz,
    trajectoryInterpolationEnable)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BaseTalonPIDSetConfiguration,
                                   selectedFeedbackCoefficient,
                                   selectedFeedbackSensor)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    BaseTalonConfiguration, customParam0, customParam1, enableOptimizations,
    openloopRamp, closedloopRamp, peakOutputForward, peakOutputReverse,
    nominalOutputForward, nominalOutputReverse, neutralDeadband,
    voltageCompSaturation, voltageMeasurementFilter, velocityMeasurementPeriod,
    velocityMeasurementWindow, forwardSoftLimitThreshold,
    reverseSoftLimitThreshold, forwardSoftLimitEnable, reverseSoftLimitEnable,
    slot0, slot1, slot2, slot3, auxPIDPolarity, remoteFilter0, remoteFilter1,
    motionCruiseVelocity, motionAcceleration, motionCurveStrength,
    motionProfileTrajectoryPeriod, feedbackNotContinuous,
    remoteSensorClosedLoopDisableNeutralOnLOS, clearPositionOnLimitF,
    clearPositionOnLimitR, clearPositionOnQuadIdx,
    limitSwitchDisableNeutralOnLOS, softLimitDisableNeutralOnLOS,
    pulseWidthPeriod_EdgesPerRot, pulseWidthPeriod_FilterWindowSz,
    trajectoryInterpolationEnable, primaryPID, auxiliaryPID,
    forwardLimitSwitchSource, reverseLimitSwitchSource,
    forwardLimitSwitchDeviceID, reverseLimitSwitchDeviceID,
    forwardLimitSwitchNormal, reverseLimitSwitchNormal, sum0Term, sum1Term,
    diff0Term, diff1Term)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TalonFXPIDSetConfiguration,
                                   selectedFeedbackCoefficient,
                                   selectedFeedbackSensor)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TalonFXConfiguration, customParam0, customParam1, enableOptimizations,
    openloopRamp, closedloopRamp, peakOutputForward, peakOutputReverse,
    nominalOutputForward, nominalOutputReverse, neutralDeadband,
    voltageCompSaturation, voltageMeasurementFilter, velocityMeasurementPeriod,
    velocityMeasurementWindow, forwardSoftLimitThreshold,
    reverseSoftLimitThreshold, forwardSoftLimitEnable, reverseSoftLimitEnable,
    slot0, slot1, slot2, slot3, auxPIDPolarity, remoteFilter0, remoteFilter1,
    motionCruiseVelocity, motionAcceleration, motionCurveStrength,
    motionProfileTrajectoryPeriod, feedbackNotContinuous,
    remoteSensorClosedLoopDisableNeutralOnLOS, clearPositionOnLimitF,
    clearPositionOnLimitR, clearPositionOnQuadIdx,
    limitSwitchDisableNeutralOnLOS, softLimitDisableNeutralOnLOS,
    pulseWidthPeriod_EdgesPerRot, pulseWidthPeriod_FilterWindowSz,
    trajectoryInterpolationEnable, primaryPID, auxiliaryPID,
    forwardLimitSwitchSource, reverseLimitSwitchSource,
    forwardLimitSwitchDeviceID, reverseLimitSwitchDeviceID,
    forwardLimitSwitchNormal, reverseLimitSwitchNormal, sum0Term, sum1Term,
    diff0Term, diff1Term, supplyCurrLimit, statorCurrLimit, motorCommutation,
    absoluteSensorRange, integratedSensorOffsetDegrees, initializationStrategy)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TalonSRXPIDSetConfiguration,
                                   selectedFeedbackCoefficient,
                                   selectedFeedbackSensor)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TalonSRXConfiguration, customParam0, customParam1, enableOptimizations,
    openloopRamp, closedloopRamp, peakOutputForward, peakOutputReverse,
    nominalOutputForward, nominalOutputReverse, neutralDeadband,
    voltageCompSaturation, voltageMeasurementFilter, velocityMeasurementPeriod,
    velocityMeasurementWindow, forwardSoftLimitThreshold,
    reverseSoftLimitThreshold, forwardSoftLimitEnable, reverseSoftLimitEnable,
    slot0, slot1, slot2, slot3, auxPIDPolarity, remoteFilter0, remoteFilter1,
    motionCruiseVelocity, motionAcceleration, motionCurveStrength,
    motionProfileTrajectoryPeriod, feedbackNotContinuous,
    remoteSensorClosedLoopDisableNeutralOnLOS, clearPositionOnLimitF,
    clearPositionOnLimitR, clearPositionOnQuadIdx,
    limitSwitchDisableNeutralOnLOS, softLimitDisableNeutralOnLOS,
    pulseWidthPeriod_EdgesPerRot, pulseWidthPeriod_FilterWindowSz,
    trajectoryInterpolationEnable, primaryPID, auxiliaryPID,
    forwardLimitSwitchSource, reverseLimitSwitchSource,
    forwardLimitSwitchDeviceID, reverseLimitSwitchDeviceID,
    forwardLimitSwitchNormal, reverseLimitSwitchNormal, sum0Term, sum1Term,
    diff0Term, diff1Term, peakCurrentLimit, peakCurrentDuration)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VictorSPXPIDSetConfiguration,
                                   selectedFeedbackCoefficient,
                                   selectedFeedbackSensor)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    VictorSPXConfiguration, customParam0, customParam1, enableOptimizations,
    openloopRamp, closedloopRamp, peakOutputForward, peakOutputReverse,
    nominalOutputForward, nominalOutputReverse, neutralDeadband,
    voltageCompSaturation, voltageMeasurementFilter, velocityMeasurementPeriod,
    velocityMeasurementWindow, forwardSoftLimitThreshold,
    reverseSoftLimitThreshold, forwardSoftLimitEnable, reverseSoftLimitEnable,
    slot0, slot1, slot2, slot3, auxPIDPolarity, remoteFilter0, remoteFilter1,
    motionCruiseVelocity, motionAcceleration, motionCurveStrength,
    motionProfileTrajectoryPeriod, feedbackNotContinuous,
    remoteSensorClosedLoopDisableNeutralOnLOS, clearPositionOnLimitF,
    clearPositionOnLimitR, clearPositionOnQuadIdx,
    limitSwitchDisableNeutralOnLOS, softLimitDisableNeutralOnLOS,
    pulseWidthPeriod_EdgesPerRot, pulseWidthPeriod_FilterWindowSz,
    trajectoryInterpolationEnable, primaryPID, auxiliaryPID,
    forwardLimitSwitchSource, reverseLimitSwitchSource,
    forwardLimitSwitchDeviceID, reverseLimitSwitchDeviceID,
    forwardLimitSwitchNormal, reverseLimitSwitchNormal, sum0Term, sum1Term,
    diff0Term, diff1Term)
} // namespace can
} // namespace motorcontrol
namespace sensors {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANCoderFaults, HardwareFault, APIError,
                                   UnderVoltage, ResetDuringEn, MagnetTooWeak)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANCoderStickyFaults, HardwareFault,
                                   APIError, UnderVoltage, ResetDuringEn,
                                   MagnetTooWeak)

// NO FIELDS YET NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PigeonIMU_Faults)
// NO FIELDS YET NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PigeonIMU_StickyFaults)
// NO FIELDS YET NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CANifierFaults)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PigeonIMUConfiguration, customParam0,
                                   customParam1, enableOptimizations)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PigeonIMU::FusionStatus, heading, bIsValid,
                                   bIsFusing, description, lastError)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PigeonIMU::GeneralStatus, state, currentMode,
                                   calibrationError, bCalIsBooting, description,
                                   tempC, upTimeSec, noMotionBiasCount,
                                   tempCompensationCount, lastError)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    CANCoderConfiguration, customParam0, customParam1, enableOptimizations,
    velocityMeasurementPeriod, velocityMeasurementWindow, absoluteSensorRange,
    magnetOffsetDegrees, sensorDirection, initializationStrategy,
    sensorCoefficient, unitString, sensorTimeBase)
} // namespace sensors
} // namespace phoenix
} // namespace ctre
