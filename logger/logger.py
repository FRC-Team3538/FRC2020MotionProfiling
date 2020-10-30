from rj.StatusFrameCollection import *
import socket
import json
import time

UDP_IP_ADDRESS = '0.0.0.0'
UDP_PORT_NO = 5801

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))

def deserializeCTREMotorStatusFrame(sf):
    return {
        'firmwareVersion': sf.FirmwareVersion(),
        'baseID': sf.BaseID(),
        'deviceID': sf.DeviceID(),
        'outputCurrent': sf.OutputCurrent(),
        'busVoltage': sf.BusVoltage(),
        'outputPercent': sf.OutputPercent(),
        'outputVoltage': sf.OutputVoltage(),
        'temperature': sf.Temperature(),
        'selectedSensorPosition': sf.SelectedSensorPosition(),
        'selectedSensorVelocity': sf.SelectedSensorVelocity(),
        'closedLoopError': sf.ClosedLoopError(),
        'integralAccumulator': sf.IntegralAccumulator(),
        'errorDerivative': sf.ErrorDerivative(),
        'closedLoopTarget': sf.ClosedLoopTarget(),
        'activeTrajectoryPosition': sf.ActiveTrajectoryPosition(),
        'activeTrajectoryVelocity': sf.ActiveTrajectoryVelocity(),
        'activeTrajectoryArbFeedFwd': sf.ActiveTrajectoryArbFeedFwd(),
        'faults': sf.Faults(),
        'resetOccured': sf.ResetOccured(),
        'lastError': sf.LastError(),
        'controlMode': sf.ControlMode(),
        'statorCurrent': sf.StatorCurrent(),
        'supplyCurrent': sf.SupplyCurrent(),
        'fwdLimitSwitchClosed': sf.FwdLimitSwitchClosed(),
        'revLimitSwitchClosed': sf.RevLimitSwitchClosed(),
    }

def deserializePDPStatusFrame(sf):
    return {
        'module': sf.Module(),
        'voltage': sf.Voltage(),
        'temperature': sf.Temperature(),
        'channel0Current': sf.Channel0Current(),
        'channel1Current': sf.Channel1Current(),
        'channel2Current': sf.Channel2Current(),
        'channel3Current': sf.Channel3Current(),
        'channel4Current': sf.Channel4Current(),
        'channel5Current': sf.Channel5Current(),
        'channel6Current': sf.Channel6Current(),
        'channel7Current': sf.Channel7Current(),
        'channel8Current': sf.Channel8Current(),
        'channel9Current': sf.Channel9Current(),
        'channel10Current': sf.Channel10Current(),
        'channel11Current': sf.Channel11Current(),
        'channel12Current': sf.Channel12Current(),
        'channel13Current': sf.Channel13Current(),
        'channel14Current': sf.Channel14Current(),
        'channel15Current': sf.Channel15Current(),
        'totalCurrent': sf.TotalCurrent(),
        'totalPower': sf.TotalPower(),
        'totalEnergy': sf.TotalEnergy(),
    }

def deserializePCMStatusFrame(sf):
    return {
        'module': sf.Module(),
        'enabled': sf.Enabled(),
        'pressureSwitchValve': sf.PressureSwitchValve(),
        'compressorCurrent': sf.CompressorCurrent(),
        'closedLoopControl': sf.ClosedLoopControl(),
        'compressorCurrentTooHighFault': sf.CompressorCurrentTooHighFault(),
        'compressorShortedFault': sf.CompressorShortedFault(),
        'compressorNotConnectedFault': sf.CompressorNotConnectedFault(),
    }

def deserializeStatusFrameCollection(sfc):
    return {
        'driveLeft1': deserializeCTREMotorStatusFrame(sfc.DriveLeft1()),
        'driveLeft2': deserializeCTREMotorStatusFrame(sfc.DriveLeft2()),
        'driveRight1': deserializeCTREMotorStatusFrame(sfc.DriveRight1()),
        'driveRight2': deserializeCTREMotorStatusFrame(sfc.DriveRight2()),
        'powerDistributionPanel': deserializePDPStatusFrame(sfc.PowerDistributionPanel()),
        'pneumaticsControlModule': deserializePCMStatusFrame(sfc.PneumaticsControlModule())
    }

while True:
    data, addr = sock.recvfrom(1024, socket.MSG_DONTWAIT)
    buf = bytearray(data)
    sfc = StatusFrameCollection.GetRootAsStatusFrameCollection(buf, 0)
    print(f"Received {len(buf)} bytes from {addr} at {time.time()}")

