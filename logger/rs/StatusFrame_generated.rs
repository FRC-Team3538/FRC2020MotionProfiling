// automatically generated by the FlatBuffers compiler, do not modify



use std::mem;
use std::cmp::Ordering;

extern crate flatbuffers;
use self::flatbuffers::EndianScalar;

#[allow(unused_imports, dead_code)]
pub mod rj {

  use std::mem;
  use std::cmp::Ordering;

  extern crate flatbuffers;
  use self::flatbuffers::EndianScalar;

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub enum StatusFrame {
  NONE = 0,
  CTREMotorStatusFrame = 1,
  PDPStatusFrame = 2,
  PCMStatusFrame = 3,

}

pub const ENUM_MIN_STATUS_FRAME: u8 = 0;
pub const ENUM_MAX_STATUS_FRAME: u8 = 3;

impl<'a> flatbuffers::Follow<'a> for StatusFrame {
  type Inner = Self;
  #[inline]
  fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
    flatbuffers::read_scalar_at::<Self>(buf, loc)
  }
}

impl flatbuffers::EndianScalar for StatusFrame {
  #[inline]
  fn to_little_endian(self) -> Self {
    let n = u8::to_le(self as u8);
    let p = &n as *const u8 as *const StatusFrame;
    unsafe { *p }
  }
  #[inline]
  fn from_little_endian(self) -> Self {
    let n = u8::from_le(self as u8);
    let p = &n as *const u8 as *const StatusFrame;
    unsafe { *p }
  }
}

impl flatbuffers::Push for StatusFrame {
    type Output = StatusFrame;
    #[inline]
    fn push(&self, dst: &mut [u8], _rest: &[u8]) {
        flatbuffers::emplace_scalar::<StatusFrame>(dst, *self);
    }
}

#[allow(non_camel_case_types)]
pub const ENUM_VALUES_STATUS_FRAME:[StatusFrame; 4] = [
  StatusFrame::NONE,
  StatusFrame::CTREMotorStatusFrame,
  StatusFrame::PDPStatusFrame,
  StatusFrame::PCMStatusFrame
];

#[allow(non_camel_case_types)]
pub const ENUM_NAMES_STATUS_FRAME:[&'static str; 4] = [
    "NONE",
    "CTREMotorStatusFrame",
    "PDPStatusFrame",
    "PCMStatusFrame"
];

pub fn enum_name_status_frame(e: StatusFrame) -> &'static str {
  let index = e as u8;
  ENUM_NAMES_STATUS_FRAME[index as usize]
}

pub struct StatusFrameUnionTableOffset {}
pub enum CTREMotorStatusFrameOffset {}
#[derive(Copy, Clone, Debug, PartialEq)]

pub struct CTREMotorStatusFrame<'a> {
  pub _tab: flatbuffers::Table<'a>,
}

impl<'a> flatbuffers::Follow<'a> for CTREMotorStatusFrame<'a> {
    type Inner = CTREMotorStatusFrame<'a>;
    #[inline]
    fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
        Self {
            _tab: flatbuffers::Table { buf: buf, loc: loc },
        }
    }
}

impl<'a> CTREMotorStatusFrame<'a> {
    #[inline]
    pub fn init_from_table(table: flatbuffers::Table<'a>) -> Self {
        CTREMotorStatusFrame {
            _tab: table,
        }
    }
    #[allow(unused_mut)]
    pub fn create<'bldr: 'args, 'args: 'mut_bldr, 'mut_bldr>(
        _fbb: &'mut_bldr mut flatbuffers::FlatBufferBuilder<'bldr>,
        args: &'args CTREMotorStatusFrameArgs) -> flatbuffers::WIPOffset<CTREMotorStatusFrame<'bldr>> {
      let mut builder = CTREMotorStatusFrameBuilder::new(_fbb);
      builder.add_supplyCurrent(args.supplyCurrent);
      builder.add_statorCurrent(args.statorCurrent);
      builder.add_activeTrajectoryArbFeedFwd(args.activeTrajectoryArbFeedFwd);
      builder.add_closedLoopTarget(args.closedLoopTarget);
      builder.add_errorDerivative(args.errorDerivative);
      builder.add_integralAccumulator(args.integralAccumulator);
      builder.add_temperature(args.temperature);
      builder.add_outputVoltage(args.outputVoltage);
      builder.add_outputPercent(args.outputPercent);
      builder.add_busVoltage(args.busVoltage);
      builder.add_outputCurrent(args.outputCurrent);
      builder.add_revLimitSwitchClosed(args.revLimitSwitchClosed);
      builder.add_fwdLimitSwitchClosed(args.fwdLimitSwitchClosed);
      builder.add_controlMode(args.controlMode);
      builder.add_lastError(args.lastError);
      builder.add_faults(args.faults);
      builder.add_activeTrajectoryVelocity(args.activeTrajectoryVelocity);
      builder.add_activeTrajectoryPosition(args.activeTrajectoryPosition);
      builder.add_closedLoopError(args.closedLoopError);
      builder.add_selectedSensorVelocity(args.selectedSensorVelocity);
      builder.add_selectedSensorPosition(args.selectedSensorPosition);
      builder.add_deviceID(args.deviceID);
      builder.add_baseID(args.baseID);
      builder.add_firmwareVersion(args.firmwareVersion);
      builder.add_resetOccured(args.resetOccured);
      builder.finish()
    }

    pub const VT_FIRMWAREVERSION: flatbuffers::VOffsetT = 4;
    pub const VT_BASEID: flatbuffers::VOffsetT = 6;
    pub const VT_DEVICEID: flatbuffers::VOffsetT = 8;
    pub const VT_OUTPUTCURRENT: flatbuffers::VOffsetT = 10;
    pub const VT_BUSVOLTAGE: flatbuffers::VOffsetT = 12;
    pub const VT_OUTPUTPERCENT: flatbuffers::VOffsetT = 14;
    pub const VT_OUTPUTVOLTAGE: flatbuffers::VOffsetT = 16;
    pub const VT_TEMPERATURE: flatbuffers::VOffsetT = 18;
    pub const VT_SELECTEDSENSORPOSITION: flatbuffers::VOffsetT = 20;
    pub const VT_SELECTEDSENSORVELOCITY: flatbuffers::VOffsetT = 22;
    pub const VT_CLOSEDLOOPERROR: flatbuffers::VOffsetT = 24;
    pub const VT_INTEGRALACCUMULATOR: flatbuffers::VOffsetT = 26;
    pub const VT_ERRORDERIVATIVE: flatbuffers::VOffsetT = 28;
    pub const VT_CLOSEDLOOPTARGET: flatbuffers::VOffsetT = 30;
    pub const VT_ACTIVETRAJECTORYPOSITION: flatbuffers::VOffsetT = 32;
    pub const VT_ACTIVETRAJECTORYVELOCITY: flatbuffers::VOffsetT = 34;
    pub const VT_ACTIVETRAJECTORYARBFEEDFWD: flatbuffers::VOffsetT = 36;
    pub const VT_FAULTS: flatbuffers::VOffsetT = 38;
    pub const VT_RESETOCCURED: flatbuffers::VOffsetT = 40;
    pub const VT_LASTERROR: flatbuffers::VOffsetT = 42;
    pub const VT_CONTROLMODE: flatbuffers::VOffsetT = 44;
    pub const VT_STATORCURRENT: flatbuffers::VOffsetT = 46;
    pub const VT_SUPPLYCURRENT: flatbuffers::VOffsetT = 48;
    pub const VT_FWDLIMITSWITCHCLOSED: flatbuffers::VOffsetT = 50;
    pub const VT_REVLIMITSWITCHCLOSED: flatbuffers::VOffsetT = 52;

  #[inline]
  pub fn firmwareVersion(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_FIRMWAREVERSION, Some(0)).unwrap()
  }
  #[inline]
  pub fn baseID(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_BASEID, Some(0)).unwrap()
  }
  #[inline]
  pub fn deviceID(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_DEVICEID, Some(0)).unwrap()
  }
  #[inline]
  pub fn outputCurrent(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_OUTPUTCURRENT, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn busVoltage(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_BUSVOLTAGE, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn outputPercent(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_OUTPUTPERCENT, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn outputVoltage(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_OUTPUTVOLTAGE, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn temperature(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_TEMPERATURE, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn selectedSensorPosition(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_SELECTEDSENSORPOSITION, Some(0)).unwrap()
  }
  #[inline]
  pub fn selectedSensorVelocity(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_SELECTEDSENSORVELOCITY, Some(0)).unwrap()
  }
  #[inline]
  pub fn closedLoopError(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_CLOSEDLOOPERROR, Some(0)).unwrap()
  }
  #[inline]
  pub fn integralAccumulator(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_INTEGRALACCUMULATOR, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn errorDerivative(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_ERRORDERIVATIVE, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn closedLoopTarget(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_CLOSEDLOOPTARGET, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn activeTrajectoryPosition(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_ACTIVETRAJECTORYPOSITION, Some(0)).unwrap()
  }
  #[inline]
  pub fn activeTrajectoryVelocity(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_ACTIVETRAJECTORYVELOCITY, Some(0)).unwrap()
  }
  #[inline]
  pub fn activeTrajectoryArbFeedFwd(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_ACTIVETRAJECTORYARBFEEDFWD, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn faults(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_FAULTS, Some(0)).unwrap()
  }
  #[inline]
  pub fn resetOccured(&self) -> bool {
    self._tab.get::<bool>(CTREMotorStatusFrame::VT_RESETOCCURED, Some(false)).unwrap()
  }
  #[inline]
  pub fn lastError(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_LASTERROR, Some(0)).unwrap()
  }
  #[inline]
  pub fn controlMode(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_CONTROLMODE, Some(0)).unwrap()
  }
  #[inline]
  pub fn statorCurrent(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_STATORCURRENT, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn supplyCurrent(&self) -> f64 {
    self._tab.get::<f64>(CTREMotorStatusFrame::VT_SUPPLYCURRENT, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn fwdLimitSwitchClosed(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_FWDLIMITSWITCHCLOSED, Some(0)).unwrap()
  }
  #[inline]
  pub fn revLimitSwitchClosed(&self) -> i32 {
    self._tab.get::<i32>(CTREMotorStatusFrame::VT_REVLIMITSWITCHCLOSED, Some(0)).unwrap()
  }
}

pub struct CTREMotorStatusFrameArgs {
    pub firmwareVersion: i32,
    pub baseID: i32,
    pub deviceID: i32,
    pub outputCurrent: f64,
    pub busVoltage: f64,
    pub outputPercent: f64,
    pub outputVoltage: f64,
    pub temperature: f64,
    pub selectedSensorPosition: i32,
    pub selectedSensorVelocity: i32,
    pub closedLoopError: i32,
    pub integralAccumulator: f64,
    pub errorDerivative: f64,
    pub closedLoopTarget: f64,
    pub activeTrajectoryPosition: i32,
    pub activeTrajectoryVelocity: i32,
    pub activeTrajectoryArbFeedFwd: f64,
    pub faults: i32,
    pub resetOccured: bool,
    pub lastError: i32,
    pub controlMode: i32,
    pub statorCurrent: f64,
    pub supplyCurrent: f64,
    pub fwdLimitSwitchClosed: i32,
    pub revLimitSwitchClosed: i32,
}
impl<'a> Default for CTREMotorStatusFrameArgs {
    #[inline]
    fn default() -> Self {
        CTREMotorStatusFrameArgs {
            firmwareVersion: 0,
            baseID: 0,
            deviceID: 0,
            outputCurrent: 0.0,
            busVoltage: 0.0,
            outputPercent: 0.0,
            outputVoltage: 0.0,
            temperature: 0.0,
            selectedSensorPosition: 0,
            selectedSensorVelocity: 0,
            closedLoopError: 0,
            integralAccumulator: 0.0,
            errorDerivative: 0.0,
            closedLoopTarget: 0.0,
            activeTrajectoryPosition: 0,
            activeTrajectoryVelocity: 0,
            activeTrajectoryArbFeedFwd: 0.0,
            faults: 0,
            resetOccured: false,
            lastError: 0,
            controlMode: 0,
            statorCurrent: 0.0,
            supplyCurrent: 0.0,
            fwdLimitSwitchClosed: 0,
            revLimitSwitchClosed: 0,
        }
    }
}
pub struct CTREMotorStatusFrameBuilder<'a: 'b, 'b> {
  fbb_: &'b mut flatbuffers::FlatBufferBuilder<'a>,
  start_: flatbuffers::WIPOffset<flatbuffers::TableUnfinishedWIPOffset>,
}
impl<'a: 'b, 'b> CTREMotorStatusFrameBuilder<'a, 'b> {
  #[inline]
  pub fn add_firmwareVersion(&mut self, firmwareVersion: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_FIRMWAREVERSION, firmwareVersion, 0);
  }
  #[inline]
  pub fn add_baseID(&mut self, baseID: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_BASEID, baseID, 0);
  }
  #[inline]
  pub fn add_deviceID(&mut self, deviceID: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_DEVICEID, deviceID, 0);
  }
  #[inline]
  pub fn add_outputCurrent(&mut self, outputCurrent: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_OUTPUTCURRENT, outputCurrent, 0.0);
  }
  #[inline]
  pub fn add_busVoltage(&mut self, busVoltage: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_BUSVOLTAGE, busVoltage, 0.0);
  }
  #[inline]
  pub fn add_outputPercent(&mut self, outputPercent: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_OUTPUTPERCENT, outputPercent, 0.0);
  }
  #[inline]
  pub fn add_outputVoltage(&mut self, outputVoltage: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_OUTPUTVOLTAGE, outputVoltage, 0.0);
  }
  #[inline]
  pub fn add_temperature(&mut self, temperature: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_TEMPERATURE, temperature, 0.0);
  }
  #[inline]
  pub fn add_selectedSensorPosition(&mut self, selectedSensorPosition: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_SELECTEDSENSORPOSITION, selectedSensorPosition, 0);
  }
  #[inline]
  pub fn add_selectedSensorVelocity(&mut self, selectedSensorVelocity: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_SELECTEDSENSORVELOCITY, selectedSensorVelocity, 0);
  }
  #[inline]
  pub fn add_closedLoopError(&mut self, closedLoopError: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_CLOSEDLOOPERROR, closedLoopError, 0);
  }
  #[inline]
  pub fn add_integralAccumulator(&mut self, integralAccumulator: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_INTEGRALACCUMULATOR, integralAccumulator, 0.0);
  }
  #[inline]
  pub fn add_errorDerivative(&mut self, errorDerivative: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_ERRORDERIVATIVE, errorDerivative, 0.0);
  }
  #[inline]
  pub fn add_closedLoopTarget(&mut self, closedLoopTarget: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_CLOSEDLOOPTARGET, closedLoopTarget, 0.0);
  }
  #[inline]
  pub fn add_activeTrajectoryPosition(&mut self, activeTrajectoryPosition: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_ACTIVETRAJECTORYPOSITION, activeTrajectoryPosition, 0);
  }
  #[inline]
  pub fn add_activeTrajectoryVelocity(&mut self, activeTrajectoryVelocity: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_ACTIVETRAJECTORYVELOCITY, activeTrajectoryVelocity, 0);
  }
  #[inline]
  pub fn add_activeTrajectoryArbFeedFwd(&mut self, activeTrajectoryArbFeedFwd: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_ACTIVETRAJECTORYARBFEEDFWD, activeTrajectoryArbFeedFwd, 0.0);
  }
  #[inline]
  pub fn add_faults(&mut self, faults: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_FAULTS, faults, 0);
  }
  #[inline]
  pub fn add_resetOccured(&mut self, resetOccured: bool) {
    self.fbb_.push_slot::<bool>(CTREMotorStatusFrame::VT_RESETOCCURED, resetOccured, false);
  }
  #[inline]
  pub fn add_lastError(&mut self, lastError: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_LASTERROR, lastError, 0);
  }
  #[inline]
  pub fn add_controlMode(&mut self, controlMode: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_CONTROLMODE, controlMode, 0);
  }
  #[inline]
  pub fn add_statorCurrent(&mut self, statorCurrent: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_STATORCURRENT, statorCurrent, 0.0);
  }
  #[inline]
  pub fn add_supplyCurrent(&mut self, supplyCurrent: f64) {
    self.fbb_.push_slot::<f64>(CTREMotorStatusFrame::VT_SUPPLYCURRENT, supplyCurrent, 0.0);
  }
  #[inline]
  pub fn add_fwdLimitSwitchClosed(&mut self, fwdLimitSwitchClosed: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_FWDLIMITSWITCHCLOSED, fwdLimitSwitchClosed, 0);
  }
  #[inline]
  pub fn add_revLimitSwitchClosed(&mut self, revLimitSwitchClosed: i32) {
    self.fbb_.push_slot::<i32>(CTREMotorStatusFrame::VT_REVLIMITSWITCHCLOSED, revLimitSwitchClosed, 0);
  }
  #[inline]
  pub fn new(_fbb: &'b mut flatbuffers::FlatBufferBuilder<'a>) -> CTREMotorStatusFrameBuilder<'a, 'b> {
    let start = _fbb.start_table();
    CTREMotorStatusFrameBuilder {
      fbb_: _fbb,
      start_: start,
    }
  }
  #[inline]
  pub fn finish(self) -> flatbuffers::WIPOffset<CTREMotorStatusFrame<'a>> {
    let o = self.fbb_.end_table(self.start_);
    flatbuffers::WIPOffset::new(o.value())
  }
}

pub enum PDPStatusFrameOffset {}
#[derive(Copy, Clone, Debug, PartialEq)]

pub struct PDPStatusFrame<'a> {
  pub _tab: flatbuffers::Table<'a>,
}

impl<'a> flatbuffers::Follow<'a> for PDPStatusFrame<'a> {
    type Inner = PDPStatusFrame<'a>;
    #[inline]
    fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
        Self {
            _tab: flatbuffers::Table { buf: buf, loc: loc },
        }
    }
}

impl<'a> PDPStatusFrame<'a> {
    #[inline]
    pub fn init_from_table(table: flatbuffers::Table<'a>) -> Self {
        PDPStatusFrame {
            _tab: table,
        }
    }
    #[allow(unused_mut)]
    pub fn create<'bldr: 'args, 'args: 'mut_bldr, 'mut_bldr>(
        _fbb: &'mut_bldr mut flatbuffers::FlatBufferBuilder<'bldr>,
        args: &'args PDPStatusFrameArgs<'args>) -> flatbuffers::WIPOffset<PDPStatusFrame<'bldr>> {
      let mut builder = PDPStatusFrameBuilder::new(_fbb);
      builder.add_totalEnergy(args.totalEnergy);
      builder.add_totalPower(args.totalPower);
      builder.add_totalCurrent(args.totalCurrent);
      builder.add_temperature(args.temperature);
      builder.add_voltage(args.voltage);
      if let Some(x) = args.channelCurrent { builder.add_channelCurrent(x); }
      builder.add_module(args.module);
      builder.finish()
    }

    pub const VT_MODULE: flatbuffers::VOffsetT = 4;
    pub const VT_VOLTAGE: flatbuffers::VOffsetT = 6;
    pub const VT_TEMPERATURE: flatbuffers::VOffsetT = 8;
    pub const VT_CHANNELCURRENT: flatbuffers::VOffsetT = 10;
    pub const VT_TOTALCURRENT: flatbuffers::VOffsetT = 12;
    pub const VT_TOTALPOWER: flatbuffers::VOffsetT = 14;
    pub const VT_TOTALENERGY: flatbuffers::VOffsetT = 16;

  #[inline]
  pub fn module(&self) -> i32 {
    self._tab.get::<i32>(PDPStatusFrame::VT_MODULE, Some(0)).unwrap()
  }
  #[inline]
  pub fn voltage(&self) -> f64 {
    self._tab.get::<f64>(PDPStatusFrame::VT_VOLTAGE, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn temperature(&self) -> f64 {
    self._tab.get::<f64>(PDPStatusFrame::VT_TEMPERATURE, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn channelCurrent(&self) -> Option<flatbuffers::Vector<'a, f64>> {
    self._tab.get::<flatbuffers::ForwardsUOffset<flatbuffers::Vector<'a, f64>>>(PDPStatusFrame::VT_CHANNELCURRENT, None)
  }
  #[inline]
  pub fn totalCurrent(&self) -> f64 {
    self._tab.get::<f64>(PDPStatusFrame::VT_TOTALCURRENT, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn totalPower(&self) -> f64 {
    self._tab.get::<f64>(PDPStatusFrame::VT_TOTALPOWER, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn totalEnergy(&self) -> f64 {
    self._tab.get::<f64>(PDPStatusFrame::VT_TOTALENERGY, Some(0.0)).unwrap()
  }
}

pub struct PDPStatusFrameArgs<'a> {
    pub module: i32,
    pub voltage: f64,
    pub temperature: f64,
    pub channelCurrent: Option<flatbuffers::WIPOffset<flatbuffers::Vector<'a ,  f64>>>,
    pub totalCurrent: f64,
    pub totalPower: f64,
    pub totalEnergy: f64,
}
impl<'a> Default for PDPStatusFrameArgs<'a> {
    #[inline]
    fn default() -> Self {
        PDPStatusFrameArgs {
            module: 0,
            voltage: 0.0,
            temperature: 0.0,
            channelCurrent: None,
            totalCurrent: 0.0,
            totalPower: 0.0,
            totalEnergy: 0.0,
        }
    }
}
pub struct PDPStatusFrameBuilder<'a: 'b, 'b> {
  fbb_: &'b mut flatbuffers::FlatBufferBuilder<'a>,
  start_: flatbuffers::WIPOffset<flatbuffers::TableUnfinishedWIPOffset>,
}
impl<'a: 'b, 'b> PDPStatusFrameBuilder<'a, 'b> {
  #[inline]
  pub fn add_module(&mut self, module: i32) {
    self.fbb_.push_slot::<i32>(PDPStatusFrame::VT_MODULE, module, 0);
  }
  #[inline]
  pub fn add_voltage(&mut self, voltage: f64) {
    self.fbb_.push_slot::<f64>(PDPStatusFrame::VT_VOLTAGE, voltage, 0.0);
  }
  #[inline]
  pub fn add_temperature(&mut self, temperature: f64) {
    self.fbb_.push_slot::<f64>(PDPStatusFrame::VT_TEMPERATURE, temperature, 0.0);
  }
  #[inline]
  pub fn add_channelCurrent(&mut self, channelCurrent: flatbuffers::WIPOffset<flatbuffers::Vector<'b , f64>>) {
    self.fbb_.push_slot_always::<flatbuffers::WIPOffset<_>>(PDPStatusFrame::VT_CHANNELCURRENT, channelCurrent);
  }
  #[inline]
  pub fn add_totalCurrent(&mut self, totalCurrent: f64) {
    self.fbb_.push_slot::<f64>(PDPStatusFrame::VT_TOTALCURRENT, totalCurrent, 0.0);
  }
  #[inline]
  pub fn add_totalPower(&mut self, totalPower: f64) {
    self.fbb_.push_slot::<f64>(PDPStatusFrame::VT_TOTALPOWER, totalPower, 0.0);
  }
  #[inline]
  pub fn add_totalEnergy(&mut self, totalEnergy: f64) {
    self.fbb_.push_slot::<f64>(PDPStatusFrame::VT_TOTALENERGY, totalEnergy, 0.0);
  }
  #[inline]
  pub fn new(_fbb: &'b mut flatbuffers::FlatBufferBuilder<'a>) -> PDPStatusFrameBuilder<'a, 'b> {
    let start = _fbb.start_table();
    PDPStatusFrameBuilder {
      fbb_: _fbb,
      start_: start,
    }
  }
  #[inline]
  pub fn finish(self) -> flatbuffers::WIPOffset<PDPStatusFrame<'a>> {
    let o = self.fbb_.end_table(self.start_);
    flatbuffers::WIPOffset::new(o.value())
  }
}

pub enum PCMStatusFrameOffset {}
#[derive(Copy, Clone, Debug, PartialEq)]

pub struct PCMStatusFrame<'a> {
  pub _tab: flatbuffers::Table<'a>,
}

impl<'a> flatbuffers::Follow<'a> for PCMStatusFrame<'a> {
    type Inner = PCMStatusFrame<'a>;
    #[inline]
    fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
        Self {
            _tab: flatbuffers::Table { buf: buf, loc: loc },
        }
    }
}

impl<'a> PCMStatusFrame<'a> {
    #[inline]
    pub fn init_from_table(table: flatbuffers::Table<'a>) -> Self {
        PCMStatusFrame {
            _tab: table,
        }
    }
    #[allow(unused_mut)]
    pub fn create<'bldr: 'args, 'args: 'mut_bldr, 'mut_bldr>(
        _fbb: &'mut_bldr mut flatbuffers::FlatBufferBuilder<'bldr>,
        args: &'args PCMStatusFrameArgs) -> flatbuffers::WIPOffset<PCMStatusFrame<'bldr>> {
      let mut builder = PCMStatusFrameBuilder::new(_fbb);
      builder.add_compressorCurrent(args.compressorCurrent);
      builder.add_module(args.module);
      builder.add_compressorNotConnectedFault(args.compressorNotConnectedFault);
      builder.add_compressorShortedFault(args.compressorShortedFault);
      builder.add_compressorCurrentTooHighFault(args.compressorCurrentTooHighFault);
      builder.add_closedLoopControl(args.closedLoopControl);
      builder.add_pressureSwitchValve(args.pressureSwitchValve);
      builder.add_enabled(args.enabled);
      builder.finish()
    }

    pub const VT_MODULE: flatbuffers::VOffsetT = 4;
    pub const VT_ENABLED: flatbuffers::VOffsetT = 6;
    pub const VT_PRESSURESWITCHVALVE: flatbuffers::VOffsetT = 8;
    pub const VT_COMPRESSORCURRENT: flatbuffers::VOffsetT = 10;
    pub const VT_CLOSEDLOOPCONTROL: flatbuffers::VOffsetT = 12;
    pub const VT_COMPRESSORCURRENTTOOHIGHFAULT: flatbuffers::VOffsetT = 14;
    pub const VT_COMPRESSORSHORTEDFAULT: flatbuffers::VOffsetT = 16;
    pub const VT_COMPRESSORNOTCONNECTEDFAULT: flatbuffers::VOffsetT = 18;

  #[inline]
  pub fn module(&self) -> i32 {
    self._tab.get::<i32>(PCMStatusFrame::VT_MODULE, Some(0)).unwrap()
  }
  #[inline]
  pub fn enabled(&self) -> bool {
    self._tab.get::<bool>(PCMStatusFrame::VT_ENABLED, Some(false)).unwrap()
  }
  #[inline]
  pub fn pressureSwitchValve(&self) -> bool {
    self._tab.get::<bool>(PCMStatusFrame::VT_PRESSURESWITCHVALVE, Some(false)).unwrap()
  }
  #[inline]
  pub fn compressorCurrent(&self) -> f64 {
    self._tab.get::<f64>(PCMStatusFrame::VT_COMPRESSORCURRENT, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn closedLoopControl(&self) -> bool {
    self._tab.get::<bool>(PCMStatusFrame::VT_CLOSEDLOOPCONTROL, Some(false)).unwrap()
  }
  #[inline]
  pub fn compressorCurrentTooHighFault(&self) -> bool {
    self._tab.get::<bool>(PCMStatusFrame::VT_COMPRESSORCURRENTTOOHIGHFAULT, Some(false)).unwrap()
  }
  #[inline]
  pub fn compressorShortedFault(&self) -> bool {
    self._tab.get::<bool>(PCMStatusFrame::VT_COMPRESSORSHORTEDFAULT, Some(false)).unwrap()
  }
  #[inline]
  pub fn compressorNotConnectedFault(&self) -> bool {
    self._tab.get::<bool>(PCMStatusFrame::VT_COMPRESSORNOTCONNECTEDFAULT, Some(false)).unwrap()
  }
}

pub struct PCMStatusFrameArgs {
    pub module: i32,
    pub enabled: bool,
    pub pressureSwitchValve: bool,
    pub compressorCurrent: f64,
    pub closedLoopControl: bool,
    pub compressorCurrentTooHighFault: bool,
    pub compressorShortedFault: bool,
    pub compressorNotConnectedFault: bool,
}
impl<'a> Default for PCMStatusFrameArgs {
    #[inline]
    fn default() -> Self {
        PCMStatusFrameArgs {
            module: 0,
            enabled: false,
            pressureSwitchValve: false,
            compressorCurrent: 0.0,
            closedLoopControl: false,
            compressorCurrentTooHighFault: false,
            compressorShortedFault: false,
            compressorNotConnectedFault: false,
        }
    }
}
pub struct PCMStatusFrameBuilder<'a: 'b, 'b> {
  fbb_: &'b mut flatbuffers::FlatBufferBuilder<'a>,
  start_: flatbuffers::WIPOffset<flatbuffers::TableUnfinishedWIPOffset>,
}
impl<'a: 'b, 'b> PCMStatusFrameBuilder<'a, 'b> {
  #[inline]
  pub fn add_module(&mut self, module: i32) {
    self.fbb_.push_slot::<i32>(PCMStatusFrame::VT_MODULE, module, 0);
  }
  #[inline]
  pub fn add_enabled(&mut self, enabled: bool) {
    self.fbb_.push_slot::<bool>(PCMStatusFrame::VT_ENABLED, enabled, false);
  }
  #[inline]
  pub fn add_pressureSwitchValve(&mut self, pressureSwitchValve: bool) {
    self.fbb_.push_slot::<bool>(PCMStatusFrame::VT_PRESSURESWITCHVALVE, pressureSwitchValve, false);
  }
  #[inline]
  pub fn add_compressorCurrent(&mut self, compressorCurrent: f64) {
    self.fbb_.push_slot::<f64>(PCMStatusFrame::VT_COMPRESSORCURRENT, compressorCurrent, 0.0);
  }
  #[inline]
  pub fn add_closedLoopControl(&mut self, closedLoopControl: bool) {
    self.fbb_.push_slot::<bool>(PCMStatusFrame::VT_CLOSEDLOOPCONTROL, closedLoopControl, false);
  }
  #[inline]
  pub fn add_compressorCurrentTooHighFault(&mut self, compressorCurrentTooHighFault: bool) {
    self.fbb_.push_slot::<bool>(PCMStatusFrame::VT_COMPRESSORCURRENTTOOHIGHFAULT, compressorCurrentTooHighFault, false);
  }
  #[inline]
  pub fn add_compressorShortedFault(&mut self, compressorShortedFault: bool) {
    self.fbb_.push_slot::<bool>(PCMStatusFrame::VT_COMPRESSORSHORTEDFAULT, compressorShortedFault, false);
  }
  #[inline]
  pub fn add_compressorNotConnectedFault(&mut self, compressorNotConnectedFault: bool) {
    self.fbb_.push_slot::<bool>(PCMStatusFrame::VT_COMPRESSORNOTCONNECTEDFAULT, compressorNotConnectedFault, false);
  }
  #[inline]
  pub fn new(_fbb: &'b mut flatbuffers::FlatBufferBuilder<'a>) -> PCMStatusFrameBuilder<'a, 'b> {
    let start = _fbb.start_table();
    PCMStatusFrameBuilder {
      fbb_: _fbb,
      start_: start,
    }
  }
  #[inline]
  pub fn finish(self) -> flatbuffers::WIPOffset<PCMStatusFrame<'a>> {
    let o = self.fbb_.end_table(self.start_);
    flatbuffers::WIPOffset::new(o.value())
  }
}

pub enum StatusFrameHolderOffset {}
#[derive(Copy, Clone, Debug, PartialEq)]

pub struct StatusFrameHolder<'a> {
  pub _tab: flatbuffers::Table<'a>,
}

impl<'a> flatbuffers::Follow<'a> for StatusFrameHolder<'a> {
    type Inner = StatusFrameHolder<'a>;
    #[inline]
    fn follow(buf: &'a [u8], loc: usize) -> Self::Inner {
        Self {
            _tab: flatbuffers::Table { buf: buf, loc: loc },
        }
    }
}

impl<'a> StatusFrameHolder<'a> {
    #[inline]
    pub fn init_from_table(table: flatbuffers::Table<'a>) -> Self {
        StatusFrameHolder {
            _tab: table,
        }
    }
    #[allow(unused_mut)]
    pub fn create<'bldr: 'args, 'args: 'mut_bldr, 'mut_bldr>(
        _fbb: &'mut_bldr mut flatbuffers::FlatBufferBuilder<'bldr>,
        args: &'args StatusFrameHolderArgs) -> flatbuffers::WIPOffset<StatusFrameHolder<'bldr>> {
      let mut builder = StatusFrameHolderBuilder::new(_fbb);
      builder.add_monotonicTime(args.monotonicTime);
      builder.add_unixTime(args.unixTime);
      if let Some(x) = args.statusFrame { builder.add_statusFrame(x); }
      builder.add_statusFrame_type(args.statusFrame_type);
      builder.finish()
    }

    pub const VT_UNIXTIME: flatbuffers::VOffsetT = 4;
    pub const VT_MONOTONICTIME: flatbuffers::VOffsetT = 6;
    pub const VT_STATUSFRAME_TYPE: flatbuffers::VOffsetT = 8;
    pub const VT_STATUSFRAME: flatbuffers::VOffsetT = 10;

  #[inline]
  pub fn unixTime(&self) -> f64 {
    self._tab.get::<f64>(StatusFrameHolder::VT_UNIXTIME, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn monotonicTime(&self) -> f64 {
    self._tab.get::<f64>(StatusFrameHolder::VT_MONOTONICTIME, Some(0.0)).unwrap()
  }
  #[inline]
  pub fn statusFrame_type(&self) -> StatusFrame {
    self._tab.get::<StatusFrame>(StatusFrameHolder::VT_STATUSFRAME_TYPE, Some(StatusFrame::NONE)).unwrap()
  }
  #[inline]
  pub fn statusFrame(&self) -> Option<flatbuffers::Table<'a>> {
    self._tab.get::<flatbuffers::ForwardsUOffset<flatbuffers::Table<'a>>>(StatusFrameHolder::VT_STATUSFRAME, None)
  }
  #[inline]
  #[allow(non_snake_case)]
  pub fn statusFrame_as_ctremotor_status_frame(&self) -> Option<CTREMotorStatusFrame<'a>> {
    if self.statusFrame_type() == StatusFrame::CTREMotorStatusFrame {
      self.statusFrame().map(|u| CTREMotorStatusFrame::init_from_table(u))
    } else {
      None
    }
  }

  #[inline]
  #[allow(non_snake_case)]
  pub fn statusFrame_as_pdpstatus_frame(&self) -> Option<PDPStatusFrame<'a>> {
    if self.statusFrame_type() == StatusFrame::PDPStatusFrame {
      self.statusFrame().map(|u| PDPStatusFrame::init_from_table(u))
    } else {
      None
    }
  }

  #[inline]
  #[allow(non_snake_case)]
  pub fn statusFrame_as_pcmstatus_frame(&self) -> Option<PCMStatusFrame<'a>> {
    if self.statusFrame_type() == StatusFrame::PCMStatusFrame {
      self.statusFrame().map(|u| PCMStatusFrame::init_from_table(u))
    } else {
      None
    }
  }

}

pub struct StatusFrameHolderArgs {
    pub unixTime: f64,
    pub monotonicTime: f64,
    pub statusFrame_type: StatusFrame,
    pub statusFrame: Option<flatbuffers::WIPOffset<flatbuffers::UnionWIPOffset>>,
}
impl<'a> Default for StatusFrameHolderArgs {
    #[inline]
    fn default() -> Self {
        StatusFrameHolderArgs {
            unixTime: 0.0,
            monotonicTime: 0.0,
            statusFrame_type: StatusFrame::NONE,
            statusFrame: None,
        }
    }
}
pub struct StatusFrameHolderBuilder<'a: 'b, 'b> {
  fbb_: &'b mut flatbuffers::FlatBufferBuilder<'a>,
  start_: flatbuffers::WIPOffset<flatbuffers::TableUnfinishedWIPOffset>,
}
impl<'a: 'b, 'b> StatusFrameHolderBuilder<'a, 'b> {
  #[inline]
  pub fn add_unixTime(&mut self, unixTime: f64) {
    self.fbb_.push_slot::<f64>(StatusFrameHolder::VT_UNIXTIME, unixTime, 0.0);
  }
  #[inline]
  pub fn add_monotonicTime(&mut self, monotonicTime: f64) {
    self.fbb_.push_slot::<f64>(StatusFrameHolder::VT_MONOTONICTIME, monotonicTime, 0.0);
  }
  #[inline]
  pub fn add_statusFrame_type(&mut self, statusFrame_type: StatusFrame) {
    self.fbb_.push_slot::<StatusFrame>(StatusFrameHolder::VT_STATUSFRAME_TYPE, statusFrame_type, StatusFrame::NONE);
  }
  #[inline]
  pub fn add_statusFrame(&mut self, statusFrame: flatbuffers::WIPOffset<flatbuffers::UnionWIPOffset>) {
    self.fbb_.push_slot_always::<flatbuffers::WIPOffset<_>>(StatusFrameHolder::VT_STATUSFRAME, statusFrame);
  }
  #[inline]
  pub fn new(_fbb: &'b mut flatbuffers::FlatBufferBuilder<'a>) -> StatusFrameHolderBuilder<'a, 'b> {
    let start = _fbb.start_table();
    StatusFrameHolderBuilder {
      fbb_: _fbb,
      start_: start,
    }
  }
  #[inline]
  pub fn finish(self) -> flatbuffers::WIPOffset<StatusFrameHolder<'a>> {
    let o = self.fbb_.end_table(self.start_);
    flatbuffers::WIPOffset::new(o.value())
  }
}

#[inline]
pub fn get_root_as_status_frame_holder<'a>(buf: &'a [u8]) -> StatusFrameHolder<'a> {
  flatbuffers::get_root::<StatusFrameHolder<'a>>(buf)
}

#[inline]
pub fn get_size_prefixed_root_as_status_frame_holder<'a>(buf: &'a [u8]) -> StatusFrameHolder<'a> {
  flatbuffers::get_size_prefixed_root::<StatusFrameHolder<'a>>(buf)
}

#[inline]
pub fn finish_status_frame_holder_buffer<'a, 'b>(
    fbb: &'b mut flatbuffers::FlatBufferBuilder<'a>,
    root: flatbuffers::WIPOffset<StatusFrameHolder<'a>>) {
  fbb.finish(root, None);
}

#[inline]
pub fn finish_size_prefixed_status_frame_holder_buffer<'a, 'b>(fbb: &'b mut flatbuffers::FlatBufferBuilder<'a>, root: flatbuffers::WIPOffset<StatusFrameHolder<'a>>) {
  fbb.finish_size_prefixed(root, None);
}
}  // pub mod rj
