
"use strict";

let GetRobotInfo = require('./GetRobotInfo.js')
let StopMotion = require('./StopMotion.js')
let CmdJointTrajectory = require('./CmdJointTrajectory.js')
let SetDrivePower = require('./SetDrivePower.js')
let SetRemoteLoggerLevel = require('./SetRemoteLoggerLevel.js')
let StartMotion = require('./StartMotion.js')

module.exports = {
  GetRobotInfo: GetRobotInfo,
  StopMotion: StopMotion,
  CmdJointTrajectory: CmdJointTrajectory,
  SetDrivePower: SetDrivePower,
  SetRemoteLoggerLevel: SetRemoteLoggerLevel,
  StartMotion: StartMotion,
};
