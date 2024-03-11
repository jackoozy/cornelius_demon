
"use strict";

let PlaceFeedback = require('./PlaceFeedback.js');
let PickupActionGoal = require('./PickupActionGoal.js');
let PickupGoal = require('./PickupGoal.js');
let PlaceActionGoal = require('./PlaceActionGoal.js');
let ExecuteTrajectoryActionGoal = require('./ExecuteTrajectoryActionGoal.js');
let MoveGroupSequenceActionFeedback = require('./MoveGroupSequenceActionFeedback.js');
let MoveGroupResult = require('./MoveGroupResult.js');
let PlaceGoal = require('./PlaceGoal.js');
let MoveGroupActionResult = require('./MoveGroupActionResult.js');
let PlaceActionResult = require('./PlaceActionResult.js');
let PlaceAction = require('./PlaceAction.js');
let MoveGroupSequenceActionResult = require('./MoveGroupSequenceActionResult.js');
let ExecuteTrajectoryFeedback = require('./ExecuteTrajectoryFeedback.js');
let PickupFeedback = require('./PickupFeedback.js');
let MoveGroupFeedback = require('./MoveGroupFeedback.js');
let ExecuteTrajectoryAction = require('./ExecuteTrajectoryAction.js');
let MoveGroupActionGoal = require('./MoveGroupActionGoal.js');
let ExecuteTrajectoryResult = require('./ExecuteTrajectoryResult.js');
let MoveGroupSequenceGoal = require('./MoveGroupSequenceGoal.js');
let PickupActionResult = require('./PickupActionResult.js');
let MoveGroupSequenceAction = require('./MoveGroupSequenceAction.js');
let MoveGroupActionFeedback = require('./MoveGroupActionFeedback.js');
let MoveGroupSequenceActionGoal = require('./MoveGroupSequenceActionGoal.js');
let MoveGroupSequenceFeedback = require('./MoveGroupSequenceFeedback.js');
let MoveGroupAction = require('./MoveGroupAction.js');
let MoveGroupSequenceResult = require('./MoveGroupSequenceResult.js');
let PlaceResult = require('./PlaceResult.js');
let ExecuteTrajectoryActionFeedback = require('./ExecuteTrajectoryActionFeedback.js');
let PickupAction = require('./PickupAction.js');
let PlaceActionFeedback = require('./PlaceActionFeedback.js');
let ExecuteTrajectoryActionResult = require('./ExecuteTrajectoryActionResult.js');
let MoveGroupGoal = require('./MoveGroupGoal.js');
let PickupResult = require('./PickupResult.js');
let ExecuteTrajectoryGoal = require('./ExecuteTrajectoryGoal.js');
let PickupActionFeedback = require('./PickupActionFeedback.js');
let ObjectColor = require('./ObjectColor.js');
let ContactInformation = require('./ContactInformation.js');
let DisplayTrajectory = require('./DisplayTrajectory.js');
let CartesianPoint = require('./CartesianPoint.js');
let WorkspaceParameters = require('./WorkspaceParameters.js');
let PlannerInterfaceDescription = require('./PlannerInterfaceDescription.js');
let CartesianTrajectoryPoint = require('./CartesianTrajectoryPoint.js');
let DisplayRobotState = require('./DisplayRobotState.js');
let GripperTranslation = require('./GripperTranslation.js');
let PlannerParams = require('./PlannerParams.js');
let PlanningSceneComponents = require('./PlanningSceneComponents.js');
let LinkScale = require('./LinkScale.js');
let AllowedCollisionMatrix = require('./AllowedCollisionMatrix.js');
let LinkPadding = require('./LinkPadding.js');
let PlanningScene = require('./PlanningScene.js');
let MoveItErrorCodes = require('./MoveItErrorCodes.js');
let PositionIKRequest = require('./PositionIKRequest.js');
let ConstraintEvalResult = require('./ConstraintEvalResult.js');
let BoundingVolume = require('./BoundingVolume.js');
let AttachedCollisionObject = require('./AttachedCollisionObject.js');
let MotionSequenceItem = require('./MotionSequenceItem.js');
let AllowedCollisionEntry = require('./AllowedCollisionEntry.js');
let KinematicSolverInfo = require('./KinematicSolverInfo.js');
let MotionSequenceResponse = require('./MotionSequenceResponse.js');
let CollisionObject = require('./CollisionObject.js');
let MotionSequenceRequest = require('./MotionSequenceRequest.js');
let GenericTrajectory = require('./GenericTrajectory.js');
let MotionPlanRequest = require('./MotionPlanRequest.js');
let MotionPlanDetailedResponse = require('./MotionPlanDetailedResponse.js');
let JointLimits = require('./JointLimits.js');
let VisibilityConstraint = require('./VisibilityConstraint.js');
let RobotTrajectory = require('./RobotTrajectory.js');
let PlaceLocation = require('./PlaceLocation.js');
let Grasp = require('./Grasp.js');
let TrajectoryConstraints = require('./TrajectoryConstraints.js');
let PositionConstraint = require('./PositionConstraint.js');
let JointConstraint = require('./JointConstraint.js');
let PlanningSceneWorld = require('./PlanningSceneWorld.js');
let PlanningOptions = require('./PlanningOptions.js');
let CostSource = require('./CostSource.js');
let MotionPlanResponse = require('./MotionPlanResponse.js');
let Constraints = require('./Constraints.js');
let CartesianTrajectory = require('./CartesianTrajectory.js');
let OrientationConstraint = require('./OrientationConstraint.js');
let OrientedBoundingBox = require('./OrientedBoundingBox.js');
let RobotState = require('./RobotState.js');

module.exports = {
  PlaceFeedback: PlaceFeedback,
  PickupActionGoal: PickupActionGoal,
  PickupGoal: PickupGoal,
  PlaceActionGoal: PlaceActionGoal,
  ExecuteTrajectoryActionGoal: ExecuteTrajectoryActionGoal,
  MoveGroupSequenceActionFeedback: MoveGroupSequenceActionFeedback,
  MoveGroupResult: MoveGroupResult,
  PlaceGoal: PlaceGoal,
  MoveGroupActionResult: MoveGroupActionResult,
  PlaceActionResult: PlaceActionResult,
  PlaceAction: PlaceAction,
  MoveGroupSequenceActionResult: MoveGroupSequenceActionResult,
  ExecuteTrajectoryFeedback: ExecuteTrajectoryFeedback,
  PickupFeedback: PickupFeedback,
  MoveGroupFeedback: MoveGroupFeedback,
  ExecuteTrajectoryAction: ExecuteTrajectoryAction,
  MoveGroupActionGoal: MoveGroupActionGoal,
  ExecuteTrajectoryResult: ExecuteTrajectoryResult,
  MoveGroupSequenceGoal: MoveGroupSequenceGoal,
  PickupActionResult: PickupActionResult,
  MoveGroupSequenceAction: MoveGroupSequenceAction,
  MoveGroupActionFeedback: MoveGroupActionFeedback,
  MoveGroupSequenceActionGoal: MoveGroupSequenceActionGoal,
  MoveGroupSequenceFeedback: MoveGroupSequenceFeedback,
  MoveGroupAction: MoveGroupAction,
  MoveGroupSequenceResult: MoveGroupSequenceResult,
  PlaceResult: PlaceResult,
  ExecuteTrajectoryActionFeedback: ExecuteTrajectoryActionFeedback,
  PickupAction: PickupAction,
  PlaceActionFeedback: PlaceActionFeedback,
  ExecuteTrajectoryActionResult: ExecuteTrajectoryActionResult,
  MoveGroupGoal: MoveGroupGoal,
  PickupResult: PickupResult,
  ExecuteTrajectoryGoal: ExecuteTrajectoryGoal,
  PickupActionFeedback: PickupActionFeedback,
  ObjectColor: ObjectColor,
  ContactInformation: ContactInformation,
  DisplayTrajectory: DisplayTrajectory,
  CartesianPoint: CartesianPoint,
  WorkspaceParameters: WorkspaceParameters,
  PlannerInterfaceDescription: PlannerInterfaceDescription,
  CartesianTrajectoryPoint: CartesianTrajectoryPoint,
  DisplayRobotState: DisplayRobotState,
  GripperTranslation: GripperTranslation,
  PlannerParams: PlannerParams,
  PlanningSceneComponents: PlanningSceneComponents,
  LinkScale: LinkScale,
  AllowedCollisionMatrix: AllowedCollisionMatrix,
  LinkPadding: LinkPadding,
  PlanningScene: PlanningScene,
  MoveItErrorCodes: MoveItErrorCodes,
  PositionIKRequest: PositionIKRequest,
  ConstraintEvalResult: ConstraintEvalResult,
  BoundingVolume: BoundingVolume,
  AttachedCollisionObject: AttachedCollisionObject,
  MotionSequenceItem: MotionSequenceItem,
  AllowedCollisionEntry: AllowedCollisionEntry,
  KinematicSolverInfo: KinematicSolverInfo,
  MotionSequenceResponse: MotionSequenceResponse,
  CollisionObject: CollisionObject,
  MotionSequenceRequest: MotionSequenceRequest,
  GenericTrajectory: GenericTrajectory,
  MotionPlanRequest: MotionPlanRequest,
  MotionPlanDetailedResponse: MotionPlanDetailedResponse,
  JointLimits: JointLimits,
  VisibilityConstraint: VisibilityConstraint,
  RobotTrajectory: RobotTrajectory,
  PlaceLocation: PlaceLocation,
  Grasp: Grasp,
  TrajectoryConstraints: TrajectoryConstraints,
  PositionConstraint: PositionConstraint,
  JointConstraint: JointConstraint,
  PlanningSceneWorld: PlanningSceneWorld,
  PlanningOptions: PlanningOptions,
  CostSource: CostSource,
  MotionPlanResponse: MotionPlanResponse,
  Constraints: Constraints,
  CartesianTrajectory: CartesianTrajectory,
  OrientationConstraint: OrientationConstraint,
  OrientedBoundingBox: OrientedBoundingBox,
  RobotState: RobotState,
};
