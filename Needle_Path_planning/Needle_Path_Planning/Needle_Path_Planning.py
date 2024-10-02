import logging
import os
from typing import Optional
import vtk, SimpleITK, sitkUtils
import slicer
from slicer.i18n import tr as _
from slicer.ScriptedLoadableModule import ScriptedLoadableModule, ScriptedLoadableModuleWidget, ScriptedLoadableModuleLogic, ScriptedLoadableModuleTest
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import parameterNodeWrapper
from slicer import vtkMRMLScalarVolumeNode, vtkMRMLMarkupsFiducialNode, vtkMRMLLabelMapVolumeNode
import numpy as np
import math

class Needle_Path_Planning(ScriptedLoadableModule):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent.title = _("Needle Path Planning")
        self.parent.categories = [_("Neurosurgery Path Planning")]
        self.parent.dependencies = []
        self.parent.contributors = ["Jiaheng Wang KCL"]
        self.parent.helpText = _("This module provides functionality to plan a straight path for needle insertion, optimizing distance from obstacles.")
        self.parent.acknowledgementText = _("Thanks")

@parameterNodeWrapper
class NeedlePathPlanningParameterNode:
    entryPoints: vtkMRMLMarkupsFiducialNode
    targetPoints: vtkMRMLMarkupsFiducialNode
    hypothalamusLabelMap: vtkMRMLLabelMapVolumeNode
    vesselsLabelMap: vtkMRMLLabelMapVolumeNode
    ventricleLabelMap: vtkMRMLLabelMapVolumeNode
    cortexLabelMap: vtkMRMLLabelMapVolumeNode

class NeedlePathPlanningWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None

    def setup(self):
        super().setup()
        uiWidget = slicer.util.loadUI(self.resourcePath("UI/NeedlePathPlanning.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)
        uiWidget.setMRMLScene(slicer.mrmlScene)

        self.logic = NeedlePathPlanningLogic()

        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

        self.ui.CountThePoints.connect("clicked(bool)", self.onPointCounting)
        self.ui.PlanThePath.connect("clicked(bool)", self.onPathPlanning)
        self.ui.PrepareTheData.connect("clicked(bool)", self.onDataSending)

        self.initializeParameterNode()

    def initializeParameterNode(self):
        self.setParameterNode(self.logic.getParameterNode())
        if not self._parameterNode.entryPoints:
            self._parameterNode.entryPoints = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLMarkupsFiducialNode")
        if not self._parameterNode.targetPoints:
            self._parameterNode.targetPoints = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLMarkupsFiducialNode")
        if not self._parameterNode.hypothalamusLabelMap:
            self._parameterNode.hypothalamusLabelMap = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLLabelMapVolumeNode")

    def setParameterNode(self, inputParameterNode: Optional[NeedlePathPlanningParameterNode]):
        if self._parameterNode:
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._onParameterNodeModified)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._onParameterNodeModified)
            self._onParameterNodeModified()

    def _onParameterNodeModified(self):
        if (self._parameterNode and self._parameterNode.entryPoints and 
            self._parameterNode.targetPoints and self._parameterNode.hypothalamusLabelMap):
            self.ui.applyButton.enabled = True
        else:
            self.ui.applyButton.enabled = False

    def onPointCounting(self):
        """ Check if target points are inside hypothalamus label map """
        self.logic.summarizeData(self._parameterNode.entryPoints, self._parameterNode.targetPoints)

    def onPathPlanning(self):
        """ Plan the path and ensure cortex angle threshold and vessel distance """
        cortexAngleThreshold = self.ui.CortexAngleThreshold.value  
        needleLengthThreshold = self.ui.NeedleLengthThreshold.value  

        self.logic.performPathPlanning(
            entryPoints=self._parameterNode.entryPoints,
            targetPoints=self._parameterNode.targetPoints,
            hypothalamusLabelMap=self._parameterNode.hypothalamusLabelMap,
            vesselsLabelMap=self._parameterNode.vesselsLabelMap,
            ventricleLabelMap=self._parameterNode.ventricleLabelMap,
            cortexLabelMap=self._parameterNode.cortexLabelMap,
            cortexAngleThreshold=cortexAngleThreshold,  
            needleLengthThreshold=needleLengthThreshold  
        )

    def onDataSending(self):
        """ Prepare the data for ROS """
        self.logic.prepareROSData()

class NeedlePathPlanningLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        super().__init__()
        self.shNode = slicer.mrmlScene.GetSubjectHierarchyNode()

    def getParameterNode(self):
        return NeedlePathPlanningParameterNode(super().getParameterNode())

    def summarizeData(self, entryPoints, targetPoints):
        numEntries = entryPoints.GetNumberOfControlPoints()
        numTargets = targetPoints.GetNumberOfControlPoints()
        logging.info(f"Number of entry points: {numEntries}, Number of target points: {numTargets}")
        # Check if target points are inside hypothalamus
        self.checkPointsInLabelMap(targetPoints, self._parameterNode.hypothalamusLabelMap)

    def checkPointsInLabelMap(self, points, labelMap):
        pointCoordinates = slicer.util.arrayFromMarkupsControlPoints(points)
        labelMapArray = slicer.util.arrayFromVolume(labelMap)
        for point in pointCoordinates:
            ijk = np.round(slicer.util.rasToIJK(point, labelMap)).astype(int)
            if labelMapArray[ijk[0], ijk[1], ijk[2]] != 0:
                logging.info(f"Point {point} is inside the label map.")
            else:
                logging.info(f"Point {point} is outside the label map.")

    def performPathPlanning(self, entryPoints, targetPoints, hypothalamusLabelMap, vesselsLabelMap, ventricleLabelMap, cortexLabelMap, cortexAngleThreshold, needleLengthThreshold):
        logging.info("Path planning is in progress...")
        
        # Step 1: Filter paths based on angle and check intersection with the cortex
        validPaths = self.calculatePathAngles(entryPoints, targetPoints, cortexLabelMap, cortexAngleThreshold)
        
        # Step 2: Filter paths based on length
        validPaths = self.filterPathsByLength(validPaths, needleLengthThreshold)
        
        # Step 3: Ensure paths maximize distance from vessels
        self.maximizeDistanceFromVessels(validPaths, vesselsLabelMap)

    def filterPathsByLength(self, paths, needleLengthThreshold):
        """ Filter out paths that are longer than the specified length threshold """
        validPaths = []
        for entry, target in paths:
            length = np.linalg.norm(target - entry)
            if length <= needleLengthThreshold:
                logging.info(f"Path from {entry} to {target} meets the length threshold: {length} <= {needleLengthThreshold}")
                validPaths.append((entry, target))
            else:
                logging.info(f"Path from {entry} to {target} exceeds the length threshold: {length} > {needleLengthThreshold}")
        
        if not validPaths:
            logging.warning("No valid paths found that meet the length threshold.")
        
        return validPaths

    def calculatePathAngles(self, paths, cortexLabelMap, cortexAngleThreshold):
        """ Ensure the cortex angle threshold is met and the path avoids the cortex. """

        entryCoords = [p[0] for p in paths]
        targetCoords = [p[1] for p in paths]
        cortexArray = slicer.util.arrayFromVolume(cortexLabelMap)  # Extract cortex label map as numpy array

        validPaths = []

        # Calculate transformation matrices to convert between RAS and IJK
        spacing = np.array(cortexLabelMap.GetSpacing())
        rotation = vtk.vtkMatrix4x4()
        cortexLabelMap.GetIJKToRASDirectionMatrix(rotation)
        rotation = slicer.util.arrayFromVTKMatrix(rotation)[0:3, 0:3]
        origin = np.array(cortexLabelMap.GetOrigin())

        for entry, target in zip(entryCoords, targetCoords):
            vector = target - entry
            normalizedVector = vector / np.linalg.norm(vector)  # Normalize the vector

            # Find the point on the cortex closest to the target point
            ijkPoint = np.round(self.convert_RAS_to_IJK(target, spacing, rotation, origin)).astype(int)

            if (0 <= ijkPoint[0] < cortexArray.shape[0] and
                    0 <= ijkPoint[1] < cortexArray.shape[1] and
                    0 <= ijkPoint[2] < cortexArray.shape[2]):
                # Calculate normal from the cortex at this point
                cortexNormal = self.calculateNormalFromCortex(ijkPoint, cortexArray)

                # Convert to angles
                angle = np.arccos(np.dot(normalizedVector, cortexNormal)) * 180 / np.pi
                if angle > 90:
                    angle = 180 - angle  # Ensure angle is between 0 and 90 degrees

                if angle <= cortexAngleThreshold:
                    logging.info(f"Path from {entry} to {target} meets the angle threshold: {angle} <= {cortexAngleThreshold}")
                    if not self.isPathIntersectingCortex(entry, target, cortexArray, spacing, rotation, origin):
                        validPaths.append((entry, target))
                else:
                    logging.info(f"Path from {entry} to {target} exceeds the angle threshold: {angle} > {cortexAngleThreshold}")
            else:
                logging.info(f"Target point {target} is outside the cortex area.")

        if validPaths:
            logging.info(f"Number of valid paths: {len(validPaths)}")
        else:
            logging.warning("No valid paths found that meet the cortex angle threshold.")
        
        return validPaths

    def calculateNormalFromCortex(self, ijkPoint, cortexArray):
        """ Calculate normal vector from the cortex at the given IJK point """
        # Use central difference method to estimate the gradient in the cortex label map
        gradient = np.array([0.0, 0.0, 0.0])

        # Define neighboring points for finite difference calculation
        neighbors = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)]

        for i, (dx, dy, dz) in enumerate(neighbors):
            neighbor_ijk = ijkPoint + np.array([dx, dy, dz])
            if (0 <= neighbor_ijk[0] < cortexArray.shape[0] and
                    0 <= neighbor_ijk[1] < cortexArray.shape[1] and
                    0 <= neighbor_ijk[2] < cortexArray.shape[2]):
                gradient[i // 2] += (cortexArray[neighbor_ijk[0], neighbor_ijk[1], neighbor_ijk[2]] -
                                    cortexArray[ijkPoint[0], ijkPoint[1], ijkPoint[2]])

        # Normalize the gradient to obtain the normal
        norm = np.linalg.norm(gradient)
        if norm != 0:
            gradient /= norm
        return gradient


    def isPathIntersectingCortex(self, entry, target, cortexArray, spacing, rotation, origin):
        """ Check if the line from entry to target intersects with the cortex. """

        # Discretize the path between entry and target
        numSteps = int(np.linalg.norm(target - entry) / spacing[0])  # Estimate number of steps based on spacing
        for step in range(numSteps + 1):
            point = entry + (target - entry) * (step / numSteps)  # Interpolate points along the line
            ijkPoint = np.round(self.convert_RAS_to_IJK(point, spacing, rotation, origin)).astype(int)

            # Check if the point falls inside the cortex (label map value > 0)
            if (0 <= ijkPoint[0] < cortexArray.shape[0] and
                    0 <= ijkPoint[1] < cortexArray.shape[1] and
                    0 <= ijkPoint[2] < cortexArray.shape[2]):
                if cortexArray[ijkPoint[0], ijkPoint[1], ijkPoint[2]] > 0:
                    return True  # The path intersects the cortex

        return False  # No intersection with the cortex


    def maximizeDistanceFromVessels(self, entryPoints, targetPoints, vesselsLabelMap):
        """ Ensure the planned path maximizes the distance from vessels """
        entryCoords = slicer.util.arrayFromMarkupsControlPoints(entryPoints)
        targetCoords = slicer.util.arrayFromMarkupsControlPoints(targetPoints)
        vesselsArray = slicer.util.arrayFromVolume(vesselsLabelMap)
        for entry, target in zip(entryCoords, targetCoords):
            # Calculate distance from vessel map (simplified example)
            distance = np.linalg.norm(np.array(vesselsArray.shape) / 2 - (entry + target) / 2)
            logging.info(f"Path from {entry} to {target} has distance to vessels: {distance}")

    def prepareROSData(self):
        """ Prepare data for ROS and send using OpenIGTLink """
        logging.info("Preparing data for ROS integration.")

        if not hasattr(self, 'start_point_for_ROS') or not hasattr(self, 'end_point_for_ROS'):
            logging.error("No valid path found to prepare data for ROS.")
            return

        # Prepare the data to send
        entry_point = self.start_point_for_ROS
        target_point = self.end_point_for_ROS

        # Create the message structure for ROS
        ros_data = {
            "entry_point": entry_point.tolist(),  # Convert to a list format
            "target_point": target_point.tolist()
        }

        logging.info(f"Prepared data for ROS: Entry point {entry_point}, Target point {target_point}")

        # Send data using OpenIGTLink
        try:
            connector_node = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLIGTLConnectorNode")
            if not connector_node:
                raise RuntimeError("OpenIGTLink connector node not found. Ensure that the connector is properly set up.")

            message_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTextNode", "ROS_Path_Message")
            message_node.SetText(str(ros_data))  # Convert dictionary to string
            connector_node.PushNode(message_node)
            logging.info("Data successfully sent to ROS via OpenIGTLink.")
            
        except Exception as e:
            logging.error(f"Failed to send data to ROS: {str(e)}")


class NeedlePathPlanningTest(ScriptedLoadableModuleTest):
    def setUp(self):
        slicer.mrmlScene.Clear()

    def runTest(self):
        self.setUp()
        self.testPathPlanning()

    def testPathPlanning(self):
        logic = NeedlePathPlanningLogic()
        logic.summarizeData(None, None)
