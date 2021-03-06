cmake_minimum_required(VERSION 3.1.0)

PROJECT (GroundTruth)
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build Type")
set( MessageIDsPath ${CMAKE_CURRENT_SOURCE_DIR}/../uchile-robotics-team/Src/Tools/MessageQueue CACHE PATH "Path to MessageIDs file used by BHuman code")

set( GROUNDTRUTH_SRC
  src/Modules/Camera.cpp
  src/Modules/Camera.h
  src/Modules/BackgroundModel.cpp
  src/Modules/BackgroundModel.h
  src/Modules/BallPerceptor.cpp
  src/Modules/BallPerceptor.h
  src/Modules/BlobProvider.cpp
  src/Modules/BlobProvider.h
  src/Modules/GroundTruthConfiguration.cpp
  src/Modules/GroundTruthConfiguration.h
  src/Modules/GroundTruthProvider.cpp
  src/Modules/GroundTruthProvider.h
  src/Modules/Regionizer.cpp
  src/Modules/Regionizer.h
  src/Modules/RobotPerceptor.cpp
  src/Modules/RobotPerceptor.h
  src/Modules/RobotPoseProvider.cpp
  src/Modules/RobotPoseProvider.h
  src/Modules/Segmentator.cpp
  src/Modules/Segmentator.h
  src/Processes/GroundTruth.cpp
  src/Processes/GroundTruth.h
  src/Representations/BallPerception.cpp
  src/Representations/BallPerception.h
  src/Representations/Blobs.cpp
  src/Representations/Blobs.h
  src/Representations/CameraInfo.cpp
  src/Representations/CameraInfo.h
  src/Representations/ColorModel/ColorCalibration.h
  src/Representations/ColorModel/ColorModel.cpp
  src/Representations/ColorModel/ColorModel.h
  src/Representations/FrameInfo.h
  src/Representations/FieldDimensions.h
  src/Representations/FieldDimensions.cpp
  src/Representations/GroundTruthMessages.h
  src/Representations/Image.cpp
  src/Representations/Image.h
  src/Representations/Regions.cpp
  src/Representations/Regions.h
  src/Representations/RobotPercept.cpp
  src/Representations/RobotPercept.h
  src/Representations/RobotPose.cpp
  src/Representations/RobotPose.h
  src/Tools/ColorClasses.cpp
  src/Tools/ColorClasses.h
  src/Tools/ColorModelConversions.h
  src/Tools/Comm/Comm.h
  src/Tools/Comm/GroundTruthMessageHandler.cpp
  src/Tools/Comm/GroundTruthMessageHandler.h
  src/Tools/Comm/SPLStandardMessage.h
  src/Tools/Comm/SPLStandardMessageWrapper.cpp
  src/Tools/Comm/SPLStandardMessageWrapper.h
  src/Tools/Comm/UdpComm.cpp
  src/Tools/Comm/UdpComm.h
  src/Tools/Debugging/Debugging.h
  src/Tools/Debugging/DebugRequest.cpp
  src/Tools/Debugging/DebugRequest.h
  src/Tools/Debugging/DebugDrawings.cpp
  src/Tools/Debugging/DebugDrawings.h
  src/Tools/Enum.cpp
  src/Tools/Enum.h
  src/Tools/File.cpp
  src/Tools/File.h
  src/Tools/Global.cpp
  src/Tools/Global.h
  src/Tools/Math/Line.cpp
  src/Tools/Math/Line.h
  src/Tools/Math/Pose2D.h
  src/Tools/Math/Range.h
  src/Tools/Math/Transformation.cpp
  src/Tools/Math/Transformation.h
  src/Tools/Math/Vector2.h
  src/Tools/Messages/MessageQueue.cpp
  src/Tools/Messages/MessageQueue.h
  src/Tools/ModuleManager/Blackboard.cpp
  src/Tools/ModuleManager/Blackboard.h
  src/Tools/ModuleManager/Module.cpp
  src/Tools/ModuleManager/Module.h
  src/Tools/ModuleManager/ModuleManager.cpp
  src/Tools/ModuleManager/ModuleManager.h
  src/Tools/Process.cpp
  src/Tools/Process.h
  src/Tools/Settings.cpp
  src/Tools/Settings.h
  src/Tools/Streamable.h
  src/Tools/SystemCall.cpp
  src/Tools/SystemCall.h

  ${MessageIDsPath}/MessageIDs.h
)


set( CAMERA_CALIBRATION
   Utils/CameraCalibration/calibration.cpp
   Utils/CameraCalibration/settings.h
   src/Tools/File.cpp
   src/Tools/File.h
)

set( FIELD_CALIBRATOR
   Utils/FieldCalibrator/fieldCalibrator.cpp
   src/Representations/FieldDimensions.h
   src/Representations/FieldDimensions.cpp
)

set( CALIBRATOR_SRC_DIR Utils/CalibratorTool)

QT4_WRAP_CPP(CALIBRATION_TOOL_MOCSrcs ${CALIBRATOR_SRC_DIR}/ListViewsDockWidget.h ${CALIBRATOR_SRC_DIR}/MainWindow.h ${CALIBRATOR_SRC_DIR}/RegisteredDockWidget.h ${CALIBRATOR_SRC_DIR}/Views/ImageView.h ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/ColorCalibrationView.h ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/RangeSelector.h ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/ThresholdSelector.h)

QT4_ADD_RESOURCES(CALIBRATION_TOOL_RESOURCES_RCC ${CALIBRATOR_SRC_DIR}/CalibratorTool.qrc)

set( CALIBRATOR_TOOL_SRC
  ${CALIBRATOR_SRC_DIR}/CalibratorTool.h
  ${CALIBRATOR_SRC_DIR}/Controller.cpp
  ${CALIBRATOR_SRC_DIR}/Controller.h
  ${CALIBRATOR_SRC_DIR}/GroundTruthWrapper.cpp
  ${CALIBRATOR_SRC_DIR}/GroundTruthWrapper.h
  ${CALIBRATOR_SRC_DIR}/ListViewsDockWidget.cpp
  ${CALIBRATOR_SRC_DIR}/ListViewsDockWidget.h
  ${CALIBRATOR_SRC_DIR}/Main.cpp
  ${CALIBRATOR_SRC_DIR}/MainWindow.cpp
  ${CALIBRATOR_SRC_DIR}/MainWindow.h
  ${CALIBRATOR_SRC_DIR}/RegisteredDockWidget.cpp
  ${CALIBRATOR_SRC_DIR}/RegisteredDockWidget.h
  ${CALIBRATOR_SRC_DIR}/Synchronization.h
  ${CALIBRATOR_SRC_DIR}/Views/ImageView.cpp
  ${CALIBRATOR_SRC_DIR}/Views/ImageView.h
  ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/ColorCalibrationView.cpp
  ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/ColorCalibrationView.h
  ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/RangeSelector.cpp
  ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/RangeSelector.h
  ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/ThresholdSelector.cpp
  ${CALIBRATOR_SRC_DIR}/Views/ColorCalibrationView/ThresholdSelector.h
  ${CALIBRATOR_SRC_DIR}/Visualization/DebugDrawing.cpp
  ${CALIBRATOR_SRC_DIR}/Visualization/DebugDrawing.h
  ${CALIBRATOR_SRC_DIR}/Visualization/PaintMethods.cpp
  ${CALIBRATOR_SRC_DIR}/Visualization/PaintMethods.h
)

set( VISION_TOOL_SRC_DIR Utils/VisionTool)

QT4_WRAP_CPP(VISION_TOOL_MOCSrcs ${VISION_TOOL_SRC_DIR}/VisionToolWidget.h ${VISION_TOOL_SRC_DIR}/MainWindow.h)

set( VISION_TOOL_SRC
  ${VISION_TOOL_SRC_DIR}/Main.cpp
  ${VISION_TOOL_SRC_DIR}/MainWindow.cpp
  ${VISION_TOOL_SRC_DIR}/MainWindow.h
  ${VISION_TOOL_SRC_DIR}/CommunicationHandler.cpp
  ${VISION_TOOL_SRC_DIR}/CommunicationHandler.h
  ${VISION_TOOL_SRC_DIR}/VisionTool.cpp
  ${VISION_TOOL_SRC_DIR}/VisionTool.h
  ${VISION_TOOL_SRC_DIR}/VisionToolWidget.cpp
  ${VISION_TOOL_SRC_DIR}/VisionToolWidget.h
  src/Tools/Comm/GroundTruthMessageHandler.cpp
  src/Tools/Comm/GroundTruthMessageHandler.h
  src/Tools/Comm/SPLStandardMessage.h
  src/Tools/Comm/SPLStandardMessageWrapper.cpp
  src/Tools/Comm/SPLStandardMessageWrapper.h
  src/Tools/Comm/UdpComm.cpp
  src/Tools/Comm/UdpComm.h
  src/Tools/Enum.cpp
  src/Tools/Enum.h
  src/Tools/File.cpp
  src/Tools/File.h
  src/Tools/Global.cpp
  src/Tools/Global.h
  src/Tools/Math/Pose2D.h
  src/Tools/Math/Vector2.h
  src/Tools/Messages/MessageQueue.cpp
  src/Tools/Messages/MessageQueue.h
  src/Tools/Settings.cpp
  src/Tools/Settings.h
  src/Tools/Streamable.h
  src/Tools/SystemCall.cpp
  src/Tools/SystemCall.h

  ${MessageIDsPath}/MessageIDs.h
)

set( LIBQXT_SRC_DIR Utils/libqxt)

QT4_WRAP_CPP(LIBQXT_MOC_Srcs ${LIBQXT_SRC_DIR}/qxtspanslider_p.h ${LIBQXT_SRC_DIR}/qxtspanslider.h)

set( LIBQXT_SRC
  ${LIBQXT_SRC_DIR}/qxtglobal.h
  ${LIBQXT_SRC_DIR}/qxtnamespace.h
  ${LIBQXT_SRC_DIR}/qxtspanslider_p.h
  ${LIBQXT_SRC_DIR}/qxtspanslider.cpp
  ${LIBQXT_SRC_DIR}/qxtspanslider.h
)

source_group(src\\Processes REGULAR_EXPRESSION "src/Processes/*")
source_group(src\\Modules REGULAR_EXPRESSION "Modules/*")
source_group(src\\Representations REGULAR_EXPRESSION "Representations/*")
source_group(src\\Representations\\ColorModel REGULAR_EXPRESSION "Representations/ColorModel/*")
source_group(src\\Tools REGULAR_EXPRESSION "Tools/*")
source_group(src\\Tools\\ModuleManager REGULAR_EXPRESSION "Tools/ModuleManager/*")
source_group(src\\Tools\\Comm REGULAR_EXPRESSION "Tools/Comm/*")
source_group(src\\Tools\\Debugging REGULAR_EXPRESSION "Tools/Debugging/*")
source_group(src\\Tools\\Messages REGULAR_EXPRESSION "Tools/Messages/*")
source_group(src\\Tools\\Math REGULAR_EXPRESSION "Tools/Math/*")
source_group(Utils\\CalibratorTool REGULAR_EXPRESSION "Utils/CalibratorTool/*")
source_group(Utils\\CalibratorTool\\Views REGULAR_EXPRESSION "Utils/CalibratorTool/Views/*")
source_group(Utils\\CalibratorTool\\Visualization REGULAR_EXPRESSION "Utils/CalibratorTool/Visualization/*")
source_group(Utils\\CalibratorTool\\Views\\ColorCalibrationView REGULAR_EXPRESSION "Utils/CalibratorTool/Views/ColorCalibrationView/*")
source_group(Utils\\libqxt REGULAR_EXPRESSION "Utils/libqxt*")
source_group(Utils\\VisionTool REGULAR_EXPRESSION "Utils/VisionTool/*")

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
add_definitions(-DOSX)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
add_definitions(-DLINUX)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
add_definitions(-DWINDOWS)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(${OpenCV_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/Utils/libqxt ${MessageIDsPath})

add_library( groundTruth SHARED ${GROUNDTRUTH_SRC} ${OpenCV_INCLUDE})
target_link_libraries( groundTruth ${OpenCV_LIBS} )

add_library( libqxt STATIC ${LIBQXT_SRC} ${LIBQXT_MOC_Srcs})
target_link_libraries(libqxt ${QT_LIBRARIES})
target_compile_definitions(libqxt PRIVATE QXT_STATIC )

add_executable( calibration ${CAMERA_CALIBRATION} ${OpenCV_INCLUDE})
target_link_libraries( calibration ${OpenCV_LIBS} )

add_executable( fieldCalibrator ${FIELD_CALIBRATOR} ${OpenCV_INCLUDE})
target_link_libraries( fieldCalibrator ${OpenCV_LIBS} )

add_executable(gt src/Processes/main.cpp)
target_link_libraries(gt groundTruth)

include_directories(${OpenCV_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/Utils/CalibratorTool  ${CMAKE_CURRENT_BINARY_DIR})

add_executable(calibrationTool ${CALIBRATOR_TOOL_SRC} ${CALIBRATION_TOOL_MOCSrcs} ${CALIBRATION_TOOL_RESOURCES_RCC})
target_link_libraries(calibrationTool groundTruth libqxt ${QT_LIBRARIES} ${OpenCV_LIBS})
target_compile_definitions(calibrationTool PRIVATE QXT_STATIC)

add_executable(visionTool ${VISION_TOOL_SRC} ${VISION_TOOL_MOCSrcs} ${OpenCV_INCLUDE})
target_link_libraries(visionTool ${OpenCV_LIBS} ${QT_LIBRARIES})
