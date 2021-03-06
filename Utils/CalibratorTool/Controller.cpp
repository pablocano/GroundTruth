//
//  Controller.cpp
//  GroundTruth
//
//  Created by Pablo Cano Montecinos on 02-09-16.
//
//

#include "Controller.h"
#include "Views/ImageView.h"
#include "Views/ColorCalibrationView/ColorCalibrationView.h"
#include "Tools/SystemCall.h"
#include "MainWindow.h"


CalibratorTool::Application* Controller::application = 0;

Controller::Controller(CalibratorTool::Application& application)
: groundTruthWrapper(0),
  colorCalibrationChanged(false),
  colorTableTimeStamp(0)
{
  this->application = &application;
  
  debugIn.setSize(5200000);
  debugOut.setSize(2800000);
  
  for(int i = 0; i < numOfMessageIDs; ++i)
  {
    waitingFor[i] = 0;
    polled[i] = false;
  }
  
  groundTruthWrapper = new GroundTruthWrapper(this);
  poll(idDebugResponse);
  poll(idDrawingManager);
  poll(idColorCalibration);
  
  SYNC;
  debugOut << DebugRequest("representation:ImageBGR");
  debugOut.finishMessage(idDebugRequest);
  
  groundTruthWrapper->start();
}

Controller::~Controller()
{
  qDeleteAll(views);
  groundTruthWrapper->quit();
  groundTruthWrapper->wait();
  delete groundTruthWrapper;
}

void Controller::compile()
{
  addCategory("GroundTruth", 0, ":/Icons/GroundTruth.png");
  addView(new ImageView("GroundTruth.Images.EastCam", *this, "LeftCam", false, true),"GroundTruth.Images");
  addView(new ImageView("GroundTruth.Images.EastCamSegmented", *this, "RightCam", true, true),"GroundTruth.Images");
  addView(new ColorCalibrationView("GroundTruth.Calibrations.ColorCalibration",*this),"GroundTruth.Calibrations");
}

void Controller::addView(CalibratorTool::Object* object, const CalibratorTool::Object* parent, int flags)
{
  views.append(object);
  application->registerObject(*object, parent, flags | CalibratorTool::Flag::showParent);
}

void Controller::addView(CalibratorTool::Object* object, const QString& categoryName, int flags)
{
  CalibratorTool::Object* category = application->resolveObject(categoryName);
  if(!category)
  {
    int lio = categoryName.lastIndexOf('.');
    QString subParentName = categoryName.mid(0, lio);
    QString name = categoryName.mid(lio + 1);
    category = addCategory(name, subParentName);
  }
  addView(object, category, flags);
}

CalibratorTool::Object* Controller::addCategory(const QString& name, const CalibratorTool::Object* parent, const char* icon)
{
  class Category : public CalibratorTool::Object
  {
  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon) {}
    
  private:
    QString name;
    QString fullName;
    QIcon icon;
    
    virtual const QString& getDisplayName() const {return name;}
    virtual const QString& getFullName() const {return fullName;}
    virtual const QIcon* getIcon() const {return &icon;}
  };
  
  CalibratorTool::Object* category = new Category(name, parent ? parent->getFullName() + "." + name : name, icon ? icon : ":/Icons/folder.png");
  views.append(category);
  application->registerObject(*category, parent, CalibratorTool::Flag::windowless | CalibratorTool::Flag::hidden);
  return category;
}

CalibratorTool::Object* Controller::addCategory(const QString& name, const QString& parentName)
{
  CalibratorTool::Object* parent = application->resolveObject(parentName);
  if(!parent)
  {
    int lio = parentName.lastIndexOf('.');
    QString subParentName = parentName.mid(0, lio);
    QString name = parentName.mid(lio + 1);
    parent = addCategory(name, subParentName);
  }
  return addCategory(name, parent);
}

void Controller::update()
{
  receive();
  if(colorCalibrationChanged && SystemCall::getTimeSince(colorTableTimeStamp) > 200)
  {
    SYNC;
    colorCalibrationChanged = false;
    colorTableTimeStamp = SystemCall::getCurrentSystemTime();
    colorModel.fromColorCalibration(colorCalibration, prevColorCalibration);
    {
      SYNC_WITH(*groundTruthWrapper);
      debugOut << colorCalibration;
      debugOut.finishMessage(idColorCalibration);
    }
  }
}

void Controller::stop()
{
  groundTruthWrapper->shouldStop = true;
}

void Controller::saveColorCalibration()
{
  SYNC_WITH(*groundTruthWrapper);
  debugOut << DebugRequest("module:GroundTruthConfiguration:saveColorCalibration");
  debugOut.finishMessage(idDebugRequest);
  
}

bool Controller::poll(MessageID id)
{
  if(waitingFor[id] > 0)
  {
    return false;
  }
  else if(polled[id])
    return true;
  else
  {
    polled[id] = true;
    switch(id)
    {
      case idDebugResponse:
      {
        SYNC;
        debugOut << DebugRequest("poll");
        debugOut.finishMessage(idDebugRequest);
        waitingFor[id] = 1;
        break;
      }
      case idDrawingManager:
      {
        SYNC;
        drawingManager.clear();
        debugOut << DebugRequest("automated requests:DrawingManager", true);
        debugOut.finishMessage(idDebugRequest);
        waitingFor[id] = 1;
      }
      
      case idColorCalibration:
      {
        SYNC;
        debugOut << DebugRequest("representation:ColorCalibration", true);
        debugOut.finishMessage(idDebugRequest);
        waitingFor[id] = 1;
        break;
      }
      default:
        return false;
    }
    return false; // Must return a boolean !
  }
  return false;
}

bool Controller::handleMessage(MessageQueue& message)
{
  SYNC;
  switch (message.getMessageID()) {
    case idProcessBegin:
    {
      message >> processIdentifier;
      if (processIdentifier == 'e') {
        currentImage = &eastImage;
      }
      else
      {
        currentImage = &westmage;
      }
      return true;
    }
    case idDebugResponse:
    {
      std::string description;
      bool enable;
      message >> description >> enable;
      if(description != "pollingFinished")
        debugRequestTable.addRequest(DebugRequest(description, enable), true);
      return true;
    }
    case idColorCalibration:
    {
      message >> colorCalibration;
      colorCalibrationChanged = true;
      return true;
    }
    case idImage:
    {
      message >> *currentImage;
      return true;
    }
    case idDebugDrawing:
    {
      if(polled[idDrawingManager] && !waitingFor[idDrawingManager]) // drawing manager not up-to-date
      {
        char shapeType,
        id;
        message >> shapeType >> id;
        const char* name = drawingManager.getDrawingName(id); // const char* is required here
        std::string type = drawingManager.getDrawingType(name);
        
        if(type == "drawingOnImage")
          incompleteImageDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType);
        //else if(type == "drawingOnField")
          //incompleteFieldDrawings[name].addShapeFromQueue(message, (::Drawings::ShapeType)shapeType);
      }
      return true;
    }
    case idProcessFinished:
    {
      if(processIdentifier == 'e')
      {
        currentImageDrawings = &eastCamImageDrawings;
      }
      else //processIdentifier == 'd'
      {
        currentImageDrawings = &westCamImageDrawings;
      }
      // Delete all image drawings received earlier from the current process
      for(Drawings::iterator i = currentImageDrawings->begin(), next; i != currentImageDrawings->end(); i = next)
      {
        next = i;
        ++next;
        if(i->second.processIdentifier == processIdentifier)
          currentImageDrawings->erase(i);
      }
      
      // Add all image drawings received now from the current process
      for(Drawings::const_iterator i = incompleteImageDrawings.begin(); i != incompleteImageDrawings.end(); ++i)
      {
        DebugDrawing& debugDrawing = (*currentImageDrawings)[i->first];
        debugDrawing = i->second;
        debugDrawing.processIdentifier = processIdentifier;
      }
      incompleteImageDrawings.clear();
      return true;
    }
    case idDrawingManager:
    {
      message >> drawingManager;
      --waitingFor[idDrawingManager];
      return true;
    }
    default:
      return false;
  }
}

void Controller::receive()
{
  SYNC_WITH(*groundTruthWrapper);
  debugIn.handleAllMessages(*this);
  debugIn.clear();
}

void Controller::drDebugDrawing(const std::string &request)
{
  SYNC;
  std::string debugRequest = std::string("debug drawing:") + request;
  for(int i = 0; i < debugRequestTable.currentNumberOfDebugRequests; ++i)
  {
    if(debugRequestTable.debugRequests[i].description == debugRequest)
    {
      DebugRequest& d = debugRequestTable.debugRequests[i];
      if (debugRequestTable.isActive(debugRequest.c_str())) {
        d.enable = false;
      }
      else{
        d.enable = true;
      }
      SYNC_WITH(*groundTruthWrapper);
      debugOut << (const DebugRequest&)d;
      debugOut.finishMessage(idDebugRequest);
      return;
    }
  }
}