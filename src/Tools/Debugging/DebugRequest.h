//
//  DebugRequest.h
//  GroundTruth
//
//  Created by Pablo Cano Montecinos on 06-09-16.
//
//
#pragma once

#include "Tools/Messages/MessageQueue.h"
#include <string>
#include <unordered_map>

class DebugRequest {
public:
  /** Constructor, resets the table */
  DebugRequest();
  DebugRequest(const std::string& description, bool enable = true);
  
  std::string description;
  bool enable;
};

/**
 * Streaming operator that reads a DebugRequest from a stream.
 * @param stream The stream from which is read.
 * @param debugRequest The DebugRequest object.
 * @return The stream.
 */
MessageQueue& operator>>(MessageQueue& stream, DebugRequest& debugRequest);

/**
 * Streaming operator that writes a DebugRequest to a stream.
 * @param stream The stream to write on.
 * @param debugRequest The DebugRequest object.
 * @return The stream.
 */
MessageQueue& operator<<(MessageQueue& stream, const DebugRequest& debugRequest);

class DebugRequestTable
{
private:
  /**
   * Default constructor.
   * No other instance of this class is allowed except the one accessible via getDebugRequestTable
   * therefore the constructor is private. */
  DebugRequestTable();
  
  /* Copy constructor.
   * Copying instances of this class is not allowed
   * therefore the copy constructor is private. */
  DebugRequestTable(const DebugRequestTable&) {}
  
  /*
   * only a process is allowed to create the instance.
   */
  friend class Process;
  friend class Controller;
  
  enum { maxNumberOfDebugRequests = 1000 };
  
public:
  
  /**
   * The Debug Key Table
   */
  DebugRequest debugRequests[maxNumberOfDebugRequests];
  int currentNumberOfDebugRequests;
  
  /** */
  void addRequest(const DebugRequest& debugRequest, bool force = false);
  
  /** */
  void removeRequest(const char* description);
  
  bool isActive(const char* name) const
  {
    if(name == lastName)
      return lastIndex < currentNumberOfDebugRequests && debugRequests[lastIndex].enable;
    else if(currentNumberOfDebugRequests == 0)
      return false;
    else
    {
      lastName = name;
      std::unordered_map<const char*, int>::const_iterator iter = nameToIndex.find(name);
      if(iter != nameToIndex.end())
      {
        lastIndex = iter->second;
        return lastIndex < currentNumberOfDebugRequests && debugRequests[lastIndex].enable;
      }
      for(lastIndex = 0; lastIndex < currentNumberOfDebugRequests; ++lastIndex)
        if(debugRequests[lastIndex].description == name)
        {
          nameToIndex[name] = lastIndex;
          return debugRequests[lastIndex].enable;
        }
      nameToIndex[name] = lastIndex;
      return false;
    }
  }
  
  /** */
  void disable(const char* name);
  
  /** */
  bool notYetPolled(const char* name);
  
  /** */
  void removeAllRequests()
  {
    currentNumberOfDebugRequests = 0;
  }
  
  /** */
  bool poll;
  
  int pollCounter;
  
  const char* alreadyPolledDebugRequests[maxNumberOfDebugRequests];
  int alreadyPolledDebugRequestCounter;
  mutable const char* lastName;
  
  mutable int lastIndex;
  mutable std::unordered_map<const char*, int> nameToIndex;

};