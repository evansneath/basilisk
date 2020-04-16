/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <vector>
#include <stdint.h>
#include <set>
#include <thread>
#include <mutex>
#include <iostream>
#include "architecture/system_model/sys_process.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/message_logger.h"
#include "utilities/bskLogging.h"

/*! \addtogroup SimArchGroup Simulation Architecture Classes
 *  This architecture group contains the source used to drive/schedule/interface
 *  with the simulation.
 * @{
 */

typedef enum varAccessType {
    messageBuffer = 0,
    logBuffer = 1
}VarAccessType;

class SimThreadExecution
{
public:
    SimThreadExecution();
    SimThreadExecution(uint64_t threadIdent, uint64_t currentSimNanos=0, messageLogger *logger=nullptr);    //!< Constructor for a given sim thread
    ~SimThreadExecution();   //!< Destructor for given sim thread
    void updateNewStopTime(uint64_t newStopNanos) {stopThreadNanos = newStopNanos;}
    void clearProcessList() {processList.clear();}
    void addNewProcess(SysProcess* newProc) {processList.push_back(newProc);}
    bool threadActive() {return this->threadRunning;};
    bool threadValid() {return (!this->terminateThread);}
    void killThread() {this->terminateThread=true;}
    void lockThread();
    void unlockThread();
    void lockMaster();
    void unlockMaster();
    void StepUntilStop();  //!< Step simulation until stop time uint64_t reached
    void SingleStepProcesses(int64_t stopPri=-1); //!< Step only the next Task in the simulation
public:
    uint64_t currentThreadNanos;  //!< Current simulation time available at thread
    uint64_t stopThreadNanos;   //!< Current stop conditions for the thread
    int64_t stopThreadPriority; //!< Current stop priority for thread
    uint64_t threadID;          //!< Identifier for thread
    std::thread *threadContext;
    uint64_t CurrentNanos;  //!< [ns] Current sim time
    uint64_t NextTaskTime;  //!< [ns] time for the next Task
    int64_t nextProcPriority;  //!< [-] Priority level for the next process
    messageLogger *messageLogs;  //!< -- Message log data
private:
    bool threadRunning;            //!< Flag that will allow for easy concurrent locking
    bool terminateThread;          //!< Flag that indicates that it is time to take thread down
    std::mutex masterThreadLock;   //!< Lock that ensures master thread won't proceed
    std::mutex selfThreadLock;     //!< Lock that ensures this thread only reaches allowed time
    std::vector<SysProcess*> processList;  //!< List of processes associated with thread
};

//! The top-level container for an entire simulation
class SimModel
{
public:
    SimModel();  //!< The SimModel constructor
    ~SimModel();  //!< SimModel destructor
    void selfInitSimulation();  //!< Method to initialize all added Tasks
    void crossInitSimulation();  //!< Method to initialize all added Tasks
    void resetInitSimulation();  //!< Method to reset all added tasks
    void StepUntilStop(uint64_t SimStopTime, int64_t stopPri); //!< Method to step threads
    void PrintSimulatedMessageData();  //!< Print out all messages that have been created
    void addNewProcess(SysProcess *newProc);
    void clearProcsFromThreads();
    void resetThreads(uint64_t threadCount);
    void deleteThreads();
    uint64_t IsMsgCreated(std::string MessageName);
    uint64_t GetWriteData(std::string MessageName, uint64_t MaxSize,
                          void *MessageData, VarAccessType logType = messageBuffer,
                          uint64_t LatestOffset=0);  //!< Grab a particular MessageName with MaxSize limited
    void ResetSimulation();  //!< Reset simulation back to zero
    void WriteMessageData(std::string MessageName, uint64_t MessageSize,
                          uint64_t ClockTime, void *MessageData);  //!< Write in a single message
    void CreateNewMessage(std::string processName, std::string MessageName,
        uint64_t MessageSize, uint64_t NumBuffers=2,
        std::string messageStruct = "");  //!< Create a new message for use
    void logThisMessage(std::string messageName, uint64_t messagePeriod=0);
    int64_t getNumMessages();  //!< Get number of messages in simulation
    std::string getMessageName(int64_t messageID, int64_t buffSelect);  //!< Get name for specified message ID
    MessageIdentData getMessageID(std::string messageName);  //!< Get the ID associated with message name
    void populateMessageHeader(std::string messageName,
        MessageHeaderData* headerOut);  //!< Get header data associated with msg
    std::set<std::string> getUniqueMessageNames();
    std::set<std::pair<long int, long int>> getMessageExchangeData(std::string messageName,
        std::set<unsigned long> procList  = std::set<unsigned long>());
    void terminateSimulation();
    std::set<long int> findChildModules(std::string procName);
    BSKLogger bskLogger;                      //!< -- BSK Logging

public:
    std::vector<SysProcess *> processList;  //!< -- List of processes we've created
    std::vector<SimThreadExecution*> threadList;  //!< -- Array of threads that we're running on
    std::string SimulationName;  //!< -- Identifier for Sim
    messageLogger messageLogs;  //!< -- Message log data
    uint64_t NextTaskTime;      //!< ns Current simulation nanos across all of the threads
    uint64_t CurrentNanos;      //!< ns Current simulation nanos across all of the threads
};

/*! @} */
#endif /* _SimModel_H_ */
